#!/usr/bin/env python

"""
@file   : place_routine.py
@brief  : ROS node for handling the "place" step in a pick-and-place task.

@description:
    This node does the following:
    - Requests line coefficients from a ROS service (/straight_line_srv).
    - Computes a set of placement poses along a line.
    - Publishes those poses to a topic (/placement_positions).
    - Provides an action server to execute the actual placement of an object using MoveIt!.
    - Manages object detachment using gazebo_ros_link_attacher.

@date   : 2025-06-10
@authors: Caduceo Andrea, Girardello Sofia, Gizzarone Manuel 
"""

import rospy
from tiago_iaslab_simulation.srv import Coeffs
from geometry_msgs.msg import Point, Quaternion, PoseStamped, PoseArray, Pose
from assignment_2.msg import PlaceObjectAction, PlaceObjectFeedback, PlaceObjectResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander ,RobotTrajectory
import tf.transformations as tft
from gazebo_ros_link_attacher.srv import Attach
import actionlib
from copy import deepcopy
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix, translation_matrix, concatenate_matrices, quaternion_multiply
import tf

class PlaceRoutine:
    """
    Class implementing the place phase of a pick-and-place task.
    """

    def __init__(self):

        rospy.init_node('place_routine')

        # Service name to obtain placement line coefficients
        self.service_name = '/straight_line_srv'
        rospy.loginfo("Waiting for /straight_line_srv service...")
        rospy.wait_for_service(self.service_name)
        rospy.loginfo("Service available. Sending request...")

        # Publisher for computed placement poses
        self.placement_pub = rospy.Publisher('/placement_positions', PoseArray, queue_size=10)

        # TF listener to transform poses
        self.tf_tag10_robot = tf.TransformListener()

        # Initialize MoveIt components
        self.robot = RobotCommander()
        self.arm_group = MoveGroupCommander('arm_torso')
        self.robot_frame = 'base_link'
        self.end_effector_link = 'gripper_link'
        self.gripper_group = MoveGroupCommander('gripper')

        # Gazebo link attacher services
        rospy.wait_for_service('/link_attacher_node/attach')
        rospy.wait_for_service('/link_attacher_node/detach')
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach) 

        # Initialize the place action server
        self.place_action_server = actionlib.SimpleActionServer('place_object', PlaceObjectAction, self.execute_place, False)
        self.place_action_server.start()
        rospy.loginfo("Place action server is ready.")

        # Retrieve line coefficients from service
        self.coeffs = self.get_line_coefficients()

        # IDs for different object shapes
        self.list_hexagon = [1, 2, 3]
        self.list_cube = [4, 5, 6]
        self.list_triangular = [7, 8, 9]

        # Publish the placement positions once there's a subscriber
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Check ifnode_c there is at least a subscriber to the topic before publishing
            if self.placement_pub.get_num_connections() > 0:
                self.publish_positions() 
                break
            rate.sleep()

    def get_line_coefficients(self):
        """
        Request coefficients from the /straight_line_srv service.
        These are used to compute the placement line (y = mx + q).
        """

        try:
            get_coeffs = rospy.ServiceProxy(self.service_name, Coeffs)
            response = get_coeffs(ready=True)
            rospy.loginfo(f"Received coefficients: m = {response.coeffs[0]}, q = {response.coeffs[1]}")
            return response.coeffs
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def publish_positions(self):
        """
        Compute placement poses along a line and publish them to /placement_positions.
        """
        
        pose_array = PoseArray()
        pose_array.header.frame_id = "tag_10"
        pose_array.header.stamp = rospy.Time.now()

        if self.coeffs:
            m, q = self.coeffs[0], self.coeffs[1]
            n_points = 10

            # Adjust coefficients if values fall outside useful range:

            # Ensure to not exceed the reachable workspace
            if q >= 0.15: 
                q -= 0.1
            
            # Ensure to not cover the tag_10 frame
            if q < 0.05:
                q += 0.06

            # Horizontal spacing between consecutive placements
            x_spacing = 0.1 / m

            for i in range(n_points): 
                x = (i)*x_spacing
                y = m * x + q 

                pose = Pose()
                pose.position = Point(x, y, 0.0)
                pose.orientation = Quaternion(0, 0, 0, 1)

                pose_array.poses.append(pose)

            self.placement_pub.publish(pose_array)
            rospy.loginfo(f"Published placement positions.")
        else:
            rospy.logwarn("No coefficients provided, skipping publish.")


    def execute_place(self, goal):
        """
        Action server callback that handles placing an object.
        Moves the robot to a computed pose and releases the object using Gazebo's link attacher.
        """

        feedback = PlaceObjectFeedback()
        result = PlaceObjectResult()
        home_joints_position = [0.34, 0.07, 0.34, -3.13, 1.31, 1.58, -0.0, 0.0]

        try:
            goal.place_pose.header.stamp = rospy.Time(0)

            # Transform pose from tag frame to base frame
            try:
                place_pose = self.tf_tag10_robot.transformPose(self.robot_frame, goal.place_pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(f"Error during transform from tag_10 to {self.robot_frame}! Error: {e}")
                return

            tag_id = goal.id
            place_pose.pose.position.z += 0.26

            q_orig = [
            place_pose.pose.orientation.x,
            place_pose.pose.orientation.y,
            place_pose.pose.orientation.z,
            place_pose.pose.orientation.w
            ]

            # Rotate the pose 180° around Z axis
            q_rot = quaternion_from_euler(0, 0, math.pi)

            # Compute the new orientation
            q_new = quaternion_multiply(q_rot, q_orig)

            # Apply the new orientation as placement position
            place_pose.pose.orientation.x = q_new[0]
            place_pose.pose.orientation.y = q_new[1]
            place_pose.pose.orientation.z = q_new[2]
            place_pose.pose.orientation.w = q_new[3]
  
            # Move above the target pose first
            above_place_pose = deepcopy(place_pose)
            above_place_pose.pose.position.z += 0.1

            done_above = self.move_arm_to_pose(above_place_pose)

            if done_above:
                done_place = self.move_arm_to_pose(place_pose)

            # Detach the object from the gripper using Gazebo_ros_link_attacher
            rospy.loginfo("Detaching object from gripper...")
            attach_req = Attach()
            attach_req.model_name_1 = "tiago"
            attach_req.link_name_1 = "arm_7_link"
            attach_req.link_name_2 = f"tag{tag_id}"
            if tag_id in self.list_hexagon: 
                attach_req.model_name_2 = f"Hexagon_{tag_id}" 
                # Additional check since in Gazebo, the first hexagon model is named "Hexagon" (without "_1" suffix),
                # while subsequent ones are named "Hexagon_2", "Hexagon_3", etc.
                if tag_id == 1: 
                    attach_req.model_name_2 = "Hexagon" 
            elif tag_id in self.list_triangular:
                attach_req.model_name_2 = f"Triangle_{tag_id}"
                # Additional check 
                if tag_id == 7: 
                    attach_req.model_name_2 = "Triangle"
            else:
                attach_req.model_name_2 = f"cube_{tag_id}"
                # Additional check 
                if tag_id == 4: 
                    attach_req.model_name_2 = "cube"
                
            try:
                self.detach_srv(attach_req.model_name_1, attach_req.link_name_1, attach_req.model_name_2, attach_req.link_name_2)
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to attach object: {e}")
                result.success = False
                self.pick_action_server.set_succeeded(result)
                return

            rospy.sleep(0.1)

            # Open the gripper
            rospy.loginfo("Opening the gripper...")
            gripper_joint_position = [0.04, 0.04]  # Adjust this value depending on your gripper's range
            self.gripper_group.set_joint_value_target(gripper_joint_position)
            self.gripper_group.go(wait = True)
                
            # Move back to above position
            rospy.sleep(1)
            self.move_arm_to_pose(above_place_pose)
            rospy.sleep(2)

            # Move to home position
            self.arm_group.set_joint_value_target(home_joints_position)
            self.arm_group.go(wait = True)

            if done_above and done_place:
                result.success = True
            else:
                result.success = False

            self.place_action_server.set_succeeded(result)

        except Exception as e:
            rospy.logerr(f"Error during place operation! Error: {e}")
            result.success = False
            self.place_action_server.set_succeeded(result)
    
    
    def move_arm_to_pose(self, target_pose):
        """
        Move the robot arm to the target pose using MoveIt!'s planning interface.
        """

        rospy.loginfo(f"Starting moving towards target pose.")

        self.arm_group.set_pose_reference_frame(self.robot_frame)
        self.arm_group.set_end_effector_link(self.end_effector_link)

        # Set the current joint state as the start state for planning
        self.arm_group.set_start_state_to_current_state()

        # Planning parameters
        self.arm_group.set_goal_tolerance(0.005)  # Relax goal tolerance
        self.arm_group.set_num_planning_attempts(10)  # Increase planning attempts

        self.arm_group.set_pose_target(target_pose)

        # Compute Cartesian path to target
        waypoints = [target_pose.pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        if fraction < 0.9:
            rospy.logwarn("Target pose is not fully reachable. Planning fraction: {:.2f}".format(fraction))
            return False
        else:
            success = self.arm_group.execute(plan, wait = True)
            rospy.loginfo(f"Successfully moved to target pose.")
            self.arm_group.set_start_state_to_current_state()

        self.arm_group.clear_pose_targets()
        return success


if __name__ == '__main__':
    try:
        PlaceRoutine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

