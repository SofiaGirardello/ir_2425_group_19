#!/usr/bin/env python

"""
@file    pick_object_server.py
@brief   Pick Action Server Node for Object Manipulation

@details
This ROS node implements an action server that performs pick operations using
a robotic manipulator controlled via MoveIt!. It receives goals containing
object poses and corresponding AprilTag IDs, computes appropriate grasp and
approach poses based on the object type, and executes the motion plan to pick
the object.

The node also handles object-specific pose adjustments, gripper control, and
attaches the object in the Gazebo simulation environment using the
`gazebo_ros_link_attacher` services.

Supported object types are:
  - Hexagons (IDs: 1, 2, 3)
  - Cubes (IDs: 4, 5, 6)
  - Triangles (IDs: 7, 8, 9)

@date    2025-06-10
@authors Andrea Caduceo, Sofia Girardello, Manuel Gizzarone
"""

import rospy
import actionlib
from assignment_2.msg import PickObjectAction, PickObjectFeedback, PickObjectResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Quaternion
from gazebo_ros_link_attacher.srv import Attach
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix, translation_matrix, concatenate_matrices, quaternion_multiply
from collections import deque
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander ,RobotTrajectory
from copy import deepcopy
from assignment_2.objectCollision import AddCollisionObject
import math
import tf
import tf.transformations as tft

class PickObjectServer:
    def __init__(self):
        """
        Initializes the PickObjectServer node.
        Sets up MoveIt interfaces, Gazebo attachment services, and the action server.
        """

        rospy.init_node('pick_object_server')

        # Collision manager to handle object collision models
        self.collision_obj_manager = AddCollisionObject()

        # Initialize MoveIt components
        self.robot = RobotCommander()
        self.arm_group = MoveGroupCommander('arm_torso')
        self.gripper_group = MoveGroupCommander('gripper')

        # Robot frames
        self.robot_frame = 'base_link'
        self.end_effector_link = 'gripper_link'

        # Gazebo link attacher services
        rospy.wait_for_service('/link_attacher_node/attach')
        rospy.wait_for_service('/link_attacher_node/detach')
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach) 

        # Initialize the pick action server
        self.pick_action_server = actionlib.SimpleActionServer('pick_object', PickObjectAction, self.execute_pick, False)
        self.pick_action_server.start()
        rospy.loginfo("Pick action server is ready.")

        # ID categories for each object shape
        self.list_hexagon = [1, 2, 3]
        self.list_cube = [4, 5, 6]
        self.list_triangular = [7, 8, 9]


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

    
    def execute_pick(self, goal):
        """
        Callback for the action server: execute the pick action for the object pose and ID.
        """

        # Move to default home joint configuration
        home_joints_position = [0.34, 0.07, 0.34, -3.13, 1.31, 1.58, -0.0, 0.0]
        self.arm_group.set_joint_value_target(home_joints_position)
        self.arm_group.go(wait = True)

        feedback = PickObjectFeedback()
        result = PickObjectResult()

        try:
            tag_id = goal.id
            target_pose = goal.object_pose

            # Flag variable to enable different orientation graspings
            second_trial = False

            # Compute poses for pre-grasp and grasp
            above_pick_pose, pick_pose = self.process_pose(tag_id, target_pose, second_trial)

            done_above = self.move_arm_to_pose(above_pick_pose)
            rospy.sleep(1)

            # Retry with adjusted pose if first attempt fails
            if not done_above and (tag_id in self.list_cube or tag_id in self.list_hexagon):
                second_trial = True
                above_pick_pose, pick_pose = self.process_pose(tag_id, target_pose, second_trial)
                done_above = self.move_arm_to_pose(above_pick_pose)

            if done_above:

                done_pick = self.move_arm_to_pose(pick_pose)

                if done_pick:
                    # Remove collision object
                    self.collision_obj_manager.remove_collision_object(tag_id)

                    # Close the gripper
                    rospy.loginfo("Closing the gripper...")
                    gripper_joint_position = [0.015, 0.015]  
                    self.gripper_group.set_joint_value_target(gripper_joint_position)
                    self.gripper_group.go(wait = True)

                    # Attach the object to the gripper using Gazebo_ros_link_attacher
                    rospy.loginfo("Attaching object to gripper...")
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
                        self.attach_srv(attach_req.model_name_1, attach_req.link_name_1, attach_req.model_name_2, attach_req.link_name_2)
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Failed to attach object: {e}")
                        result.success = False
                        self.pick_action_server.set_succeeded(result)
                        return

                    # Return to above position after picking
                    self.move_arm_to_pose(above_pick_pose)

                    result.success = True
                else:
                    result.success = False

                # Return to above position after picking
                self.arm_group.set_joint_value_target(home_joints_position)
                self.arm_group.go(wait = True)
            else:
                result.success = False

            self.pick_action_server.set_succeeded(result)

        except Exception as e:
            rospy.logerr(f"Error during pick operation! Error: {e}")
            result.success = False
            self.pick_action_server.set_succeeded(result)
    


    def process_pose(self, tag_id, target_pose, second_trial):
        """
        Computes the actual grasp and pre-grasp poses depending on object infos:
        -ID of the AprilTag on the object;
        -Original pose from detection.
        -Whether this is a retry attempt.
        """

        pick_pose = deepcopy(target_pose)

        # Extract original quaternion
        q_orig = [
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
        ]

        # Hexagon logic: rotation around Z
        if tag_id in self.list_hexagon:
            
            # Apply a 90° rotation around Z
            q_rot = quaternion_from_euler(0, 0, math.pi / 2)
            q_new = quaternion_multiply(q_rot, q_orig)

            # Optional second trial: additional rotation
            if second_trial:
                # Apply a -60° rotation around Z
                q_rot = quaternion_from_euler(0, 0, -math.pi / 3)
                q_new_2 = quaternion_multiply(q_rot, q_new)

            # Applica il nuovo orientamento a pick_pose
            pick_pose.pose.orientation.x = q_new[0]
            pick_pose.pose.orientation.y = q_new[1]
            pick_pose.pose.orientation.z = q_new[2]
            pick_pose.pose.orientation.w = q_new[3]

            pick_pose.pose.position.z += 0.18  
            above_pick = deepcopy(pick_pose)
            above_pick.pose.position.z += 0.1

        # Triangle logic: translation in Z, rotation around X
        elif tag_id in self.list_triangular:
            
            # Build transformation matrix from pose (translation + rotation)
            translation = tft.translation_matrix([
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z
            ])

            rotation = tft.quaternion_matrix([
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w
            ])

            tag_pose_matrix = tft.concatenate_matrices(translation, rotation)

            # Offset in tag's frame: move slightly along Z to grasp the flat side
            inward_offset_distance = -0.035 # Negative values to move in the opposite direction of Z
            local_offset = [0, 0, inward_offset_distance, 1]

            # Transform this offset into base_link frame
            transformed_offset = tag_pose_matrix.dot(local_offset)

            # Set new pick pose position with upward offset
            pick_pose.pose.position.x = transformed_offset[0]
            pick_pose.pose.position.y = transformed_offset[1]
            pick_pose.pose.position.z = transformed_offset[2] + 0.26

            # Rotate of 45° around X to align with inclined face
            rot_x_45 = tft.euler_matrix(math.radians(45), 0, 0)
            new_rotation_matrix = tft.concatenate_matrices(rotation, rot_x_45)
            q_new = tft.quaternion_from_matrix(new_rotation_matrix)

            # Assing the computed orientation
            pick_pose.pose.orientation.x = q_new[0]
            pick_pose.pose.orientation.y = q_new[1]
            pick_pose.pose.orientation.z = q_new[2]
            pick_pose.pose.orientation.w = q_new[3]

            # Generate above-pick pose for safe descent
            above_pick  = deepcopy(pick_pose)
            above_pick.pose.position.z += 0.1

        # Cube logic: no translation/rotation required
        elif tag_id in self.list_cube:
            
            # Optional second trial: additional rotation of 90° around Z for alternative grasp
            if second_trial:
                q_rot = quaternion_from_euler(0, 0, math.pi / 2)
                q_new = quaternion_multiply(q_rot, q_orig)

                pick_pose.pose.orientation.x = q_new[0]
                pick_pose.pose.orientation.y = q_new[1]
                pick_pose.pose.orientation.z = q_new[2]
                pick_pose.pose.orientation.w = q_new[3]
            
            pick_pose.pose.position.z += 0.23
            above_pick  = deepcopy(pick_pose)
            above_pick.pose.position.z += 0.12

        return above_pick, pick_pose

            
if __name__ == '__main__':
    try:
        PickObjectServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
