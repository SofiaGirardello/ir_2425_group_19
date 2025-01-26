#!/usr/bin/env python

"""
@file   : node_c.py
@brief  : Pick-and-place action server.

@details: This node implements an action server to handle pick-and-place tasks.
          It receives goals containing object poses and placement poses, and
          uses MoveIt! to control the robot's arm to perform the operation.

@date   : 2024-1-20
@authors : Caduceo Andrea, Girardello Sofia, Gizzarone Manuel
"""

import rospy
import actionlib
from assignment_2.msg import PickPlaceAction, PickPlaceFeedback, PickPlaceResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Quaternion
from gazebo_ros_link_attacher.srv import Attach
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from collections import deque
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander ,RobotTrajectory
from assignment_2.objectCollision import AddCollisionObject

class PickPlaceServer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('node_c_pick_place_server')

        # Initialize MoveIt components
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander('arm_torso')  # Move group for the arm
        self.gripper_group = MoveGroupCommander('gripper')
        self.arm_group.set_end_effector_link('gripper_link')
        self.home_position = None

        # Initialize the move_base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server!") 

        self.collision_object = AddCollisionObject()

        # Gazebo link attacher services
        rospy.wait_for_service('/link_attacher_node/attach')
        rospy.wait_for_service('/link_attacher_node/detach')
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach) 

        # Initialize the action server
        self.server = actionlib.SimpleActionServer('pick_place', PickPlaceAction, self.execute, False)
        self.server.start()
        rospy.loginfo("Pick-and-place action server is ready.")

        # Queue for storing incoming goals
        #self.goal_queue = deque()  # Use deque for efficient pop from left
        #self.processing_goal = False  # Flag to track goal processing status

        self.navigate_to_pickup_table(8.8, 0.0, 0.0)

        # Pre-move robot to a midway point
        # self.navigate_to_pickup_table(8.8, -3.0, -1.57)

        # Debugging the arm's position
        # current_pose = self.arm_group.get_current_pose().pose
        # rospy.loginfo(f"Current arm pose: {current_pose}")

        # Debugging joint values
        # current_joints = self.arm_group.get_current_joint_values()
        # rospy.loginfo(f"Current joint values: {current_joints}")


        # Set the current joint state as the start state
        self.arm_group.set_start_state_to_current_state()

        self.target_joint_values = [
        0.34,
        0.07,     # Joint 1: Base rotation (e.g., 0.5 radians)
        0.34,    # Joint 2: Shoulder (e.g., -0.3 radians)
        -3.13,     # Joint 3: Elbow (e.g., 0.8 radians)
        1.31,    # Joint 4: Wrist pitch (e.g., -1.2 radians)
        1.58,     # Joint 5: Wrist roll (e.g., 0.6 radians)
        -0.0,    # Joint 6: Wrist yaw (e.g., -0.4 radians)
        0.0    # Joint 7: Gripper (e.g., 0.2 radians or a specific angle for the gripper)     
        ]

        # Set the target joint values
        self.arm_group.set_joint_value_target(self.target_joint_values)

        self.arm_group.go(wait=True)

        self.navigate_to_pickup_table(9.2, -2.0, 3.14)

        tilt_angle = -1 # Angle in rad, negative for downward inclinations
        self.tilt_head(tilt_angle)
        rospy.sleep(3)
        self.tilt_head(0)
        # Move robot in front of the pick-up table 
        self.navigate_to_pickup_table(8.8, -3.0, 3.14)

        self.tilt_head(tilt_angle/2)


    # CHeck reachable workspace
    def move_arm_to_pose(self, target_pose):
        """
        Move the robot arm to the target pose using MoveIt!'s planning interface.
        """
        self.arm_group.set_pose_reference_frame('base_link')
        self.arm_group.set_pose_target(target_pose)
          

        self.plan = self.arm_group.plan()
        # Plan the motion and check for collisions
        success = self.arm_group.go(wait=True)

        if success:
            rospy.loginfo(f"Successfully moved to target pose: {target_pose}")
        else:
            rospy.logerr("Failed due to plan motion.")
            self.arm_group.set_joint_value_target(self.target_joint_values)
            self.arm_group.go(wait=True)
        
        self.arm_group.clear_pose_targets()

    def navigate_to_pickup_table(self, x, y, yaw):
        """
        Navigate the robot to a specific pickup table position.
        :param x: X-coordinate of the table
        :param y: Y-coordinate of the table
        :param yaw: Orientation (yaw) in radians
        """

        # Define the goal for move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # Convert yaw to quaternion for orientation
        quaternion = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        # Send the goal to the MoveBase action server
        rospy.loginfo(f"Navigating to pickup table: x={x}, y={y}, yaw={yaw}")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        # Check result status
        if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully reached the pickup table!")
        else:
            rospy.logerr("Navigation to pickup table failed.")

    def tilt_head(self, tilt_angle):
        """
        Tilts the robot's head to a specified angle for better camera positioning.
        Parameters:
            tilt_angle (float): The desired tilt angle in radians. Negative values tilt the head downward.
        """
        # Publisher for the topic that controls the head's movement 
        pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(1)
        
        # Create the JointTrajectory message to define the head's motion
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.joint_names = ['head_1_joint', 'head_2_joint']
        
        # Create a trajectory point to define the target position and duration
        point = JointTrajectoryPoint()
        point.positions = [0.0, tilt_angle]  
        point.time_from_start = rospy.Duration(3) 
        
        # Add the trajectory point to the message
        trajectory_msg.points.append(point)
        pub.publish(trajectory_msg)

    def execute(self, goal):
        """
        Executes the pick-and-place action for the received goal.
        """
        feedback = PickPlaceFeedback()
        result = PickPlaceResult()

        try:
            # Process the received goal directly
            #self.process_goal(goal)

            # Execute pick-and-place operation
            self.execute_pick_and_place(goal)

            # Send feedback only after the placing operation is complete
            feedback.current_step = "Pick-and-place task completed"
            self.server.publish_feedback(feedback)

            # Final success
            result.success = True
            result.message = "Pick-and-place operation completed successfully."
            self.server.set_succeeded(result)

        except Exception as e:
            # Handle any errors
            rospy.logerr(f"Error during pick-and-place: {e}")
            result.success = False
            result.message = f"Operation failed: {e}"
            self.server.set_aborted(result)
    
    def process_goal(self, goal):
        """
        Processes a pick-and-place goal directly (no queuing).
        """
        if hasattr(goal, 'id'):
            rospy.loginfo(f"Picking object identified by AprilTag ID: {goal.id}")
        else:
            rospy.logwarn("Goal does not contain a valid AprilTag ID.")

        # Add the logic to execute the pick-and-place operation here
        rospy.loginfo("Executing pick-and-place operation...")
        # Example placeholder: Replace with your pick-and-place logic
        self.execute_pick_and_place(goal)
    
    def execute_pick_and_place(self, goal):
        """
        Executes the pick-and-place task for the given goal.
        """
        try:
            
            self.pick_and_place(goal.id, goal.object_pose, goal.place_pose)

        except Exception as e:
            rospy.logerr(f"Error during pick-and-place: {e}")

    def pick_and_place(self, id, object_pose, place_pose):
        """
        Perform the pick-and-place task by manipulating the robot's arm and gripper.
        """
        # Implement the full pick-and-place task here using MoveIt
        rospy.loginfo(f"Picking object at: {object_pose}")
        rospy.loginfo(f"Placing object at: {place_pose}")

        # 1. Assign an initial configuration to the arm
        self.home_position = self.arm_group.get_current_joint_values()

        # 2. Move arm to a position above the object
        above_object_pose = Pose()
        above_object_pose.position.x = object_pose.pose.position.x 
        above_object_pose.position.y = object_pose.pose.position.y 
        above_object_pose.position.z = object_pose.pose.position.z + 0.5

        object_orientation = object_pose.pose.orientation
        _, _, yaw = euler_from_quaternion([object_orientation.x, object_orientation.y, object_orientation.z, object_orientation.w])
        
        # Convert Euler angles (roll, pitch, yaw) back to quaternion
        new_orientation = quaternion_from_euler(0, 0, yaw)

        # Assuming you want to set the orientation in a new pose
        above_object_pose.orientation.x = new_orientation[0]
        above_object_pose.orientation.y = new_orientation[1]
        above_object_pose.orientation.z = new_orientation[2]
        above_object_pose.orientation.w = new_orientation[3]
        # above_object_pose.orientation = object_pose.pose.orientation # Qua ce lo prendiamo in culo
        
        self.move_arm_to_pose(above_object_pose)
        rospy.loginfo("I am on above object pose")
        
        # Debugging the arm's position
        #current_pose = self.arm_group.get_current_pose().pose
        #rospy.loginfo(f"Current arm pose: {current_pose}")

        # 4. Remove the collision object
        self.collision_object.remove_collision_object(id)

        # 3. Grasping the object through a linear movement (move down to the object)
        object_pose.pose.position.z = object_pose.pose.position.z + 0.2
        self.move_arm_to_pose(object_pose)
        rospy.loginfo("I am on object pose")


        # 5. Attach the object to the gripper using Gazebo_ros_link_attacher
        rospy.loginfo("Attaching object to gripper...")
        attach_req = Attach()
        attach_req.model_name_1 = "tiago"
        attach_req.link_name_1 = "arm_7_link"
        attach_req.model_name_2 = f"tag{id}"
        attach_req.link_name_2 = f"tag{id}_link"
        self.attach_srv(attach_req.model_name_1, attach_req.link_name_1, attach_req.model_name_2, attach_req.link_name_2)

        # 6. Close the gripper
        rospy.loginfo("Closing the gripper...")
        gripper_joint_position = 0.0  # Adjust this value depending on your gripper's range
        self.gripper_group.set_joint_value_target([gripper_joint_position])
        self.gripper_group.go(wait=True)

        # 7. Return to the initial position
        self.move_arm_to_pose(above_object_pose)

        # 8. Move the arm to the home pose
        self.arm_group.set_start_state_to_current_state()
        self.arm_group.set_joint_value_target(self.home_position)
        self.arm_group.go(wait=True)

        # 10. Navigate to the placement position
        rospy.loginfo("Navigating to the placement position...")
        self.navigate_to_pickup_table(8.8, -2.0, 3.14)

        # 11. Place the object on the table
        rospy.loginfo(f"Placing object at: {place_pose.position.x}, {place_pose.position.y}")
        self.move_arm_to_pose(place_pose)

        # 12. Open the gripper
        rospy.loginfo("Opening the gripper...")
        gripper_joint_position = 0.1  # Adjust this value depending on your gripper's range
        self.gripper_group.set_joint_value_target([gripper_joint_position])
        self.gripper_group.go(wait=True)

        # 13. Detach the object from the gripper using Gazebo_ros_link_attacher
        rospy.loginfo("Detaching object from gripper...")
        detach_req = Attach()
        detach_req.model_name_1 = "tiago"
        detach_req.link_name_1 = "arm_7_link"
        detach_req.model_name_2 = f"tag{id}"
        detach_req.link_name_2 = f"tag{id}_link"
        self.detach_srv(detach_req)

        # 14. Move in front of the table 
        self.navigate_to_pickup_table(8.8, -3, 3.14)

        rospy.loginfo("Pick and place operation completed.")

if __name__ == '__main__':
    try:
        PickPlaceServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
