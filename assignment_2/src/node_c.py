#!/usr/bin/env python3

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
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from gazebo_ros_link_attacher.srv import Attach, Detach
from collections import deque

class PickPlaceServer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('node_c_pick_place_server')

        # Initialize MoveIt components
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander('arm')  
        self.gripper_group = MoveGroupCommander('gripper')

		# Initialize the move_base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server!") 

        # Gazebo link attacher services
        rospy.wait_for_service('/link_attacher_node/attach')
        rospy.wait_for_service('/link_attacher_node/detach')
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Detach) 

        # Initialize the action server
        self.server = actionlib.SimpleActionServer('pick_place', PickPlaceAction, self.execute, False)
        self.server.start()
        rospy.loginfo("Pick-and-place action server is ready.")

        # Queue for storing incoming goals
        self.goal_queue = deque()  # Use deque for efficient pop from left
        self.processing_goal = False  # Flag to track goal processing status

        # Pre-move robot to the pickup table
        self.move_to_pickup_table()

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
        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()

        # Check result status
        if move_base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully reached the pickup table!")
        else:
            rospy.logerr("Navigation to pickup table failed.")

    def execute(self, goal):
        """
        Executes the pick-and-place action for the received goal.
        """
        feedback = PickPlaceFeedback()
        result = PickPlaceResult()

        try:
            # Queue the received goal
            self.queue_pick_place_goal(goal)

            # Wait for and process the next goal in the queue
            self.process_next_goal()

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

    def queue_pick_place_goal(self, goal):
        """
        Queues a pick-and-place goal for processing.
        """
        rospy.loginfo("Queuing pick-and-place goal.")
        self.goal_queue.append(goal)

    def process_next_goal(self):
        """
        Processes the next goal in the queue if one exists.
        """
        if self.processing_goal:
            return  # If already processing a goal, return immediately

        if self.goal_queue:
            self.processing_goal = True  # Set processing flag to True

            # Pop the next goal from the queue
            goal = self.goal_queue.popleft()

            # Send the goal to the MoveIt for execution
            self.execute_pick_and_place(goal)

            # After the goal is processed, reset the processing flag and continue
            self.processing_goal = False
            self.process_next_goal()  # Process next goal if available

    def execute_pick_and_place(self, goal):
        """
        Executes the pick-and-place task for the given goal.
        """
        feedback = PickPlaceFeedback()
        try:
            feedback.current_step = "Executing pick-and-place task"
            self.server.publish_feedback(feedback)

            # Execute the pick-and-place operation
            self.pick_and_place(goal.object_pose, goal.place_pose)

            feedback.current_step = "Pick-and-place task completed"
            self.server.publish_feedback(feedback)

        except Exception as e:
            rospy.logerr(f"Error during pick-and-place: {e}")

    def pick_and_place(self, object_pose, place_pose):
        """
        Perform the pick-and-place task by manipulating the robot's arm and gripper.
        """
        # Implement the full pick-and-place task here using MoveIt
        rospy.loginfo(f"Picking object at: {object_pose}")
        rospy.loginfo(f"Placing object at: {place_pose}")

		# 1. Assign an initial configuration to the arm
        self.arm_group.set_named_target('home')
        self.arm_group.go(wait=True)

		# 2. Move arm to a position above the object
        above_object_pose = Pose()
        above_object_pose.position.x = object_pose.position.x
        above_object_pose.position.y = object_pose.position.y
        above_object_pose.position.z = object_pose.position.z + 0.1 
        above_object_pose.orientation.w = 1.0  # No rotation
        
        self.arm_group.set_pose_target(above_object_pose)
        self.arm_group.go(wait=True)

		# 3. Grasping the object through a linear movement (move down to the object)
        self.arm_group.set_pose_target(object_pose)
        self.arm_group.go(wait=True)

		# 4. Remove the collision object

		# 5. Attach the object to the gripper using Gazebo_ros_link_attacher
        rospy.loginfo("Attaching object to gripper...")
        attach_req = Attach()
        attach_req.model_name_1 = "robot"
        attach_req.link_name_1 = "arm_7_link"
        attach_req.model_name_2 = "target_object"
        attach_req.link_name_2 = "link"
        self.attach_srv(attach_req)

		# 6. Close the gripper
        rospy.loginfo("Closing the gripper to grasp the object...")
        self.gripper_group.set_named_target('close')
        self.gripper_group.go(wait=True)

		# 7. Return to the initial position
        			  						   self.arm_group.set_pose_target(above_object_pose)
self.arm_group.go(wait=True)

		# 8. Move the arm to an intermediate pose
        intermediate_pose = Pose()
        intermediate_pose.position.x = 0.5
        intermediate_pose.position.y = 0.0
        intermediate_pose.position.z = 0.8
        intermediate_pose.orientation.w = 1.0
        self.arm_group.set_pose_target(intermediate_pose)
        self.arm_group.go(wait=True)

		# 9. Move to a safe pose (e.g., fold the arm close to the body)
        safe_pose = Pose()
        safe_pose.position.x = 0.3
        safe_pose.position.y = 0.0
        safe_pose.position.z = 0.6
        safe_pose.orientation.w = 1.0
        self.arm_group.set_pose_target(safe_pose)
        self.arm_group.go(wait=True)

		# 10. Navigate to the placement position
rospy.loginfo("Navigating to the placement position...")

# Define the goal for move_base
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

# Set the position to the placement position
goal.target_pose.pose.position.x = place_pose.position.x
goal.target_pose.pose.position.y = place_pose.position.y
goal.target_pose.pose.position.z = 0.0  # Navigation doesn't use Z

# Use orientation from the place_pose
goal.target_pose.pose.orientation = place_pose.orientation

# Send the goal to the move_base server
rospy.loginfo(f"Sending navigation goal to: x={place_pose.position.x}, y={place_pose.position.y}")
move_base_client.send_goal(goal)
move_base_client.wait_for_result()

# Check if the robot successfully navigated
if move_base_client.get_state() == GoalStatus.SUCCEEDED:
    rospy.loginfo("Successfully reached the placement position!")
else:
    rospy.logerr("Failed to navigate to the placement position.")

		# 11. Place the object on the table
        rospy.loginfo(f"Placing object at: {place_pose.position.x}, {place_pose.position.y}")
        self.arm_group.set_pose_target(place_pose)
        self.arm_group.go(wait=True)

		# 12. Open the gripper
        rospy.loginfo("Opening the gripper...")
        self.gripper_group.set_named_target('open')
        self.gripper_group.go(wait=True)

		# 13. Detach the object from the gripper using Gazebo_ros_link_attacher
        rospy.loginfo("Detaching object from gripper...")
        detach_req = Detach()
        detach_req.model_name_1 = "robot"
        detach_req.link_name_1 = "arm_7_link"
        detach_req.model_name_2 = "target_object"
        detach_req.link_name_2 = "link"
        self.detach_srv(detach_req)

		rospy.loginfo("Pick and place operation completed.")


if __name__ == '__main__':
    try:
        PickPlaceServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



      
