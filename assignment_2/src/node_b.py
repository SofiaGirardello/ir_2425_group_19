#!/usr/bin/env python3

"""
@file   : node_c.py
@brief  : Pick-and-place action server.

@details: This node implements an action server to handle pick-and-place tasks.
          It receives goals containing object poses and placement poses, and
          uses MoveIt! to control the robot's arm to perform the operation.

@date   : 2024-1-20
@authors: Caduceo Andrea, Girardello Sofia, Gizzarone Manuel
"""

import rospy
import actionlib
from assignment_2.msg import PickPlaceAction, PickPlaceFeedback, PickPlaceResult
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, Detach
from collections import deque
import traceback


class PickPlaceServer:
    def __init__(self):
        rospy.init_node('node_c_pick_place_server')

        # Initialize MoveIt components
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander('arm')
        self.gripper_group = MoveGroupCommander('gripper')

        # Initialize move_base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server!")

        # Gazebo link attacher services
        rospy.wait_for_service('/link_attacher_node/attach')
        rospy.wait_for_service('/link_attacher_node/detach')
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Detach)

        # Initialize action server
        self.server = actionlib.SimpleActionServer('pick_place', PickPlaceAction, self.execute, False)
        self.server.start()
        rospy.loginfo("Pick-and-place action server is ready.")

        # Queue for handling goals
        self.goal_queue = deque()
        self.processing_goal = False

    def execute(self, goal):
        """
        Executes the pick-and-place action for the received goal.
        """
        feedback = PickPlaceFeedback()
        result = PickPlaceResult()

        try:
            self.queue_pick_place_goal(goal)
            self.process_next_goal()

            result.success = True
            result.message = "Pick-and-place operation completed successfully."
            self.server.set_succeeded(result)

        except Exception as e:
            rospy.logerr(f"Error during pick-and-place: {e}")
            rospy.logerr(traceback.format_exc())
            result.success = False
            result.message = f"Operation failed: {e}"
            self.server.set_aborted(result)

    def queue_pick_place_goal(self, goal):
        rospy.loginfo("Queuing pick-and-place goal.")
        self.goal_queue.append(goal)

    def process_next_goal(self):
        if self.processing_goal:
            return

        while self.goal_queue:
            self.processing_goal = True
            goal = self.goal_queue.popleft()
            self.execute_pick_and_place(goal)
        self.processing_goal = False

    def execute_pick_and_place(self, goal):
        feedback = PickPlaceFeedback()

        try:
            feedback.current_step = "Executing pick-and-place task"
            self.server.publish_feedback(feedback)
            self.pick_and_place(goal.object_pose, goal.place_pose)
            feedback.current_step = "Pick-and-place task completed"
            self.server.publish_feedback(feedback)

        except Exception as e:
            rospy.logerr(f"Error during pick-and-place execution: {e}")
            rospy.logerr(traceback.format_exc())

    def pick_and_place(self, object_pose, place_pose):
        rospy.loginfo(f"Picking object at: {object_pose}")
        rospy.loginfo(f"Placing object at: {place_pose}")

        # Move arm to home position
        self.arm_group.set_named_target('home')
        self.arm_group.go(wait=True)

        # Move above the object
        above_object_pose = Pose()
        above_object_pose.position.x = object_pose.position.x
        above_object_pose.position.y = object_pose.position.y
        above_object_pose.position.z = object_pose.position.z + 0.1
        above_object_pose.orientation.w = 1.0
        self.arm_group.set_pose_target(above_object_pose)
        self.arm_group.go(wait=True)

        # Move to the object and grasp it
        self.arm_group.set_pose_target(object_pose)
        self.arm_group.go(wait=True)

        rospy.loginfo("Attaching object to gripper...")
        attach_req = Attach()
        attach_req.model_name_1 = "robot"
        attach_req.link_name_1 = "arm_7_link"
        attach_req.model_name_2 = "target_object"
        attach_req.link_name_2 = "link"
        self.attach_srv(attach_req)

        rospy.loginfo("Closing gripper...")
        self.gripper_group.set_named_target('close')
        self.gripper_group.go(wait=True)

        # Move back above the object
        self.arm_group.set_pose_target(above_object_pose)
        self.arm_group.go(wait=True)

        # Navigate to the placement position
        self.navigate_to_place_position(place_pose)

        # Place the object
        self.arm_group.set_pose_target(place_pose)
        self.arm_group.go(wait=True)

        rospy.loginfo("Opening gripper...")
        self.gripper_group.set_named_target('open')
        self.gripper_group.go(wait=True)

        rospy.loginfo("Detaching object from gripper...")
        detach_req = Detach()
        detach_req.model_name_1 = "robot"
        detach_req.link_name_1 = "arm_7_link"
        detach_req.model_name_2 = "target_object"
        detach_req.link_name_2 = "link"
        self.detach_srv(detach_req)

    def navigate_to_place_position(self, place_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = place_pose.position.x
        goal.target_pose.pose.position.y = place_pose.position.y
        goal.target_pose.pose.orientation = place_pose.orientation

        rospy.loginfo(f"Navigating to placement position at ({place_pose.position.x}, {place_pose.position.y})...")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully reached the placement position!")
        else:
            rospy.logerr("Failed to navigate to the placement position.")


if __name__ == '__main__':
    try:
        PickPlaceServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

      
