#!/usr/bin/env python

"""
@file   : navigator.py
@brief  : A ROS node for controlling robot navigation among predefined waypoints and managing narrow corridor navigation. 
          The node sends navigation goals to the `move_base` action server, tracks the robot's progress, and adjusts its path 
          when obstacles are detected in narrow corridors.

@details:
    - The script defines two classes: 
      1. `WaypointNavigator`: Handles navigation among a set of predefined waypoints. It can also activate and deactivate 
         the `CorridorNavigator` when the robot needs to navigate through a narrow corridor.
      2. `CorridorNavigator`: Controls robot movement within a narrow corridor using laser scan data to avoid obstacles.
      
    - The system interacts with the `move_base` action server to send movement goals, and listens to laser scan data and pose 
      updates to ensure safe navigation.
    - The robot's movement in the corridor is handled by controlling its velocity, adjusting based on laser scan feedback.
    - The script listens to the `/research_status` topic to stop navigation once the required conditions are met.
    - It also provides updates on navigation progress via the `/navigation_status` topic.

@date   : 2024-12-23
@authors : Caduceo Andrea, Girardello Sofia, Gizzarone Manuel
"""

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from assignment_1.msg import research_status, info_navigation
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

"""
Class used to handle the navigation among the waypoints in order to cover all the map.
"""
# WAYPOINT NAVIGATOR CLASS
class WaypointNavigator:
    def __init__(self, use_corridor_navigator):
        """
        Initialize the WaypointNavigator class with MoveBaseAction client and waypoint definitions.
        """
        # Initialize the MoveBaseAction client 
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server!")

        # Waypoints definition
        self.waypoints = [
            {"x": 0.0, "y": 0.0, "yaw": -2},
            {"x": 1.0, "y": 0.0, "yaw": 0},
            {"x": 5.5, "y": 0.0, "yaw": 0},
            {"x": 8.5, "y": -1, "yaw": 3.14},
            {"x": 8.5, "y": -3, "yaw": 3.14},
            {"x": 11.5, "y": -3.3, "yaw": -0.5},
            {"x": 12.5, "y": -0.5, "yaw": 3.14},
            {"x": 12.5, "y": 0.5, "yaw": 1.57},
            {"x": 10, "y": 0.6, "yaw": 3.14}
        ]

        self.current_waypoint_index = 0
        
        # Use CorridorNavigator based on user input
        self.use_corridor_navigator = use_corridor_navigator
        self.corridor_navigator = None
        if self.use_corridor_navigator:
            self.corridor_navigator = CorridorNavigator(self)
            rospy.loginfo("CorridorNavigator enabled.")
        else:
            rospy.loginfo("CorridorNavigator disabled.")
        
        # Publisher to send navigation status updates
        self.publisher = rospy.Publisher('/navigation_status', info_navigation, queue_size=10)

        # Subscriber to listen for research status updates
        rospy.Subscriber('/research_status', research_status, self.research_status_cb)

    def send_next_waypoint(self):
        """
        Sends the robot to the next waypoint in the list.
        Handles transitions to and from narrow corridor navigation.
        """
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("All waypoints completed. Shutting down navigator node...")
            rospy.signal_shutdown("Navigator node stopped.")
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        x = waypoint["x"]
        y = waypoint["y"]
        yaw = waypoint["yaw"]
        
        # Publish the navigation status for the current waypoint
        info_msg = info_navigation()
        info_msg.x = x 
        info_msg.y = y
        self.publisher.publish(info_msg)

        # Handle narrow corridor navigation at specific waypoints
        if self.use_corridor_navigator and self.current_waypoint_index == 2:
            if not self.corridor_navigator.active:
                rospy.loginfo("Entering narrow corridor range. Switching to CorridorNavigator.")
                self.corridor_navigator.navigate_corridor()
            return
        else:
            if self.use_corridor_navigator and self.corridor_navigator.active:
                self.corridor_navigator.stop_navigation()

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

        rospy.loginfo(f"Navigating to waypoint: x={x}, y={y}, yaw={yaw}")
        self.client.send_goal(goal, done_cb=self.goal_done_callback)

    def goal_done_callback(self, status, result):
        """
        Callback for when the goal is completed or aborted by the action server.
        """
        if status == actionlib.GoalStatus.ABORTED:
            rospy.logerr("Navigation aborted by the action server. Shutting down...")
            rospy.signal_shutdown("Navigator node stopped due to aborted goal.")
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.logwarn("Navigation preempted. Shutting down...")
            rospy.signal_shutdown("Navigator node preempted.")
        else:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached!")
            self.current_waypoint_index += 1
            self.send_next_waypoint()

    def research_status_cb(self, msg):
        """
        Callback for research status updates from node_b.
        Handles stopping navigation if all IDs are found.
        """
        if msg.status:
            self.send_next_waypoint()
        else:
            rospy.loginfo("All IDs found!")
            state = self.client.get_state()
            if state == actionlib.GoalStatus.ACTIVE:
                self.client.cancel_goal()

"""
Class used to handle the navigation in a narrow corridor via ad hoc control law.
"""
# CORRIDOR NAVIGATOR CLASS
class CorridorNavigator:
    def __init__(self, waypoint_navigator):
        """
        Initialize the CorridorNavigator for handling navigation in narrow corridors.
        """
        # Velocity command publisher
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        
        # Laser scan subscriber
        rospy.Subscriber('/scan', LaserScan, self.laser_scan_cb)
        
        # Pose subscriber for position updates
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_cb)

        # Parameters for control law
        self.safe_distance = 0.1  
        self.min_side_distance = 0.4  
        self.max_speed = 0.5  
        self.max_turn_speed = 0.5  
        self.turning_factor = 0.02  

        self.active = False
        self.current_x_position = 0.0  
        self.waypoint_navigator = waypoint_navigator

    def pose_cb(self, msg):
        """
        Updates the robot's x position for corridor navigation.
        """
        self.current_x_position = msg.pose.pose.position.x

    def stop_navigation(self):
        """
        Stops corridor navigation and returns control to waypoint navigation.
        """
        rospy.loginfo("Deactivating CorridorNavigator. Returning to waypoint navigation.")
        self.active = False
        self.stop()
        self.waypoint_navigator.current_waypoint_index = 3
        self.waypoint_navigator.send_next_waypoint()

    def navigate_corridor(self):
        """
        Activates corridor navigation mode.
        """
        rospy.loginfo("CorridorNavigator activated. Navigating using laser scan data.")
        self.active = True

    def laser_scan_cb(self, data):
        """
        Processes laser scan data to navigate the narrow corridor.
        """
        if not self.active:
            return

        if self.current_x_position >= 5.5:
            rospy.loginfo("Exiting corridor navigation area.")
            self.stop_navigation()
            return

        ranges = data.ranges

        if not ranges:
            rospy.logwarn("No laser scan data received!")
            return

        front_range = sum(ranges[0:10] + ranges[350:359]) / len(ranges[0:10] + ranges[350:359])  
        left_range = sum(ranges[60:120]) / len(ranges[60:120])  
        right_range = sum(ranges[240:300]) / len(ranges[240:300])  

        cmd_vel = Twist()

        if front_range < self.safe_distance:
            rospy.logwarn(f"Obstacle detected in front! Stopping.")
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        else:
            cmd_vel.linear.x = self.max_speed

            balance_threshold = 0.4 
            distance_diff = abs(left_range - right_range)

            if distance_diff < balance_threshold:
                cmd_vel.angular.z = 0.0
            else:
                if left_range > right_range:
                    cmd_vel.angular.z = self.turning_factor * distance_diff
                else:
                    cmd_vel.angular.z = -self.turning_factor * distance_diff

        self.cmd_vel_pub.publish(cmd_vel)

    def stop(self):
        """
        Stops the robot.
        """
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

if __name__ == '__main__':
    try:
		# Initialize the ROS node
        rospy.init_node('navigator_node', anonymous=True)
        
        # Read the ROS parameter for CorridorNavigator
        use_corridor_navigator = rospy.get_param('~use_corridor_navigator', False)
        
        # Call the navigation function
        WaypointNavigator(use_corridor_navigator)
        
        rospy.loginfo("Navigator node ready!")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Error! Navigator node stopped!")




 
