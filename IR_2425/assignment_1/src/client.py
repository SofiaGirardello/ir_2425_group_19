#!/usr/bin/env python

"""
@file   : client.py
@brief  : A ROS client node for controlling a robot to search for AprilTags using a predefined set of waypoints.
          It subscribes to the navigation status to track the robot's progress and interacts with the 
          'find_apriltags' action server to search for specific AprilTags.

@details:
    - This script performs the following tasks:
      1. Requests a list of target AprilTag IDs from the service `/apriltag_ids_srv`.
      2. Subscribes to the `/navigation_status` topic to receive updates on the robot's progress to waypoints.
      3. Sends a goal to the action server `find_apriltags` to find the specified AprilTags.
      4. Processes feedback for each detected AprilTag, including logging the detected ID, position, and orientation (yaw).
      5. Logs the completion of the search once all the tags are found or the action completes.
      6. Logs information about the robot's progress as it navigates between waypoints.

@date   : 2024-12-23
@authors : Caduceo Andrea, Girardello Sofia, Gizzarone Manuel 
"""

import rospy
from tiago_iaslab_simulation.srv import Objs, ObjsRequest
from assignment_1.msg import FindAprilTagsAction, FindAprilTagsGoal, FindAprilTagsFeedback, target_ids, info_navigation
import actionlib
from tf.transformations import euler_from_quaternion


# GET TARGET IDs FUNCTION
def get_target_ids():
    """
    Request and retrieve the list of target AprilTag IDs from the service /apriltag_ids_srv.
    Returns a list of target IDs.
    """
    # Waits for the service to be available 
    rospy.wait_for_service('/apriltag_ids_srv')

    try:
        apriltag_ids_service = rospy.ServiceProxy('/apriltag_ids_srv', Objs)
        request = ObjsRequest()
        request.ready = True
        response = apriltag_ids_service(request)
        return response.ids
    except rospy.ServiceException as e:
        rospy.logerr(f"Error while calling the service: {e}")
        return []


# FEEDBACK CALLBACK FUNCTION
def feedback_callback(feedback):
    """
    Callback function that gets triggered each time a new AprilTag is detected.
    Logs the detected tag's ID, position, and orientation (yaw angle).
    """
    rospy.loginfo("###### NEW FEEDBACK ######")
    
    # Extract id, position and orientation from the feedback message
    detected_id = feedback.detected_id
    detected_position = feedback.detected_pose.pose.position
    detected_orientation = feedback.detected_pose.pose.orientation

    # Convert from quaternion to Euler angles
    quaternion = (detected_orientation.x, detected_orientation.y, detected_orientation.z, detected_orientation.w)
    _, _, yaw = euler_from_quaternion(quaternion)

    # Print the feedback message
    rospy.loginfo("New AprilTag detected!")
    rospy.loginfo(f"AprilTag ID: {detected_id}")
    rospy.loginfo(f"Detected position: x = {detected_position.x:.2f}, y = {detected_position.y:.2f}, yaw angle = {yaw:.2f} rad")
    rospy.loginfo("###### FEEDBACK ENDS ######")


# DONE CALLBACK FUNCTION
def done_callback(state, result):
    """
    Callback function that is called when the FindAprilTagsAction is completed.
    Logs the final result of the action, including the position of each found AprilTag.
    """
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot has completed the search!")
        
        # Iterate over each found AprilTag pose 
        for i, pose in enumerate(result.found_poses):
            position = pose.pose.position
            orientation = pose.pose.orientation

            # Convert from quaternion to Euler angles
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            _, _, yaw = euler_from_quaternion(quaternion)

            rospy.loginfo(f"AprilTag ID {result.found_ids[i]} position: x = {position.x:.2f}, y = {position.y:.2f}, yaw angle = {yaw:.2f} rad")

    elif state == actionlib.GoalStatus.PREEMPTED:
        rospy.logwarn(f"The action was preempted by the client!")

    else:
        rospy.logerr(f"The action was aborted by the server! Time limit probably exceeded!")


# NAVIGATION CALLBACK FUNCTION
def navigation_callback(info_msg):
    """
    Callback function for /navigation_status.
    Logs the robot's progress through waypoints and when it reaches each waypoint.
    """
    # Only log if the robot is moving to a new waypoint
    if info_msg.x != 0.0 or info_msg.y != 0.0:
        rospy.loginfo(f"###### NEW WAYPOINT ######")
        rospy.loginfo(f"Moving to waypoint: x = {info_msg.x}, y = {info_msg.y}")

    # If we reach the last waypoint, log the completion of the series of waypoints
    if info_msg.x == 10.0 and info_msg.y == 0.6:
        rospy.loginfo("###### LAST WAYPOINT REACHED ######")


if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('node_a_client', anonymous=True)
        
        # Subscribe to the navigation status to track progress to waypoints
        rospy.Subscriber('/navigation_status', info_navigation, navigation_callback)
        
        # Obtain the IDs to find
        target = get_target_ids()
        
        if not target:
            rospy.logwarn("No target IDs received! Exiting.")
            exit(1)
            
        rospy.loginfo(f"Target IDs received from ids_generator_node: {target}")

        # Create an action client
        client = actionlib.SimpleActionClient('find_apriltags', FindAprilTagsAction)
        rospy.loginfo("Waiting for action server ...")
        client.wait_for_server()
        rospy.loginfo("Action server started!")

        # Set up the goal with the received target IDs
        goal = FindAprilTagsGoal()
        goal.target_ids = target_ids()  
        goal.target_ids.ids = list(target)

        # Send the goal to the action server
        client.send_goal(goal, feedback_cb=feedback_callback, done_cb=done_callback)

        # Wait for the result of the action (this blocks further execution until the action is completed)
        client.wait_for_result()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node_a_client stopped execution!")
