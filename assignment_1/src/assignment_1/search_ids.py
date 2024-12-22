#!/usr/bin/env python

"""
@file   : search_ids.py
@brief  : A ROS action server that searches for AprilTags and reports their poses with respect to the map frame.
          The server listens for target IDs, detects the AprilTags, transforms their poses to the map frame, 
          and informs the navigation system once the research is complete.

@details:
    - This script provides an action server that listens for a list of target IDs from an action client. The action server
      performs a search for AprilTags and transforms their detected poses into the map frame. The server communicates
      with the navigator node about the start and end of the research, and when all target IDs have been found, it
      notifies the client with the results.
    - The ROS node subscribes to the `/tag_detections` topic to get the AprilTag detection data and uses the `tf` listener
      to transform poses from the camera frame to the map frame.  
    - Upon successful detection of all target IDs, the research status is updated, and the action is completed with a success message.

@date   : 2024-12-23
@authors : Caduceo Andrea, Girardello Sofia, Gizzarone Manuel
"""

import rospy
import actionlib
import tf
from assignment_1.msg import FindAprilTagsAction, FindAprilTagsFeedback, FindAprilTagsResult, research_status
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped 

"""
Classsed to handle the recognition of the apriltags and the detection of their poses with respect to the map frame. When the target IDs are received
from the action client, the navigator node is informed that it can begin navigation. When the research is finished, the navigator node is informed that it can stop
the navigation.
"""

# FIND APRIL TAGS CLASS
class FindAprilTags:
    def __init__(self, name):
        """
        Initializes the FindAprilTags action server and sets up required publishers and subscribers.
        """
        # Set up the action server
        self.server_name = name
        self.server = actionlib.SimpleActionServer(
            self.server_name,
            FindAprilTagsAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        # Start the action server
        self.server.start()

        # Class variables
        self.target_ids = []  # List of target IDs to find
        self.detected_ids = []  # List of IDs already found while moving
        self.detected_poses = []  # List of detected poses

        # Publisher to communicate with the navigator node about research status
        self.pub = rospy.Publisher('/research_status', research_status, queue_size=10)
        self.status_msg = research_status()

        # Subscriber for AprilTags detection during robot's motion
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback)

        # Transform listener for converting poses between camera and map frames
        self.listener = tf.TransformListener()
        self.target_frame = "map"
        self.source_frame = ""

    def execute_cb(self, goal):
        """
        Executes the action when a goal (target IDs) is received from the action client.
        """
        # Save the received target IDs
        self.target_ids = goal.target_ids.ids

        # Inform the navigator node that it can start the navigation
        self.status_msg.status = True
        self.pub.publish(self.status_msg)

        rospy.loginfo("Search for Apriltags started!")

        # Set a flag used to check during the execution if all the IDs have been found
        flag = False

        try:
            timeout_duration = rospy.Duration(150)  # Timeout of 2 minutes and 30 seconds to complete the search
            start_time = rospy.Time.now()
            rate = rospy.Rate(1)

            while not rospy.is_shutdown():
                # Check if the client requested to cancel the action
                if self.server.is_preempt_requested():
                    rospy.loginfo("Action preempted by the client.")
                    self.server.set_preempted()
                    self.status_msg.status = False
                    self.pub.publish(self.status_msg)
                    break

                # Check when all the IDs have been found
                if len(self.detected_ids) == len(self.target_ids):
                    rospy.loginfo("All AprilTags have been found! Stopping the navigation...")
                    self.status_msg.status = False
                    self.pub.publish(self.status_msg)
                    flag = True
                    break

                # Check if the timeout has been reached
                if rospy.Time.now() - start_time > timeout_duration:
                    self.server.set_aborted()
                    self.status_msg.status = False
                    self.pub.publish(self.status_msg)
                    break

                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS shutdown detected! Exiting...")

        # Handle the result of the action
        if flag:
            result = FindAprilTagsResult()

            # Set the result of the action to success and compose the result message
            result.success = True
            result.found_ids = self.detected_ids
            result.found_poses = self.detected_poses

            # Send the result to the action client
            self.server.set_succeeded(result)

            rospy.signal_shutdown("Action completed successfully! Stopping the server node...")
        else:
            rospy.signal_shutdown("Action failed! Stopping the server node...")

    def detection_callback(self, msg):
        """
        Callback function for AprilTag detections. It processes detected tags and transforms their poses to the map frame.
        Every time a new AprilTag is detected by the camera, this function checks whether the ID is present in the list of target IDs and only if so,
        saves the relevant data and informs the action client by sending feedback.
        """
        # Obtain the source frame
        self.source_frame = msg.header.frame_id

        # Wait until the transform is available
        while not self.listener.canTransform(self.target_frame, self.source_frame, rospy.Time(0)):
            rospy.sleep(0.5)

        for detection in msg.detections:
            tag_id = detection.id[0]
            # Check if the ID is in the target list and if it has not already been found before
            if tag_id in self.target_ids and tag_id not in self.detected_ids:

                pos_in = PoseStamped()
                pos_out = PoseStamped()

                # Obtain the pose in the camera frame
                pos_in.header.frame_id = detection.pose.header.frame_id
                pos_in.pose.position = detection.pose.pose.pose.position
                pos_in.pose.orientation = detection.pose.pose.pose.orientation

                # Transform the pose in the map frame
                try:
                    pos_out = self.listener.transformPose(self.target_frame, pos_in)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr(f"Error during transform: {e}")

                # Publish feedback
                feedback = FindAprilTagsFeedback()
                feedback.detected_id = tag_id
                feedback.detected_pose = pos_out
                self.server.publish_feedback(feedback)

                # Save the ID and the pose in the respective class variables
                self.detected_ids.append(tag_id)
                self.detected_poses.append(pos_out)

