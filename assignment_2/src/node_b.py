#!/usr/bin/env python

"""
@file   : node_b.py
@brief  : Object detection using AprilTags and pose transformation to the target frame.

@details: This node detects AprilTags, transforms their poses to the robot's base_link frame, 
          and sends a pick-and-place goal to the action server.

@date   : 2024-1-20
@authors : Caduceo Andrea, Girardello Sofia, Gizzarone Manuel 
"""

import rospy
import actionlib
from apriltag_ros.msg import AprilTagDetectionArray
from assignment_2.msg import PickPlaceAction, PickPlaceGoal
from geometry_msgs.msg import PoseStamped
import tf

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('node_b_object_detection')
        self.listener = tf.TransformListener()

        # Define the target frame to which we want to transform poses
        self.target_frame = 'base_link'

        # Lists to track detected IDs and poses
        self.detected_ids = []
        self.detected_poses = []

		# Placement positions
        self.placement_positions = []
        self.current_position_index = 0

        # Subscribe to the /tag_detections topic to get AprilTag detections
        self.detection_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback)

		# Subscribe to /placement_positions
		self.placement_sub = rospy.Subscriber('/placement_positions', Point, self.placement_callback)

        # Action client for pick-and-place operation
        self.client = actionlib.SimpleActionClient('pick_place', PickPlaceAction)
        rospy.loginfo("Waiting for pick_place action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to pick_place action server.")

	def placement_callback(self, msg):
        """
        Callback to receive placement positions from the PlacementLineNode.
        """
        self.placement_positions.append(msg)
        rospy.loginfo(f"Received placement position: {msg}")

    def detection_callback(self, msg):
        """
        Callback function for AprilTag detections. It processes detected tags and transforms their poses to the target frame.
        """
        source_frame = msg.header.frame_id

        # Wait for the transform to be available
        try:
            self.listener.waitForTransform(self.target_frame, source_frame, rospy.Time.now(), rospy.Duration(1.0))
        except tf.Exception as e:
            rospy.logerr(f"Transform unavailable: {e}")
			return

        # Process each detected tag
        for detection in msg.detections:
            tag_id = detection.id[0]

            # Check if the tag has already been processed
            if tag_id in self.detected_ids:
                continue

            # Create a PoseStamped object for the tag's pose
            pos_in = PoseStamped()
            pos_in.header.frame_id = source_frame
            pos_in.header.stamp = rospy.Time.now()
            pos_in.pose = detection.pose.pose.pose

            # Transform the pose to the target frame
            try:
                pos_out = self.listener.transformPose(self.target_frame, pos_in)

                # Save the ID and pose
                self.detected_ids.append(tag_id)
                self.detected_poses.append(pos_out)

				# Assign the next available placement position
                if self.current_position_index < len(self.placement_positions):
                    placement_point = self.placement_positions[self.current_position_index]
                    place_pose = PoseStamped()
                    place_pose.header.frame_id = self.target_frame
                    place_pose.pose.position = placement_point
                    place_pose.pose.orientation.w = 1.0

                	# Send a pick-and-place goal to the 	action server
                	self.send_pick_place_goal(tag_id, pos_out)

					# Update the index for the next object
                    self.current_position_index += 1

				else:
                    rospy.logwarn("No more placement positions available.")

            except tf.Exception as e:
                	rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")

    def send_pick_place_goal(self, tag_id, object_pose):
        """
        Sends a pick-and-place goal to the action server.
        """
        goal = PickPlaceGoal()
        goal.object_pose = object_pose
        goal.place_pose = goal.place_pose = place_pose

        rospy.loginfo(f"Sending pick-and-place goal for tag ID {tag_id}")
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()

        result = self.client.get_result()
        if result.success:
            rospy.loginfo(f"Pick-and-place successful for tag ID {tag_id}: {result.message}")
        else:
            rospy.logwarn(f"Pick-and-place failed for tag ID {tag_id}: {result.message}")


    def feedback_callback(self, feedback):
        """
        Handles feedback from the pick-and-place action server.
        """
        rospy.loginfo(f"Action feedback: {feedback.current_step}")

# Main function to run the node
if __name__ == '__main__':
    try:
        ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass