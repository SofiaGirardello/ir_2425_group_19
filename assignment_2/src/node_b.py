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
from geometry_msgs.msg import PoseStamped, Point
import tf
from assignment_2.objectCollision import AddCollisionObject


class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('node_b_object_detection')
        self.listener = tf.TransformListener()
        self.listener_tag10 = tf.TransformListener()

        # Define the target frame to which we want to transform poses for defining collision objects
        self.target_frame = 'map'

        # Lists to track detected IDs and poses
        self.detected_ids = []
        self.detected_poses = []

        # Placement positions
        self.placement_positions = []
        self.current_position_index = 0

        self.obj_collision = AddCollisionObject()

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

    def detection_callback(self, msg):
        """
        Callback function for AprilTag detections. It processes detected tags and transforms their poses to the target frame.
        """
        source_frame = msg.header.frame_id

        # Wait for the transform to be available
        try:
            self.listener.waitForTransform(self.target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        except tf.Exception as e:
            rospy.logerr(f"Transform unavailable: {e}")
            return

        for detection in msg.detections:
            tag_id = detection.id[0]
            if tag_id not in self.detected_ids and tag_id != 10:
                self.detected_ids.append(tag_id)
                # Create a PoseStamped object for the tag's pose
                pos_in = PoseStamped()
                pos_in.header.frame_id = source_frame
                pos_in.header.stamp = rospy.Time(0)
                pos_in.pose = detection.pose.pose.pose

                # Transform the pose to the target frame
                try:
                    pos_out = self.listener.transformPose(self.target_frame, pos_in)
                except tf.Exception as e:
                    rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")

                # Create collision object for current id
                self.obj_collision.add_new_collision_object(tag_id, pos_out)

                self.detected_poses.append(pos_out)
        
        if len(self.detected_ids) >= 8:
            rospy.loginfo("Mannaggia il cristo")
            
            try:
                self.listener.waitForTransform("base_link", source_frame, rospy.Time(0), rospy.Duration(1.0))
                self.listener_tag10.waitForTransform("base_link", "tag10", rospy.Time(0), rospy.Duration(1.0))
            except tf.Exception as e:
                rospy.logerr(f"Transform unavailable: {e}")
                return
            try:
                pick_poses = self.listener.transformPose("base_link", self.detected_poses)
                place_poses = self.listener_tag10.transformPose("base_link", self.placement_positions)
            except tf.Exception as e:
                rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")

            self.send_pick_place_goal(self.detected_ids, pick_poses, place_poses)
            self.detection_sub.unregister()

    def send_pick_place_goal(self, detected_ids, detected_poses, place_poses):
        """
        Sends a pick-and-place goal to the action server.
        """

        cont = 0
        for id, pose_pick, pose_place in zip(detected_ids, detected_poses, place_poses):

            if cont >= 4:
                break
            goal = PickPlaceGoal()
            goal.object_pose = pose_pick
            goal.place_pose = pose_place
            goal.id = id

            rospy.loginfo(f"Sending pick-and-place goal for tag ID {id}")
            self.client.send_goal(goal, feedback_cb=self.feedback_callback)
            self.client.wait_for_result()

            result = self.client.get_result()

            rospy.loginfo(f"Received result: {result}")
            if result.success:
                rospy.loginfo(f"Pick-and-place successful for tag ID {id}")
            else:
                rospy.logwarn(f"Pick-and-place failed for tag ID {id}")

            cont += 1

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


