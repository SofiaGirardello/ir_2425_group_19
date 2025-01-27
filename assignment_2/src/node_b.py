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

        # Subscribe to /placement_positions
        self.placement_sub = rospy.Subscriber('/placement_positions', PoseStamped, self.placement_callback)

        # Subscribe to the /tag_detections topic to get AprilTag detections
        self.detection_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback)

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

            if tag_id not in self.detected_ids:
                self.detected_ids.append(tag_id)
                # Create a PoseStamped object for the tag's pose
                pos_in = PoseStamped()
                pos_in.header.frame_id = source_frame
                pos_in.header.stamp = rospy.Time(0)
                pos_in.pose = detection.pose.pose.pose

                # Transform the pose to the target frame
                try:
                    pos_out = self.listener.transformPose(self.target_frame, pos_in)
                    rospy.loginfo("Suvessfully transformed")
                except tf.Exception as e:
                    rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")

                if tag_id != 10:
                    # Create collision object for current id
                    self.obj_collision.add_new_collision_object(tag_id, pos_out)
                    self.detected_poses.append(pos_out)
                else:
                    for ind in range(len(self.placement_positions)):
                        self.placement_positions[ind].header.frame_id = "tag_10"

        if len(self.detected_ids) >= 4:
            
            try:
                self.listener.waitForTransform("base_link", self.target_frame, rospy.Time(0), rospy.Duration(1.0))
                self.listener_tag10.waitForTransform("base_link", "tag_10", rospy.Time(0), rospy.Duration(1.0))
            except tf.Exception as e:
                rospy.logerr(f"Transform unavailable: {e}")
                return
            try:
                pick_poses = []
                place_poses = []
                for detection in self.detected_poses:
                    pick_pose = self.listener.transformPose("base_link", detection)
                    pick_poses.append(pick_pose)
                for detection in self.placement_positions:
                    place_pose = self.listener_tag10.transformPose("base_link", detection)
                    place_poses.append(place_pose)
            except tf.Exception as e:
                rospy.logerr(f"Failed to transform pose for tag ID {tag_id}: {e}")

            rospy.loginfo(f"Detected ids: {self.detected_ids}")

            if 10 in self.detected_ids:

                ind = self.detected_ids.index(10)
                self.detected_ids.pop(ind)
                self.detected_poses.pop(ind)

            self.send_pick_place_goal(self.detected_ids, pick_poses, place_poses)
            # self.detection_sub.unregister()

    def send_pick_place_goal(self, detected_ids, detected_poses, place_poses):
        """
        Sends a pick-and-place goal to the action server.
        """

        min_ind = min(len(detected_poses), len(place_poses))
        
        for i in range(1, min_ind+1):
            
            goal = PickPlaceGoal()
            goal.object_pose = detected_poses[i]
            goal.place_pose = place_poses[i]
            goal.id = detected_ids[i]

            rospy.loginfo(f"Sending pick-and-place goal for tag ID {goal.id}")
            self.client.send_goal(goal, feedback_cb=self.feedback_callback)
            self.client.wait_for_result()

            result = self.client.get_result()

            rospy.loginfo(f"Received result: {result}")
            if result.success:
                rospy.loginfo(f"Pick-and-place successful for tag ID {goal.id}")
            else:
                rospy.logwarn(f"Pick-and-place failed for tag ID {goal.id}")


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


