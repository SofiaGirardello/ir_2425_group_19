#!/usr/bin/env python

"""
@file   : object_detection_node.py
@brief  : Object detection using AprilTags and pose transformation to the target frame.

@details: This node detects AprilTags, transforms their poses to the robot's base_link frame, 
          and sends pick-and-place goals to action servers based on detected objects' colors and positions.

@date   : 2025-06-10
@authors: Caduceo Andrea, Girardello Sofia, Gizzarone Manuel 
"""

import rospy
import actionlib
from apriltag_ros.msg import AprilTagDetectionArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_commander import MoveGroupCommander
from assignment_2.msg import PickObjectAction, PickObjectGoal, PlaceObjectAction, PlaceObjectGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PoseArray
from actionlib_msgs.msg import GoalStatus
import tf
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import numpy as np
from assignment_2.objectCollision import AddCollisionObject

class ObjectDetectionNode:
    def __init__(self):
        """
        Initialize the ObjectDetectionNode with all necessary components:
        - ROS node initialization
        - TF listeners
        - Action clients for navigation, picking and placing
        - Subscribers for tag detections and placement positions
        - Collision object manager
        """
        rospy.init_node('object_detection_node')
        
        # Frame references
        self.robot_frame = "base_link"
        
        # TF listeners for coordinate transformations
        self.tf_camera_robot = tf.TransformListener()
        self.tf_tag10_robot = tf.TransformListener()
        
        # Data storage
        self.detected_ids = []        # List of detected tag IDs
        self.detected_poses = []      # List of detected poses in robot frame
        self.picked_ids = []          # List of already picked object IDs
        self.detected_colors = []     # List of detected object colors
        self.placement_positions = [] # Placement positions from node_a (in tag_10 frame)
        
        # Collision object manager
        self.collision_obj_manager = AddCollisionObject()
        
        # Camera and image processing
        self.bridge = CvBridge()
        self.latest_image = None
        self.K = None  # Camera intrinsic matrix
        
        # Initialize subscribers
        self.setup_subscribers()
        
        # Initialize action clients
        self.setup_action_clients()
        
        # Get user input for operation mode
        self.get_operation_mode()
        
        # Robot initialization and positioning
        self.initialize_robot()
        
        # Navigation and operation parameters
        self.setup_navigation_parameters()
        
        # Main execution loop
        self.execute_main_loop()

    def setup_subscribers(self):
        """
        Initialize all ROS subscribers.
        """
        
        # Placement positions subscriber
        self.placement_sub = rospy.Subscriber('/placement_positions', PoseArray, self.placement_callback)
        
        # Camera info subscriber (for intrinsic parameters)
        rospy.Subscriber("/xtion/rgb/camera_info", CameraInfo, self.camera_info_callback)
        
        # Image subscriber for color detection
        self.image_sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback)

    def setup_action_clients(self):
        """
        Initialize all action clients and wait for servers.
        """
        
        # Pick action client
        self.pick_action_client = actionlib.SimpleActionClient('pick_object', PickObjectAction)
        rospy.loginfo("Waiting for pick object action server...")
        self.pick_action_client.wait_for_server()
        rospy.loginfo("Connected to pick object action server.")
        
        # Place action client
        self.place_action_client = actionlib.SimpleActionClient('place_object', PlaceObjectAction)
        rospy.loginfo("Waiting for place object action server...")
        self.place_action_client.wait_for_server()
        rospy.loginfo("Connected to place object action server.")
        
        # Move base action client for navigation
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server!")

    def get_operation_mode(self):
        """
        Get user input to determine operation mode.
        """
        
        self.mode_selection = int(input("Enter the working mode (0:number of objects, 1:BLUE, 2:RED, 3:GREEN) "))
        rospy.loginfo(f"Selected mode: {self.mode_selection}")
        
        # Determine number of objects to pick based on mode
        if self.mode_selection == 0:
            self.num_objects = int(input("Enter the number of objects you want to be picked: "))
        else:
            self.num_objects = 3  # Default for color modes

    def initialize_robot(self):
        """
        Initialize robot position and arm configuration.
        """
        
        # Initial navigation to starting position
        self.navigate_to_point([8.8, 0.0, -1.57])
        
        # Initialize arm commander and move to home position
        self.arm_group = MoveGroupCommander('arm_torso')
        home_joints_position = [0.34, 0.07, 0.34, -3.13, 1.31, 1.58, -0.0, 0.0]
        self.arm_group.set_joint_value_target(home_joints_position)
        self.arm_group.go(wait=True)

    def setup_navigation_parameters(self):
        """
        Set up navigation parameters and waypoints.
        """
        
        # Docking positions for pick and place tables
        self.pickup_table = [8.8, -3.0, 3.14]
        self.place_table = [8.7, -2.2, 3.14]
        
        # Navigation waypoints (different sets based on operation mode)
        if self.mode_selection == 0:
            self.list_pickup_positions = [
                [8.8, -2.7, 3.14],
                [8.8, -3.0, 3.14],
                [8.8, -3.0, 3.14],
                [8.8, -3.2, 3.14], 
                [8.0, -3.9, 1.57],
                [7.7, -3.9, 1.57],
                [7.6, -3.9, 1.57],
                [6.9, -2.6, 6.28],
                [7.1, -2.5, 6.28]
            ]
        else:
            self.list_pickup_positions = [
                [8.8, -2.8, 3.14],
                [8.0, -3.9, 1.57],
                [6.9, -3.1, 6.28]
            ]
        
        # Operation counters and flags
        self.place_idx = 0      # Index for placement positions
        self.pick_idx = 0       # Index for picking operations
        self.navigation_idx = 0 # Index for navigation waypoints
        self.done_counter = 0   # Counter for successfully placed objects
        
        # Operation state flags
        self.done_pick = True          # Flag indicating successful pick operation
        self.second_trial_pick = False # Flag for retry after failed pick

    def execute_main_loop(self):
        """
        Main execution loop for pick and place operations.
        """
        
        while self.done_counter < self.num_objects:
            self.execute_routine()
        rospy.loginfo(f"Pick and place task executed successfully!")
        rospy.signal_shutdown("Task done")
        sys.exit(0)

    def execute_routine(self):
        """
        Execute one complete pick-and-place routine:
        1. Navigate to appropriate position
        2. Detect objects and their colors
        3. Pick object if it matches selection criteria
        4. Place object at designated position
        """
        
        # Navigation phase
        self.execute_navigation_phase()
        
        # Object detection phase
        self.detect_objects()
        
        # Process detected objects
        self.process_detected_objects()

    def execute_navigation_phase(self):
        """
        Handle navigation to appropriate positions based on current state.
        """

        if (len(self.list_pickup_positions) <= self.navigation_idx):
            rospy.logerr(f"Navigated around the whole table!")
            rospy.signal_shutdown("Navigation error")
            sys.exit(1)
            
        if self.done_pick:
            # Standard navigation sequence
            if self.mode_selection == 0:
                if self.navigation_idx > 3:
                    self.navigate_to_point([8.9, -2.7, -1.57])
                    self.navigate_to_point([8.8, -3.7, -1.57])
                    if self.navigation_idx > 6:
                        self.navigate_to_point([7.1, -3.9, 3.14])
            else:
                if self.navigation_idx > 0:
                    self.navigate_to_point([8.9, -2.7, -1.57])
                    self.navigate_to_point([8.8, -3.7, -1.57])
                    if self.navigation_idx > 1:
                        self.navigate_to_point([7.1, -3.9, 3.14])
        elif not self.second_trial_pick:
            # Special navigation for retry cases
            if self.mode_selection == 0:
                if self.navigation_idx == 7:
                    self.navigate_to_point([7.1, -3.9, 3.14])
                elif self.navigation_idx == 4:
                    self.navigate_to_point([8.8, -3.7, -1.57])
            else:
                if self.navigation_idx == 1:
                    self.navigate_to_point([8.8, -3.7, -1.57])
                elif self.navigation_idx == 2:
                    self.navigate_to_point([7.1, -3.9, 3.14])

        # Navigate to current pickup position
        if not self.second_trial_pick:
            self.navigate_to_point(self.list_pickup_positions[self.navigation_idx])

    def detect_objects(self):
        """
        Detect objects by subscribing to AprilTag detections and moving head.
        """
        
        # Reset detection lists
        self.detected_ids = []
        self.detected_poses = []
        self.detected_colors = []
        
        # Subscribe to tag detections and move head to scan area
        self.detection_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback)
        rospy.sleep(1)
        self.move_head()
        rospy.sleep(1)
        
        rospy.loginfo(f"Detected AprilTags: {len(self.detected_ids)}")

    def process_detected_objects(self):
        """
        Process detected objects:
        - Check if they match selection criteria
        - Attempt pick operation
        - If successful, place object
        """
        
        min_ind = min(len(self.detected_poses), len(self.placement_positions))
        
        # Adjust placement table position if needed
        initial_position = self.placement_positions[0]
        if initial_position.pose.position.y > 0.15 and self.place_idx == 0:
            self.place_table[0] -= 0.15

        for idx in range(min_ind):
            tag_id = self.detected_ids[idx]
            color = self.detected_colors[idx]
            
            # Skip already picked objects
            if tag_id in self.picked_ids:
                continue 
            
            # Get object and placement poses
            pick_pose = self.detected_poses[idx]
            place_pose = self.placement_positions[self.place_idx]
            
            # Check if object matches selection criteria
            if self.object_matches_selection(color):
                # Attempt pick operation
                self.done_pick = self.send_pick_goal(tag_id, pick_pose)
                
                if self.done_pick:
                    self.handle_successful_pick(tag_id, place_pose)
                    return
                else:
                    self.handle_failed_pick()
            else:
                rospy.loginfo("Object color doesn't match selection criteria")
                self.done_pick = False

    def object_matches_selection(self, color):
        """
        Check if object color matches the selected mode.
        """
        
        if (self.mode_selection == 1 and color != 'blue') or \
           (self.mode_selection == 2 and color != 'red') or \
           (self.mode_selection == 3 and color != 'green'):
            return False
        return True

    def handle_successful_pick(self, tag_id, place_pose):
        """
        Handle operations after successful pick.
        """
        
        self.second_trial_pick = False 
        self.navigation_idx += 1
        self.pick_idx += 1
        self.picked_ids.append(tag_id)

        rospy.loginfo(f"Placement table position: {self.place_table}")
        
        # Navigate to intermediate positions before placement
        self.navigate_to_intermediate_positions()
        
        # Navigate to placement table
        self.navigate_to_point(self.place_table)
        rospy.sleep(1)
        
        # Adjust head position for placement
        self.move_head(
            pan_range=(0.4, 0.4),  
            tilt_down=-1,        
            tilt_up=0.0,           
            step=1.0,             
            duration=2.0           
        )
        rospy.sleep(1)
        
        # Attempt place operation
        done_place = self.send_place_goal(tag_id, place_pose)

        if done_place:
            self.handle_successful_placement()
        else: 
            rospy.logerr(f"Placing positions are too far! Try again with different values of the straight line!")
            rospy.signal_shutdown("Position error")
            sys.exit(1)

    def navigate_to_intermediate_positions(self):
        """
        Navigate through intermediate positions based on operation mode.
        """
        
        if self.mode_selection == 0:
            if self.navigation_idx > 4:
                if self.navigation_idx > 7:
                    self.navigate_to_point([7.0, -3.8, -1.57])
                self.navigate_to_point([8.8, -3.8, 0])
        else:
            if self.navigation_idx > 1:
                if self.navigation_idx > 2:
                    self.navigate_to_point([7.0, -3.8, -1.57])
                self.navigate_to_point([8.8, -3.8, 0])

    def handle_successful_placement(self):
        """
        Update state after successful placement.
        """
        
        self.done_counter += 1
        self.place_idx += 1
        if self.place_table[0] > 8.5:
            self.place_table[0] -= 0.05
        self.place_table[1] += 0.07

    def handle_failed_pick(self):
        """
        Handle failed pick operation with retry logic.
        """
        
        rospy.loginfo("Pick operation failed")
        
        if self.second_trial_pick: 
            self.second_trial_pick = False 
            self.navigation_idx += 1
            self.done_pick = False
        else: 
            self.second_trial_pick = True

    def navigate_to_point(self, position):
        """
        Navigate the robot to a specific position in the map.
        
        Args:
            position: List containing [x, y, yaw] coordinates in the map frame
        """
        
        x, y, yaw = position

        # Create navigation goal
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

        # Send goal and wait for result
        rospy.loginfo(f"Navigating to point: x={x}, y={y}, yaw={yaw}")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        # Check navigation result
        if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully reached the waypoint!")
        else:
            rospy.logerr("Navigation to waypoint failed.")

    def move_head(self, pan_range=(-0.5, 0.5), tilt_down=-1, tilt_up=0.0, step=0.5, duration=2.0):
        """
        Perform head movement routine to scan the environment.
        
        Args:
            pan_range: Tuple of min and max pan angles (radians)
            tilt_down: Tilt angle for looking down (negative)
            tilt_up: Tilt angle for returning up (usually 0)
            step: Step size (radians) for sweeping pan
            duration: Duration (sec) for each motion segment
        """
        
        pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(1)

        def send_command(pan, tilt, duration_sec):
            """
            Helper function to send head movement commands.
            """
            
            msg = JointTrajectory()
            msg.header.stamp = rospy.Time.now()
            msg.joint_names = ['head_1_joint', 'head_2_joint']
            point = JointTrajectoryPoint()
            point.positions = [pan, tilt]
            point.time_from_start = rospy.Duration(duration_sec)
            msg.points.append(point)
            pub.publish(msg)
            rospy.sleep(duration_sec + 0.2)  # Small buffer after each movement

        # Tilt head down
        send_command(0.0, tilt_down, duration)

        # Sweep from left to right
        pan_min, pan_max = pan_range
        current_pan = pan_min
        while current_pan <= pan_max:
            send_command(current_pan, tilt_down, duration)
            current_pan += step

        # Tilt head back up
        send_command(0.0, tilt_up, duration)

    def detection_callback(self, msg):
        """
        Callback for AprilTag detections. Processes detected tags, transforms their poses,
        and identifies their colors.
        
        Args:
            msg: AprilTagDetectionArray message containing detected tags
        """
        
        source_frame = msg.header.frame_id

        # Wait for transform to be available
        try:
            self.tf_camera_robot.waitForTransform(self.robot_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        except tf.Exception as e:
            rospy.logerr(f"Transform from {source_frame} to {self.robot_frame} unavailable. Error: {e}")
            return

        # Process each detection
        for detection in msg.detections:
            tag_id = detection.id[0]
            
            # Skip tag 10 (reference tag) and already detected tags
            if tag_id == 10 or tag_id in self.detected_ids:
                continue
                
            self.detected_ids.append(tag_id)

            # Transform pose from camera to robot frame
            pos_in = PoseStamped()
            pos_out = PoseStamped()

            pos_in.header.frame_id = source_frame
            pos_in.header.stamp = rospy.Time(0)
            pos_in.pose.position = detection.pose.pose.pose.position
            pos_in.pose.orientation = detection.pose.pose.pose.orientation

            # Get pixel coordinates for color detection
            pixel_coords = self.get_pixel_coords(pos_in)
            color = self.detect_color(pixel_coords)
            rospy.loginfo(f"Detected color: {color}")
            self.detected_colors.append(color)

            try:
                pos_out = self.tf_camera_robot.transformPose(self.robot_frame, pos_in)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(f"Error during transform from {source_frame} to {self.robot_frame}! Error: {e}")
                return

            # Add collision object and store pose
            self.collision_obj_manager.add_new_collision_object(tag_id, pos_out, self.robot_frame)
            self.detected_poses.append(pos_out)

    def placement_callback(self, msg):
        """
        Callback for placement positions from node_a.
        
        Args:
            msg: PoseArray message containing placement positions in tag_10 frame
        """
        
        self.placement_positions = [] 
        for pose in msg.poses:
            ps = PoseStamped()
            ps.header = msg.header  
            ps.pose = pose
            self.placement_positions.append(ps)
            rospy.loginfo(f"Received placement pose in Tag_10 frame: {pose.position}")

    def send_pick_goal(self, tag_id, pick_pose):
        """
        Send pick goal to action server.
        
        Args:
            tag_id: ID of the tag/object to pick
            pick_pose: Pose of the object in robot frame
            
        Returns:
            bool: True if pick was successful, False otherwise
        """
        
        goal = PickObjectGoal()
        goal.object_pose = pick_pose
        goal.id = tag_id

        self.pick_action_client.send_goal(goal, feedback_cb=self.feedback_callback_pick_action)
        rospy.loginfo(f"Sending pick action for Tag {tag_id} in pose: X={pick_pose.pose.position.x}, Y={pick_pose.pose.position.y}, Z={pick_pose.pose.position.z}")
        self.pick_action_client.wait_for_result()

        result = self.pick_action_client.get_result()
        return result.success if result else False

    def send_place_goal(self, tag_id, place_pose):
        """
        Send place goal to action server.
        
        Args:
            tag_id: ID of the tag/object to place
            place_pose: Pose for placement in tag_10 frame
            
        Returns:
            bool: True if place was successful, False otherwise
        """
        
        goal = PlaceObjectGoal()
        goal.place_pose = place_pose
        goal.id = tag_id

        self.place_action_client.send_goal(goal, feedback_cb=self.feedback_callback_place_action)
        rospy.loginfo(f"Sending place action for Tag {tag_id} in pose: X={place_pose.pose.position.x}, Y={place_pose.pose.position.y}, Z={place_pose.pose.position.z}")
        self.place_action_client.wait_for_result()

        result = self.place_action_client.get_result()
        return result.success if result else False

    def feedback_callback_pick_action(self, feedback):
        """
        Feedback callback for pick action.
        """
        
        pass

    def feedback_callback_place_action(self, feedback):
        """
        Feedback callback for place action.
        """
        
        pass

    def camera_info_callback(self, msg):
        """
        Callback for camera info to get intrinsic parameters.
        """
        
        self.K = np.array(msg.K).reshape((3,3))

    def image_callback(self, msg):
        """
        Callback for camera images for color detection.
        """
        
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def get_pixel_coords(self, pose_stamped):
        """
        Convert 3D pose to 2D pixel coordinates using camera intrinsics.
        
        Args:
            pose_stamped: PoseStamped message in camera frame
            
        Returns:
            tuple: (u, v) pixel coordinates or None if invalid
        """
        
        if self.K is None:
            rospy.logwarn("Camera intrinsics not available")
            return None

        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        z = pose_stamped.pose.position.z

        if z == 0:
            rospy.logwarn("Z=0 in pose, can't project to image")
            return None

        fx = self.K[0,0]
        fy = self.K[1,1]
        cx = self.K[0,2]
        cy = self.K[1,2]

        u = int((fx * x / z) + cx)
        v = int((fy * y / z) + cy)

        return u, v

    def detect_color(self, pixel_coords):
        """
        Detect dominant color in a region around given pixel coordinates.
        
        Args:
            pixel_coords: (u, v) pixel coordinates
            
        Returns:
            str: Dominant color ('red', 'green', 'blue') or None if undetermined
        """
        
        if self.latest_image is None:
            rospy.logwarn("No image received yet.")
            return None

        if pixel_coords is None:
            return None

        u, v = pixel_coords
        roi_size = 50  # Size of region of interest around the pixel

        # Check if pixel coordinates are within image bounds
        height, width, _ = self.latest_image.shape
        if u < 5 or u > width - 5 or v < 5 or v > height - 5:
            rospy.logwarn("Pixel coords near edge of image; skipping color detection.")
            return None

        # Extract ROI around the pixel
        x_min = max(u - roi_size, 0)
        x_max = min(u + roi_size, width)
        y_min = max(v - roi_size, 0)
        y_max = min(v + roi_size, height)
        roi = self.latest_image[y_min:y_max, x_min:x_max]

        # Convert to HSV color space for better color detection
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for colors (red has two ranges due to HSV wrap-around)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        lower_green = np.array([40, 70, 50])
        upper_green = np.array([80, 255, 255])
        lower_blue = np.array([100, 70, 50])
        upper_blue = np.array([130, 255, 255])

        # Create masks for each color
        mask_red1 = cv2.inRange(hsv_roi, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_roi, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv_roi, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv_roi, lower_blue, upper_blue)

        # Count non-zero pixels in each mask
        red_count = cv2.countNonZero(mask_red)
        green_count = cv2.countNonZero(mask_green)
        blue_count = cv2.countNonZero(mask_blue)

        # Determine dominant color
        counts = {'red': red_count, 'green': green_count, 'blue': blue_count}
        dominant_color = max(counts, key=counts.get)

        # Only return color if detection is confident
        if counts[dominant_color] < 10:  # Adjust threshold as needed
            rospy.loginfo("No dominant color detected.")
            return None

        return dominant_color

if __name__ == '__main__':
    try:
        ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


