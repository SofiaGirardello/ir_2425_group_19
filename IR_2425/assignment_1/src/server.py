#!/usr/bin/env python

"""
@file   : server.py
@brief  : A ROS node for controlling the robot's head tilt for better camera positioning and detecting AprilTags.
          This node communicates with the head controller to adjust the tilt angle of the robot's head,
          and also initializes the `FindAprilTags` class to search for AprilTags in the environment.

@details:
    - This script provides a service to tilt the robot's head to a specific angle for improved camera positioning. 
      The tilt angle is passed as a parameter, with negative values tilting the head downward.
    - The script integrates with the `FindAprilTags` class to handle the detection of AprilTags in the environment.
    - The robot's head tilt is controlled using a `JointTrajectory` message that defines the joint angles for the head.
    - The ROS node listens for incoming service requests from the action client and performs the head tilting operation 
      based on the given input angle.

@date   : 2024-12-23
@authors : Caduceo Andrea, Girardello Sofia, Gizzarone Manuel
"""

import rospy
import actionlib
from assignment_1.search_ids import FindAprilTags
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# TILT HEAD FUNCTION 
def tilt_head(tilt_angle):
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
    point.time_from_start = rospy.Duration(1) 
    
    # Add the trajectory point to the message
    trajectory_msg.points.append(point)
    pub.publish(trajectory_msg)


if __name__ == '__main__':
    try:
        # Initialize the ROS node_b_server (server node)
        rospy.init_node('node_b_server', anonymous=True)

        # Call the function 
        tilt_angle = -0.5  # Angle in rad, negative for downward inclinations
        tilt_head(tilt_angle)

        # Initialize the find apritags class
        find_april_tags = FindAprilTags('find_apriltags')

        rospy.loginfo("Node initialized and waiting for target IDs from the action client...")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Error! Node_b stopped!")


