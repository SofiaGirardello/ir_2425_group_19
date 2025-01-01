#!/usr/bin/env python

"""
@file   : node_a.py
@brief  : 

@details:

@date   : 2024-1-20
@authors : Caduceo Andrea, Girardello Sofia, Gizzarone Manuel 
"""

import rospy
from tiago_iaslab_simulation.srv import Coeffs
from geometry_msgs.msg import Point
import tf  

class PlacementLineNode:
    def __init__(self):
        rospy.init_node('node_a_line_placement')
        self.service_name = '/straight_line_srv'
        self.placement_pub = rospy.Publisher('/placement_positions', Point, queue_size=10)
        
        rospy.loginfo("Waiting for /straight_line_srv service...")
        rospy.wait_for_service(self.service_name)
        rospy.loginfo("Service available. Sending request...")

        self.get_line_coefficients()

    def get_line_coefficients(self):
        try:
            get_coeffs = rospy.ServiceProxy(self.service_name, Coeffs)
            response = get_coeffs(ready=True)
            coeffs = response.coeffs
            rospy.loginfo(f"Received coefficients: m = {coeffs[0]}, q = {coeffs[1]}")
            self.calculate_placement_positions(coeffs[0], coeffs[1])
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def calculate_placement_positions(self, m, q):
        positions = []
        for x in range(1, 5):  # Example x values
            y = m * x + q
            position = Point(x, y, 0)  # Z coordinate is 0
            positions.append(position)
            self.placement_pub.publish(position)
            rospy.loginfo(f"Published placement position: {position}")

if __name__ == '__main__':
    try:
        PlacementLineNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


