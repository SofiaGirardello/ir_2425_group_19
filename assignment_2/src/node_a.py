#!/usr/bin/env python

"""
@file   : node_a.py
@brief  : Node to calculate and continuously publish placement positions based on line coefficients.

@details: This node requests line coefficients from a service, calculates positions along the line, 
          and publishes them as Point messages to the /placement_positions topic continuously.

@date   : 2024-1-20
@authors: Caduceo Andrea, Girardello Sofia, Gizzarone Manuel 
"""

import rospy
from tiago_iaslab_simulation.srv import Coeffs
from geometry_msgs.msg import Point

class PlacementLineNode:
    def __init__(self):
        rospy.init_node('node_a_line_placement')
        self.service_name = '/straight_line_srv'
        self.placement_pub = rospy.Publisher('/placement_positions', Point, queue_size=10)

        rospy.loginfo("Waiting for /straight_line_srv service...")
        rospy.wait_for_service(self.service_name)
        rospy.loginfo("Service available. Sending request...")

        # Get line coefficients once
        self.coeffs = self.get_line_coefficients()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Check if there is at least a subscriber to the topic before publishing
            if self.placement_pub.get_num_connections() > 0:
                self.publish_positions() 
                break
            rate.sleep()

    def get_line_coefficients(self):
        try:
            get_coeffs = rospy.ServiceProxy(self.service_name, Coeffs)
            response = get_coeffs(ready=True)
            rospy.loginfo(f"Received coefficients: m = {response.coeffs[0]}, q = {response.coeffs[1]}")
            return response.coeffs
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def publish_positions(self):
        if self.coeffs:
            m, q = self.coeffs[0], self.coeffs[1]
            positions = []
            for x in range(1, 4): # Example x values, could be adjusted or made configurable
                x = x/10  
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

