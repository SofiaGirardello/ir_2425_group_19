#!/usr/bin/env python

import rospy
import moveit_commander
import tf
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import PlanningSceneInterface
from shape_msgs.msg import SolidPrimitive

class addCollisionObject:
    
    def add_pick_and_place_tables(self, scene):
        
        # Define the solid box for the tables
        table_primitive = SolidPrimitive()
        table_primitive.type = table_primitive.BOX
        table_primitive.dimensions = [0.9, 0.9, 0.9]

        # Define the pick table pose in map frame
        pick_table_pose = Pose()
        pick_table_pose.position.x = 7.913443
        pick_table_pose.position.y = -3.016501
        pick_table_pose.position.z = 0.305887
        pick_table_pose.orientation.w = 1.0

        # Create the collision object for the pick table
        pick_collision_object = moveit_commander.CollisionObject()

        # Initialize the collision object attributes
        pick_collision_object.header.frame_id = "map"  
        pick_collision_object.id = "pick_table"  
        pick_collision_object.primitives.append(table_primitive)
        pick_collision_object.primitive_poses.append(pick_table_pose)

        # Define the place table pose in map frame
        place_table_pose = Pose()
        place_table_pose.position.x = 7.895753
        place_table_pose.position.y = -1.923828
        place_table_pose.position.z = 0.305887
        place_table_pose.orientation.w = 1.0

        # Create the collision object for the place table
        place_collision_object = moveit_commander.CollisionObject()

        # Initialize the collision object attributes
        place_collision_object.header.frame_id = "map"  
        place_collision_object.id = "place_table"  
        place_collision_object.primitives.append(table_primitive)
        place_collision_object.primitive_poses.append(place_table_pose)

        # Add the collision objects
        pick_collision_object.operation = pick_collision_object.ADD
        place_collision_object.operation = place_collision_object.ADD

        # Add the object to the planning scene
        scene.add_object(pick_collision_object)
        scene.add_object(place_collision_object)
        rospy.loginfo("Collision objects added to the scene!")

    def __init__(self):

        # Initialize the node 'objectCollision'
        rospy.init_node('objectCollision')

        # Initialize the collision object attributes
        scene = PlanningSceneInterface()

        self.add_pick_and_place_tables(scene)
    


if __name__ == "__main__":
    try:
        addCollisionObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
