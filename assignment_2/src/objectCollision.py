#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_commander import PlanningSceneInterface

class AddCollisionObject:
    def __init__(self):
        rospy.init_node('objectCollision')

        # Initialize MoveIt! components
        self.scene = PlanningSceneInterface()
        moveit_commander.roscpp_initialize(sys.argv)

        # Add collision objects for the tables
        self.add_pick_and_place_tables()

    def add_pick_and_place_tables(self):
        # Define the solid box for the tables
        table_primitive = SolidPrimitive()
        table_primitive.type = table_primitive.BOX
        table_primitive.dimensions = [0.95, 0.95, 0.95]

        # Define the pick table pose in the map frame
        pick_table_pose = Pose()
        pick_table_pose.position.x = 7.826
        pick_table_pose.position.y = -2.983
        pick_table_pose.position.z = 0.775 / 2  # Half the height of the table
        pick_table_pose.orientation.w = 1.0

        # Create the collision object for the pick table
        pick_collision_object = moveit_commander.CollisionObject()
        pick_collision_object.header.frame_id = "map"
        pick_collision_object.id = "pick_table"
        pick_collision_object.primitives.append(table_primitive)
        pick_collision_object.primitive_poses.append(pick_table_pose)
        pick_collision_object.operation = pick_collision_object.ADD

        # Define the place table pose in the map frame
        place_table_pose = Pose()
        place_table_pose.position.x = 7.895753
        place_table_pose.position.y = -1.923828
        place_table_pose.position.z = 0.35887
        place_table_pose.orientation.w = 1.0

        # Create the collision object for the place table
        place_collision_object = moveit_commander.CollisionObject()
        place_collision_object.header.frame_id = "map"
        place_collision_object.id = "place_table"
        place_collision_object.primitives.append(table_primitive)
        place_collision_object.primitive_poses.append(place_table_pose)
        place_collision_object.operation = place_collision_object.ADD

        # Add the collision objects to the scene
        self.scene.add_object(pick_collision_object)
        self.scene.add_object(place_collision_object)

        rospy.loginfo("Collision objects added to the scene!")

if __name__ == "__main__":
    try:
        AddCollisionObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
