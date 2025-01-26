#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_commander import PlanningSceneInterface
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class AddCollisionObject:
    def __init__(self):

        # Initialize MoveIt! components
        self.scene = PlanningSceneInterface()
        moveit_commander.roscpp_initialize(sys.argv)

        self.itempose = Pose()
        self.solidprimitive = SolidPrimitive()

        # Add collision objects for the tables
        self.add_pick_and_place_tables()

    def add_pick_and_place_tables(self):

        current_objects = self.scene.get_objects()

        # Check if the collision object is already in the scene
        if "pick_table" in current_objects:
            return
        
        # Define the solid box for the tables
        table_primitive = SolidPrimitive()
        table_primitive.type = table_primitive.BOX
        table_primitive.dimensions = [0.90, 0.90, 0.45]

        # Define the pick table pose in the map frame
        pick_table_pose = Pose()
        pick_table_pose.position.x = 7.826
        pick_table_pose.position.y = -2.983
        pick_table_pose.position.z = 0.75 / 2  # Half the height of the table
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
        place_table_pose.position.z = 0.75 / 2
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

    def add_new_collision_object(self, tag_id, pos_out):

        rospy.loginfo("Start diocane")

        collision_object = moveit_commander.CollisionObject()

        
        self.solidprimitive, self.itempose = self.map_id_to_solid(tag_id, pos_out)

        collision_object.header.frame_id = "map"
        collision_object.id = str(tag_id)
        collision_object.primitives.append(self.solidprimitive)
        collision_object.primitive_poses.append(self.itempose)
        collision_object.operation = collision_object.ADD

        self.scene.add_object(collision_object)

        rospy.loginfo(f"{tag_id}")

    def remove_collision_object(self, tag_id):

        self.scene.remove_world_object(str(tag_id))

    def map_id_to_solid(self, id, pose):

        solidprimitive = SolidPrimitive()
        itempose = pose.pose
        

        object_orientation = itempose.orientation
        _, _, yaw = euler_from_quaternion([object_orientation.x, object_orientation.y, object_orientation.z, object_orientation.w])
        
        # Convert Euler angles (roll, pitch, yaw) back to quaternion
        new_orientation = quaternion_from_euler(0, 0, yaw)

        # Assuming you want to set the orientation in a new pose
        itempose.orientation.x = new_orientation[0]
        itempose.orientation.y = new_orientation[1]
        itempose.orientation.z = new_orientation[2]
        itempose.orientation.w = new_orientation[3]

        list_hexagon = [1, 2, 3]
        list_cube = [4, 5, 6]
        list_triangular = [7, 8, 9]

        cylinder_dimension = [0.1, 0.05]
        cube_dimension = [0.05, 0.05, 0.05]
        parallelepiped_dimension = [0.05, 0.07, 0.035] 

        if id in list_hexagon:
            solidprimitive.type = solidprimitive.CYLINDER
            solidprimitive.dimensions = cylinder_dimension
            itempose.position.z -= 0.05

        if id in list_cube:
            solidprimitive.type = solidprimitive.BOX
            solidprimitive.dimensions = cube_dimension
            itempose.position.z -= 0.025

        if id in list_triangular:
            solidprimitive.type = solidprimitive.BOX
            solidprimitive.dimensions = parallelepiped_dimension
            itempose.position.z -= 0.0175

        return solidprimitive, itempose
