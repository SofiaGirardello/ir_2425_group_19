#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_commander import PlanningSceneInterface, CollisionObject
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class AddCollisionObject:
    """
    Class to manage the addition and removal of collision objects in the MoveIt! planning scene.

    Functionalities:
    - Add static collision objects (tables).
    - Add dynamic collision objects (pickable items) based on tag IDs.
    - Remove objects from the planning scene after picking.
    """
    
    def __init__(self):
        """
        Initializes the planning scene interface and adds static objects (tables).
        """

        # Initialize MoveIt! components
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = PlanningSceneInterface()
        rospy.sleep(1.0)

        # Cache currently known objects to avoid duplication
        self.current_objects = set(self.scene.get_known_object_names())

        # Placeholders for item geometry
        self.itempose = Pose()
        self.solidprimitive = SolidPrimitive()

        # Add the tables to the scene if not already present
        self.add_pick_and_place_tables()

    def add_pick_and_place_tables(self):
        """
        Adds static collision objects representing the pick and place tables.
        """

        # Check to prevent adding duplicates
        if "pick_table" in self.current_objects and "place_table" in self.current_objects:
            rospy.loginfo("Tables already in the scene!")
            return
        
        # Table geometry: box of defined size
        table_primitive = SolidPrimitive()
        table_primitive.type = SolidPrimitive.BOX
        table_primitive.dimensions = [0.90, 0.90, 0.776]

        # Pick table pose 
        pick_table_pose = Pose()
        pick_table_pose.position.x = 7.917558
        pick_table_pose.position.y = -3.01654
        pick_table_pose.position.z = 0.776 / 2  # Due to how BOX 'height' parameter works 
        pick_table_pose.orientation.x = 0.0
        pick_table_pose.orientation.y = 0.0
        pick_table_pose.orientation.z = 0.0
        pick_table_pose.orientation.w = 1.0

        # Collision object definition
        pick_collision_object = moveit_commander.CollisionObject()
        pick_collision_object.header.frame_id = "map"
        pick_collision_object.id = "pick_table"
        pick_collision_object.primitives.append(table_primitive)
        pick_collision_object.primitive_poses.append(pick_table_pose)
        pick_collision_object.operation = CollisionObject.ADD

        # Place table pose 
        place_table_pose = Pose()
        place_table_pose.position.x = 7.897433
        place_table_pose.position.y = -1.923835
        place_table_pose.position.z = 0.776 / 2
        place_table_pose.orientation.x = 0.0
        place_table_pose.orientation.y = 0.0
        place_table_pose.orientation.z = 0.0
        place_table_pose.orientation.w = 1.0

        # Collision object definition
        place_collision_object = moveit_commander.CollisionObject()
        place_collision_object.header.frame_id = "map"
        place_collision_object.id = "place_table"
        place_collision_object.primitives.append(table_primitive)
        place_collision_object.primitive_poses.append(place_table_pose)
        place_collision_object.operation = CollisionObject.ADD

        # Add the collision objects to the scene
        self.scene.add_object(pick_collision_object)
        self.scene.add_object(place_collision_object)
        rospy.loginfo("Tables collision objects added to scene!")

    def add_new_collision_object(self, tag_id, pos_out, ref_frame):
        """
        Adds a new collision object to the scene based on object ID and pose.
        """

        # Determine the shape and pose offset
        self.solidprimitive, self.itempose = self.map_id_to_solid(tag_id, pos_out)

        # Define the collision object
        collision_object = moveit_commander.CollisionObject()
        collision_object.header.frame_id = ref_frame
        collision_object.id = str(tag_id)
        collision_object.primitives.append(self.solidprimitive)
        collision_object.primitive_poses.append(self.itempose)
        collision_object.operation = CollisionObject.ADD

        # Add the collision object to the scene
        self.scene.add_object(collision_object)
        rospy.loginfo(f"Object collision added for tag {tag_id}.")

    def remove_collision_object(self, tag_id):
        """
        Removes a collision object from the scene by ID.
        """
        
        tag_id_str = str(tag_id)
        
        # Check if the object exists in the scene
        current_objects = self.scene.get_known_object_names()
        
        if tag_id_str in current_objects:
            self.scene.remove_world_object(tag_id_str)
            rospy.loginfo(f"Collision object '{tag_id_str}' removed from scene.")
        else:
            rospy.logwarn(f"Collision object '{tag_id_str}' not present in the scene.")


    def map_id_to_solid(self, id, pose):
        """
        Maps a tag ID to the associated apriltag's shape and dimensions.
        """
        
        solidprimitive = SolidPrimitive()
        # Extract Pose from PoseStamped
        itempose = pose.pose

        # Shape ID groups
        list_hexagon = [1, 2, 3]
        list_cube = [4, 5, 6]
        list_triangular = [7, 8, 9]

        # Shape dimensions
        cylinder_dimension = [0.1, 0.03]                    # height, radius
        cube_dimension = [0.05, 0.05, 0.05]                 # length, width, height
        parallelepiped_dimension = [0.05, 0.07, 0.035]      # length, width, height
        
        # Assign shape and z-offset for collision modeling
        if id in list_hexagon:
            solidprimitive.type = SolidPrimitive.CYLINDER
            solidprimitive.dimensions = cylinder_dimension
            itempose.position.z -= 0.05 # Adjust to base of object

        if id in list_cube:
            solidprimitive.type = SolidPrimitive.BOX
            solidprimitive.dimensions = cube_dimension
            itempose.position.z -= 0.025 # Adjust to base of object

        if id in list_triangular:
            solidprimitive.type = SolidPrimitive.BOX
            solidprimitive.dimensions = parallelepiped_dimension
            itempose.position.z -= 0.0175 # Adjust to base of object

        return solidprimitive, itempose
