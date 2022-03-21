import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from shape_msgs.msg import Mesh
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander

class CollisionChecking:

    def __init__(self, movegroup) :
        self.move_group = MoveGroupCommander(movegroup)
        self.scene: PlanningSceneInterface = PlanningSceneInterface()
        self.robot: RobotCommander

    def pointcloud_extr(self):
        point_cloud = PointCloud2()
        point_cloud

    def generate_objects(self):
        tree = CollisionObject()
        tree.id = "my_mesh"
        tree.operation = tree.ADD
        rospy.sleep(2)

        scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=30)
        self.scene.remove_world_object(tree.id)

        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 2.0
        p.pose.position.y = 2.0
        p.pose.position.z = 0.
        self.scene.add_mesh(tree.id, p, "/home/anthony/Desktop/SFP/SFP-SimulationFresh/catkin_ws/src/test/worlds/../models/peren.dae")
        scene_pub.publish(PlanningScene)
        rospy.loginfo("Adding "+ tree.id)
        rospy.sleep(2)