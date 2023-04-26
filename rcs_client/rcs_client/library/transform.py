
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped

import math


class TRANSFORM:
    def __init__(self, world_frame, map_frame):
        self.world_frame = world_frame
        self.map_frame = map_frame

    def euler_to_quarternion(self, euler):
        """Convert Euler Angles to Quaternion
        euler: geometry_msgs/Vector3
        quaternion: geometry_msgs/Quaternion
        """
        q = quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # To make yaw from orientation

    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion
        e = euler_from_quaternion((x, y, z, w))
        return e[0], e[1], e[2]

    def goal_pose(self, x, y, th, now):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = now
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        eular = Vector3()
        eular.x = 0.0
        eular.y = 0.0
        try:
            eular.z = th
        except:
            eular.z = 0.0
        quat = self.euler_to_quarternion(eular)
        goal_pose.pose.orientation.x = quat.x
        goal_pose.pose.orientation.y = quat.y
        goal_pose.pose.orientation.z = quat.z
        goal_pose.pose.orientation.w = quat.w

        # # set the tolerance
        # goal_pose.pose_tolerance = 0.1 # set the tolerance in meters

        # # set the planning parameters
        # goal_pose.planner_id = 'GridBased'
        # goal_pose.planner_timeout = 10.0 # set the planning timeout in seconds

        return goal_pose

    def init_pose(self, x, y, theta, now):
        init_pose = PoseStamped()
        init_pose.header.frame_id = self.world_frame
        init_pose.header.stamp = now

        init_pose.pose.position.x = x
        init_pose.pose.position.y = y
        init_pose.pose.position.z = 0.0
        eular = Vector3()
        eular.x = 0.0
        eular.y = 0.0
        try:
            eular.z = theta
        except:
            eular.z = 0.0
        quat = self.euler_to_quarternion(eular)

        init_pose.pose.orientation.x = quat.x
        init_pose.pose.orientation.y = quat.y
        init_pose.pose.orientation.z = quat.z
        init_pose.pose.orientation.w = quat.w

        return init_pose

    def quaternion_to_euler(self, quaternion):
        """Convert Quaternion to Euler Angles
        quarternion: geometry_msgs/Quaternion
        euler: geometry_msgs/Vector3
        """
        e = euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )
        return Vector3(x=e[0], y=e[1], z=e[2])

    def distance(self, dx, dy):
        return math.sqrt(dx * dx + dy * dy)