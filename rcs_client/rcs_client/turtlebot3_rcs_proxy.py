
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient, ActionServer
from .library.socket_class import SOCKET
# from rcs_client.rcs_client
from threading import Thread
from techshare_ros_pkg2.action import SendCommand
from techshare_ros_pkg2.msg import RcsTelemetry
from techshare_ros_pkg2.msg import Pose
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

# Proxy specific setup (subscribe to messages required for telemetry package)
# and any actions required by incoming commands.
class Proxy(Node):
    def __init__(self):
        super().__init__('RCSProxy_turtlebot3')
        self.action_server = ActionServer(
            self,
            SendCommand,
            'incoming_command',
            execute_callback=self.on_command_received
        )


        self.position_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.position_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.publish_rcs_telemetry = self.create_publisher(
            RcsTelemetry,
            'rcs_telemetry',
            10
        )
        self.camera_sub = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.image_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.rcs_telemetry_msg = RcsTelemetry()
        self.rcs_telemetry_publisher = self.create_publisher(RcsTelemetry, 'rcs_telemetry', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.rcs_telemetry_publish)
        
    
    def rcs_telemetry_publish(self):
        self.rcs_telemetry_publisher.publish(self.rcs_telemetry_msg)
        # self.get_logger().info('Publishing a rcs telemetry')


    def on_command_received(self, goal_handle):
        command = goal_handle.request.command
        feedback_msg = SendCommand.Feedback()
        feedback_msg.rate = 10

        if command == 'CHARGE':
            self.get_logger().info('Turtlesim does not need to charge!')
            result = SendCommand.Result()
            result.result = True
            goal_handle.succeed(result)
        elif command == 'REMOVE_ARM':
            self.get_logger().info('Turtles do not have arms :c')
            result = SendCommand.Result()
            result.result = True
            goal_handle.succeed(result)
        else:
            self.get_logger().info('Command not recognized!')
            goal_handle.abort()

        goal_handle.publish_feedback(feedback_msg)


    def battery_callback(self, msg):
        self.rcs_telemetry_msg.batterylevel = 100  # msg.bms.SOC

    def position_callback(self, msg):
        self.rcs_telemetry_msg.currentcoordinate = msg.pose.pose

    def image_callback(self, msg):
        self.rcs_telemetry_msg.img = msg


def main(args=None):
    rclpy.init(args=args)
    node = Proxy()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()