import json
import time
import rclpy
from techshare_ros_pkg2.action import SendCommand
from rclpy.node import Node
from rclpy.clock import ROSClock
from rclpy.duration import Duration 
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from geometry_msgs.msg import Twist
from .library.transform import TRANSFORM
from techshare_ros_pkg2.action import SendCommand
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

class MOVE_BASE_CLIENT(Node):
    def __init__(self):
        super().__init__('move_base_proxy_node')

        self.nav_as= ActionServer(
            self,
            SendCommand,
            "navigation_command",
            execute_callback=self.on_nav_command_recieved,
            goal_callback=None,
            cancel_callback=None,
        )

        self.waypoints_as= ActionServer(
            self,
            SendCommand,
            "waypoints_command",
            execute_callback=self.on_waypoints_command_recieved,
            goal_callback=None,
            cancel_callback=None,
        )



        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('world_frame', 'map')


        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.world_frame = self.get_parameter("world_frame").value
        self.transform = TRANSFORM(self.world_frame, self.map_frame)
        self.incoming_client = ActionClient(self, SendCommand,'incoming_command')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.init_pos_topic_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.buffer = Buffer()
        self.nav = BasicNavigator()
        self.listener = TransformListener(self.buffer, self)
        self.connection = self.incoming_client.wait_for_server(5.0)  
        self.rate = self.create_rate(10)
        self.goal_duration = 0
        self.goal_poses = []

    def move_base_response_callback(self, navigation_goal):
        close_enough = False
        i = 0
        while not self.nav.isTaskComplete():
            i +=1
            feedback = self.nav.getFeedback()
            g_dis = feedback.distance_remaining #meters

            if feedback and i % 5 == 0:
                    self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(
                    feedback.distance_remaining) + ' meters.')
            now = self.nav.get_clock().now()
            # Some navigation timeout to demo cancellation
            if now - self.nav_start > Duration(seconds=100000000.0):
                self.get_logger().info('MOVE BASE Goal canceled due to timeout')
                self.nav.cancelTask()

            if "waypoint" in self.action  and g_dis < 0.5: #less than 50 cm
                self.get_logger().info("CLOSE ENOUGH TO THE GOAL")
                navigation_goal.succeed()
                close_enough = True
                break
            time.sleep(0.01)

        if not close_enough:
            result = self.nav.getResult()
            
            if result == TaskResult.SUCCEEDED and "waypoint" not in self.action:
                self.get_logger().info('MOVE BASE Goal succeeded')
                self.get_logger().info("START %s action here" %(self.action))
                message = SendCommand.Goal()
                message.command = str(self.action).upper()
                message.duration = self.goal_duration
                message.to = "INCOMING"
                message["param"] = ""
                if not self.goal_flag:
                    if self.connection:  
                        result = self.send_command(message)
                    if result == True:
                        navigation_goal.succeed()
                    else:
                        navigation_goal.abort()
                    self.goal_flag = True
        
            elif result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal was succeeded!')
                navigation_goal.succeed()
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
                navigation_goal.canceled()
            elif result == TaskResult.FAILED:
                self.get_logger().info('Goal failed!')
                navigation_goal.abort()
            else:
                self.get_logger().info('Goal has an invalid return status!')    
                navigation_goal.canceled()


    def waypoints_response_callback(self, navigation_goal): 
        i = 0
        result = TaskResult.UNKNOWN
        for waypoint in self.move_base_goals:
            self.nav.goToPose(waypoint)
            self.get_logger().info('Heading to  %d th waypoint'% i)
            i +=1
            j = 0
            while not self.nav.isTaskComplete():
                j += 1
                feedback = self.nav.getFeedback()
                g_dis = feedback.distance_remaining #meters
                if feedback and j % 5 == 0:
                    # if len(self.goal_poses) >0:
                    #     self.get_logger().info('Executing current waypoint:{0}'.format(str(feedback.current_waypoint + 1) + '/' + str(len(self.goal_poses))))
                    self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(
                        feedback.distance_remaining) + ' meters.')
                now = self.nav.get_clock().now()
                # Some navigation timeout to demo cancellation
                if now - self.nav_start > Duration(seconds=120.0):
                    self.get_logger().info('MOVE BASE Goal canceled due to timeout')
                    self.nav.cancelTask()
                    break
                if "waypoint" in self.action and g_dis < 0.1: #less than 10 cm
                    self.get_logger().info("CLOSE ENOUGH TO THE GOAL")
                    self.nav.cancelTask()
                    result = TaskResult.SUCCEEDED
                time.sleep(0.01)

        if result != TaskResult.SUCCEEDED:
            result = self.nav.getResult()    
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal was succeeded!')
            navigation_goal.succeed()
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
            navigation_goal.canceled()
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
            navigation_goal.abort()
        else:
            self.get_logger().info('Goal has an invalid return status!')    
            navigation_goal.canceled()


    def send_command(self, goal):
        print("Sending goal %s" % goal.command)
        self._send_incoming_future = self.incoming_client.send_goal_async(goal)
        self._send_incoming_future.add_done_callback(self.command_response_callback)


    def command_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('INCOMING Goal rejected :(')
            return
        self.get_logger().info('INCOMING Goal accepted :)')
        action_flag = True
        rclpy.spin_until_future_complete(self.incoming_client, future)
        if not goal_handle.result().result:
            action_flag = False
        return action_flag

    

            # if the sent message if for action command
    def on_nav_command_recieved(self, goal):
        # self.get_logger().info(goal)

        command = goal.request.command
        # sender = goal.to
        # duration = goal.duration
        goal_pose = json.loads(goal.request.param)
        self.get_logger().info('MESSAGE: {0}'.format(goal_pose))
        if command == "ACK":
            self.get_logger().info("Recieved ack.")

        elif command == "NAVIGATION" or command == "WAYPOINT":
            self.get_logger().info("NAVIGATION command")
            self.get_logger().info('Result: {0}'.format(goal_pose))
            timestamp = self.get_clock().now().to_msg()
            self.goal_x = goal_pose["x"]
            self.goal_y = goal_pose["y"]
            self.goal_th = goal_pose["th"]
            self.action = goal_pose["action"]
            self.goal_duration = goal.request.duration
            self.goal_flag = False   
            mvoe_base_goal = self.transform.goal_pose(self.goal_x, self.goal_y, self.goal_th, timestamp)  
            self.nav_start = self.nav.get_clock().now()
            try:
                self.nav.cancelTask()
            except:
                self.get_logger().info('navigatioin is not running right now')
            self.nav.goToPose(mvoe_base_goal)
            self.move_base_response_callback(goal)
            result = SendCommand.Result()
            result.done = True
            self.get_logger().info('Result: {0}'.format(result))
            return result

        elif command == "INITPOSE":
            timestamp = self.get_clock().now().to_msg()
            init_pose = self.transform.init_pose(goal_pose["x"], goal_pose["y"], goal_pose["th"], timestamp)
            self.nav.waitUntilNav2Active()
            self.nav.setInitialPose(initial_pose=init_pose)
            goal.succeed()
            result = SendCommand.Result()
            result.done = True
            self.get_logger().info('Result: {0}'.format(result))
            return result
        
    def on_waypoints_command_recieved(self, goal):
        command = goal.request.command
        waypoints = json.loads(goal.request.param)
        # self.get_logger().info('command: {0}'.format(command))
        # self.get_logger().info('waypoints: {0}'.format(waypoints))
        self.move_base_goals =[]
        for waypoint in waypoints:
            timestamp = self.get_clock().now().to_msg()
            x = waypoint["x"]
            y = waypoint["y"]
            th = waypoint["th"]
            self.action = "waypoints"
            self.get_logger().info(f"x: {x}, y: {y}, th: {th}")
            self.move_base_goals.append(self.transform.goal_pose(x, y, th, timestamp))

        self.nav_start = self.nav.get_clock().now()
        # self.nav.followWaypoints(self.move_base_goals)
        try:
            self.nav.cancelTask()
        except:
            self.get_logger().info('navigatioin is not running right now')
        # self.nav.goThroughPoses(self.move_base_goals) #<- it is not working sometimes when having a large map
        # self.move_base_response_callback(goal)
        self.waypoints_response_callback(goal)
        self.goal_poses = []
        result = SendCommand.Result()
        result.done = True
        self.get_logger().info('Result: {0}'.format(result))
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MOVE_BASE_CLIENT()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()