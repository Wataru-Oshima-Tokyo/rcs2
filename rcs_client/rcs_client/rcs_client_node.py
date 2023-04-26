import os
import json
import websocket
import time
import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from .library.socket_class import SOCKET
# from rcs_client.rcs_client
from threading import Thread
from techshare_ros_pkg2.action import SendCommand
from techshare_ros_pkg2.msg import RcsTelemetry
from action_msgs.msg import GoalStatus


class CLIENT(Node):
    def __init__(self):
        super().__init__('rcs_client_node')
        # Get name from namespace for testing.
        # On production robots, ideally get from ENV VARS
        self.robot_name = os.getenv("ROBOT_NAME")
        self.server_address = os.getenv("RCS_SERVER_ADDRESS")
        self.map_id = os.getenv("MAP_ID")
        self.declare_parameter('rcs_server_address', 'localhost')
        if self.robot_name is None:
            self.robot_name = self.get_namespace().strip("/")
        if self.server_address is None:
            self.server_address = self.get_parameter('rcs_server_address').value

        self.socket = SOCKET(self.robot_name, self.map_id, self)
        self.result_dict = {}
        self.ws_thread = Thread(target=self.websocket_handler)
        self.ws_thread.daemon = True
        self.goal_flag = False
        self.ws_thread.start()
        self.ros_handler()


    def ros_handler(self):
        
        # ROS services

        # ROS subscribers
        # Create a subscriber to "/rcs_telemetry"
        self.coordinate_sub = self.create_subscription(
            RcsTelemetry,
            "rcs_telemetry",
            self.coordinate_cb,
            10  # QoS profile depth
        )

        self._as = ActionServer(
            self,
            SendCommand,
            "msg_command",
            execute_callback=self.send_command_action,
            goal_callback=None,
            cancel_callback=None,
        )

        # Create a SimpleActionClient for "incoming_command"
        self.incoming_client = ActionClient(
            self,
            SendCommand,
            "incoming_command",
        )

        # Create a SimpleActionClient for "navigation_command"
        self.navigation_client = ActionClient(
            self,
            SendCommand,
            "navigation_command"
        )
        # Create a SimpleActionClient for "navigation_command"
        self.waypoints_client = ActionClient(
            self,
            SendCommand,
            "waypoints_command"
        )

        self.incoming_connection = self.incoming_client.wait_for_server(5.0)
        self.navigation_connection = self.navigation_client.wait_for_server(5.0)
    
    
    def on_open(self,ws):
        self.get_logger().info("WebSocket connection opened.")
        # send a message to the server to request some data
        self.socket.connection_status = True
        self.socket.initial_connection = True


    def on_error(self,ws, error):
        self.get_logger().info('WebSocket error:{0}'.format(error))

    def on_close(self,ws, a,b):
        self.get_logger().info("WebSocket connection closed.")


    def websocket_handler(self):
        while True:  # Infinite loop in thread for retrying open on disconnect
            self.socket.ws = websocket.WebSocketApp(
                "ws://%s:5000/ws/dispatch/robot/%s/"
                % (self.server_address, self.robot_name),
                on_message=self.on_message_received,
                on_open=self.on_open,
                on_error=self.on_error,
                on_close=self.on_close
            )
            self.socket.ws.run_forever()
            self.get_logger().info("Websocket connection lost, retrying after 5 seconds.")
            self.socket.connection_status = False
            time.sleep(5)

    def coordinate_cb(self, msg):
        self.get_logger().debug("sending telemetry")
        current_condition = self.socket.setCondition(msg)
        self.socket.send_telemetry(current_condition)



    # This is called when the msgtype is "navigation"
    def nav_command(self, message):
        if message["msgtype"] == "ACK":
            self.get_logger().info("Received ack.")
        else:
            action_flag = False
            try:
                self.get_logger().info("NAVIGATION CONNECTION: %s" %self.navigation_connection)
                if self.navigation_connection:
                    goal = SendCommand.Goal()
                    goal.command = message["msgtype"]
                    goal.duration = message["duration"]
                    goal.to = message["sender"]
                    pose = {
                        "x": message["x"],
                        "y": message["y"],
                        "th": message["th"],
                        "action": message["action"],
                    }
                    pose_str = json.dumps(pose)
                    goal.param = pose_str
                    self.get_logger().info("Sending goal %s" % goal.command)
                    goal_handle = self.navigation_client.send_goal_async(goal)
                    goal_handle.add_done_callback(lambda future: self.nav_goal_response_callback(future, message))
                    # Below is a busy loop to block other signals
                    # rclpy.spin_until_future_complete(self, goal_handle)

                else:
                    self.get_logger().info("Failed")
                    action_flag = False
            except:
                self.get_logger().info("Exception occurred")
                action_flag = False

            
            return action_flag

        # This is called when the msgtype is "navigation"
    def waypoint_command(self, message):
        if message["msgtype"] == "ACK":
            self.get_logger().info("Received ack.")
        else:
            action_flag = False
            try:
                self.get_logger().info("WAYPOINTS CONNECTION: %s" %self.navigation_connection)
                if self.navigation_connection:
                    goal = SendCommand.Goal()
                    goal.command = message["msgtype"]
                    goal.duration = message["duration"]
                    goal.to = message["sender"]
                    goal.param = message['waypoints']
                    self.get_logger().info("Sending waypoints %s" % goal.command)
                    goal_handle = self.waypoints_client.send_goal_async(goal)
                    goal_handle.add_done_callback(lambda future: self.waypoints_goal_response_callback(future, message))
                    # Below is a busy loop to block other signals
                    # rclpy.spin_until_future_complete(self, goal_handle)

                else:
                    self.get_logger().info("Failed")
                    action_flag = False
            except:
                self.get_logger().info("Exception occurred")
                action_flag = False

            
            return action_flag



    def nav_goal_response_callback(self, future, message):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('NAVIGATION Goal rejected :(')
            return

        self.get_logger().info('NAVIGATION Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda future: self.get_result_callback(future, message))
        
    def waypoints_goal_response_callback(self, future, message):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('WAYPOINTS Goal rejected :(')
            return

        self.get_logger().info('WAYPOINTS Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda future: self.get_result_callback(future, message))

    def get_result_callback(self, future, message):
        result = future.result().result
        status = future.result().status
        action_flag = True
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
            action_flag = False
        self.get_logger().info('get_result_callback: {0}'.format(result))
        self.socket.send_navigation_response(message["sender"], message["command_id"], action_flag)



    # If the sent message is for an action command
    def handle_command(self, message):
        if message["msgtype"] == "ACK":
            self.get_logger().info("Received ack.")
        elif message["msgtype"] == "COMMAND":
            self.get_logger().info("Received command")
            action_flag = False
            self.get_logger().info("INCOMING CONNECTION: %s" %self.incoming_connection)
            if self.incoming_connection:
                goal = SendCommand.Goal()
                goal.command = message["command"]
                goal.duration = message["duration"]
                goal.to = message["sender"]
                goal.param = message["param"]
                self.get_logger().info("Sending goal %s" % goal.command)
                goal_handle = self.incoming_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, goal_handle)

                if goal_handle.result() and goal_handle.result().success:
                    self.get_logger().info("Goal completed successfully")
                    action_flag = True
                else:
                    self.get_logger().info("Goal failed with status code: {}".format(goal_handle.result().status))
                    action_flag = False
            else:
                self.get_logger().info("Failed")
                action_flag = False

            self.socket.send_response(
                message["sender"], message["command_id"], action_flag
            )
            return action_flag

        elif message["msgtype"] == "RESPONSE":
            if message["result"] == "DESTINATIONNOTFOUND":
                self.get_logger().info("ERROR DESTINATION NOT FOUND!")
                message["result"] = False
            self.result_dict[message["command_id"]] = message["result"]


    def send_command_action(self, goal_handle):
        command_id = str(random.random())
        self.result_dict[command_id] = None
        self.get_logger().info("Send %s command to %s" % (str(goal_handle.request.command), str(goal_handle.request.to)))
        self.socket.send_command(
            command_id, str(goal_handle.request.command), str(goal_handle.request.to), goal_handle.request.duration, goal_handle.request.param
        )
        now = self.get_clock().now()
        rate = self.create_rate(10)  # 10 Hz
        print_timer = 0
        feedback_msg = SendCommand.Feedback()
        feedback_msg.sequence = 0
        feedback_msg.feedback_rate = 10

        while self.result_dict[command_id] is None:
            goal_handle.publish_feedback(feedback_msg)

            if now + self.create_duration_from_sec(int(goal_handle.request.duration) + 10) < self.get_clock().now():
                self.result_dict[command_id] = False
            else:
                if now + self.create_duration_from_sec(int(goal_handle.request.duration) + print_timer) < self.get_clock().now():
                    self.get_logger().info('Waiting for result')
                    print_timer += 1

            rate.sleep()

        if self.result_dict[command_id]:
            self.get_logger().info('%s: Succeeded' % goal_handle.request.command)
            result = SendCommand.Result()
            result.sequence = 0
            goal_handle.succeded()
        else:
            self.get_logger().info('%s: Aborted' % goal_handle.request.command)
            goal_handle.aborted()

        del self.result_dict[command_id]


    def on_message_received(self, ws, message):
        message = json.loads(message)
        self.get_logger().info("received")
        if "navigation" in message:
            try:
                message = message["navigation"]
                self.get_logger().info("Message received %s" % message)
                self.nav_command(message)
            except:
                self.get_logger().info("parsing error")
        elif "command" in message:
            try:
                message = message["command"]
                self.get_logger().info("Message received %s" % message)
                self.handle_command(message)
            except:
                self.get_logger().info("parsing error")
        elif "waypoints" in message:
            try:
                message = message["waypoints"]
                self.get_logger().info("Message received %s" % message)
                self.waypoint_command(message)
            except:
                self.get_logger().info("parsing error")


def main(args=None):
    rclpy.init(args=args)
    node = CLIENT()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()