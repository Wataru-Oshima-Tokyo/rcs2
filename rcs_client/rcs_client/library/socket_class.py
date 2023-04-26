import time
import json
from tf_transformations import euler_from_quaternion
import base64
import cv2
from cv_bridge import CvBridge,CvBridgeError
import numpy as np

class CONDITION:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.status = 0
        self.battery_level = 0
        self.image = ""


class SOCKET:
    def __init__(self, robot_name, map_id, client):
        self.robot_name = robot_name
        self.message = None
        self.connection_status = False
        self.initial_connection = False
        self.ws = None
        self.map_id = map_id
        self.client = client
        self.bridge = CvBridge()

    def create_base_message(self, msg_type):
        return {
            "sender": self.robot_name,
            "timestamp": time.time(),
            "msgtype": msg_type,
        }

    def send_command(self, command_id, command, destination, timeout_duration, param):
        message = self.create_base_message("COMMAND")
        message.update(
            {
                "destination": destination,
                "command": command,
                "command_id": command_id,
                "duration": timeout_duration,
                "param": param,
            }
        )
        self.send_msg(message)

    def send_initial_info(self):
        status = self.create_base_message("INITIAL_CONNECTION")
        status.update(
            {
                "destination": "RCS",
                "map_id": self.map_id,
            }
        )
        self.send_msg(status)

    def send_response(self, destination, command_id, result):
        response = self.create_base_message("RESPONSE")
        response.update(
            {
                "destination": destination,
                "command_id": command_id,
                "result": result,
            }
        )
        self.send_msg(response)

    def send_navigation_response(self, destination, command_id, result):
        response = self.create_base_message("NAVIGATION_RESPONSE")
        response.update(
            {
                "destination": destination,
                "command_id": command_id,
                "result": result,
            }
        )
        self.send_msg(response)

    def send_msg(self, payload):
        message = json.dumps({"message": payload})
        if payload["msgtype"] != "TELEMETRY":
            self.client.get_logger().info(message)
        if self.connection_status:
            self.ws.send("%s" % message)

    def send_telemetry(self, data):
        if self.connection_status and self.initial_connection:
            ## Sending a map id but you can add more info
            try:
                self.send_initial_info()
                self.initial_connection = False
            except:
                self.client.get_logger().info("fake connection")
        """
        Sends telemetry, intended to be a callback for a telemetry publisher.
        Overload this in subclass to change
        """
        payload = self.create_base_message("TELEMETRY")
        payload.update(
            {
                "x": data.x,
                "y": data.y,
                "theta": data.theta,
                "status": data.status,
                "battery_level": data.battery_level,
                "img": data.image,
            }
        )
        self.send_msg(payload)

    def setCondition(self, msg):
        condition = CONDITION()
        condition.x = msg.currentcoordinate.position.x
        condition.y = msg.currentcoordinate.position.y
        x = msg.currentcoordinate.orientation.x
        y = msg.currentcoordinate.orientation.y
        z = msg.currentcoordinate.orientation.z
        w = msg.currentcoordinate.orientation.w
        (roll, pitch, yaw) = euler_from_quaternion((x, y, z, w))
        condition.theta = yaw
        condition.status = msg.robotstatus
        condition.battery_level = msg.batterylevel
        # covert to opencv_image and then encode to jpg and then encode in b64
        if msg.img.encoding != "":
            cv_img = self.bridge.imgmsg_to_cv2(msg.img)
            cv_img = cv2.resize(cv_img, (640, 480))
            condition.image = base64.b64encode(cv2.imencode(".jpg", cv_img)[1])
        else:
            condition.image = ""
        return condition
