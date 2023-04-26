# dispatch/consumers.py
import json
import math
from asgiref.sync import async_to_sync
import redis
import time
from channels.generic.websocket import WebsocketConsumer
from dispatch.models import Robot, MapData
from dispatch.waypointDistributor import WAYPOINT_DISTRIBUTOR
from dispatch.DispatchTools import convert_coordinates_image_to_map, convert_coordinates_map_to_img


class UserConsumer(WebsocketConsumer):
    """
    User connects to user_ROBOTNAME
    """

    def connect(self):
        self.robot_name = self.scope["url_route"]["kwargs"]["robot_name"]
        self.robot_group_name = "robot_%s" % self.robot_name
        self.user_group_name = "user_%s" % self.robot_name
        r = redis.Redis()
        robot_exists = r.hget("layers", self.robot_group_name)
        print(self.robot_group_name)
        print(robot_exists)
        if robot_exists is None:
            return
        async_to_sync(self.channel_layer.group_add)(
            self.user_group_name, self.channel_name
        )
        r = redis.Redis()
        # Robot joins with time connected
        r.hset("layers", self.user_group_name, time.time())
        self.accept()

    def disconnect(self, close_code):
        # remove self from layer
        async_to_sync(self.channel_layer.group_discard)(
            self.user_group_name, self.channel_name
        )
        r = redis.Redis()
        r.hdel("layers", self.user_group_name)  # removes from redis at  end.

    # Receive message from WebSocket
    def receive(
        self,
        text_data=None,
    ):
        text_data_json = json.loads(text_data)
        msgtype=""
        message=""
        if "message" in text_data_json:
            msgtype="message"
            message = text_data_json["message"]
        elif "navigation" in text_data_json:
            msgtype="navigation"
            message = text_data_json["navigation"]
        elif "waypoints" in text_data_json:
            msgtype = "waypoints"
            message = text_data_json["waypoints"]
        if msgtype != "waypoints":
            async_to_sync(self.channel_layer.group_send)( 
                self.robot_group_name, {"type": "robot_command", msgtype: message}
            )
        else:
            #distribute all the waypoints to each robot
            self.waypoints_command(msgtype, message)
            
    def extraction(self, wpd,robots, message):
        wpd.storeRobots(robots)
        waypoints_json = message['waypoints']
        waypoints_list = json.loads(waypoints_json)
        wpd.storeWaypoints(waypoints_list)

    
    def waypoints_command(self, msgtype,message):
        wpd = WAYPOINT_DISTRIBUTOR()
        robots = Robot.objects.filter(map_id__map_id=message['map_id'])
        print(message)
        self.extraction(wpd, robots, message)
        robot_waypoints = wpd.getPath()
        print(robot_waypoints)
        for key in robot_waypoints.keys():
            message["waypoints"] = wpd.waypointsFormatConverter(robot_waypoints[key])
            async_to_sync(self.channel_layer.group_send)( 
                    key, {"type": "robot_command", msgtype: message}
                )
        
    def robot_command(self, event):
        # this function name is same as type that was sent
        message = event["message"]

        # Send message to WebSocket
        self.send(text_data=json.dumps({"command": message}))

    def telemetry_update(self, event):
        message = event["message"]
        # Send message to WebSocket
        r = Robot.objects.get(robot_name=self.robot_group_name)
        # On its way out to users, we convert the xy to image coordinants
        # do it here since we have access to map data
        message["resolution"] = r.map_id.resolution
        message["origin_x"] = r.map_id.origin_x,
        message["origin_y"] = r.map_id.origin_y
        pixel_x, pixel_y = convert_coordinates_map_to_img(
            message.get("x"),
            message.get("y"),
            r.map_id.resolution,
            r.map_id.origin_x,
            r.map_id.origin_y,
        )
        r, c = Robot.objects.get_or_create(robot_name=self.robot_group_name)
        r.current_x = pixel_x
        r.current_y = pixel_y
        r.current_th = (math.pi - 1.5708) - message["theta"]
        r.save()
        self.send(text_data=json.dumps({"message": message}))
    
    def send_result(self, event):
        message = event["message"]
        # Send message to WebSocket
        self.send(text_data=json.dumps({"message": message}))

##################################################################################################


class RobotConsumer(WebsocketConsumer):
    """
    robot connects to robot_ROBOTNAME
    """

    def connect(self):
        # Obtains the robot_name parameter from the url_route in dispatch/routing.py
        # Every consumer has a scope that contains information about its connection.
        self.robot_name = self.scope["url_route"]["kwargs"]["robot_name"]
        self.robot_group_name = "robot_%s" % self.robot_name
        self.user_group_name = "user_%s" % self.robot_name
        # Join Robot Group
        new_robot, c = Robot.objects.get_or_create(robot_name=self.robot_group_name)
        new_robot.save()
        async_to_sync(self.channel_layer.group_add)(
            self.robot_group_name, self.channel_name
        )
        r = redis.Redis()
        # Robot joins with time connected
        r.hset("layers", self.robot_group_name, time.time())
        self.accept()

    def disconnect(self, close_code):
        # leave robot group
        # gc.del_group(self.robot_group_name)
        Robot.objects.get(robot_name=self.robot_group_name).delete()
        async_to_sync(self.channel_layer.group_discard)(
            self.robot_group_name, self.channel_name
        )
        r = redis.Redis()
        r.hdel("layers", self.robot_group_name)  # removes from redis at end.

    # Receive message from WebSocket
    def receive(
        self,
        text_data=None,
    ):
        text_data_json = json.loads(text_data)
        message = text_data_json["message"]

        # send message to robot group
        # Sends an 'event' to a group
        # event has a special type key which corresponds to the name
        # of the method which should be invoked when consumers receive this event.
        if message["msgtype"] == "TELEMETRY":
            # Telemetry message, send to Robot User Group
            async_to_sync(self.channel_layer.group_send)(
                self.user_group_name, {"type": "telemetry_update", "message": message}
            )
        elif message["msgtype"] == "COMMAND" or message["msgtype"] == "RESPONSE" or message["msgtype"] == "NAVIGATION_RESPONSE":
            # Command message or Response message, Send to destination
            dest = message.get("destination")
            dest = "robot_%s" % dest

            # Check if layer group exists
            dest_count = Robot.objects.filter(robot_name=dest).count()

            if dest_count > 0:
                if message["msgtype"] == "COMMAND":
                    cmd = message.get("command") + " command"
                else:
                    cmd = message.get("result")
                print(
                    "Send %s from %s to %s"
                    % ((cmd), str(message["sender"]), str(message.get("destination")))
                )
                async_to_sync(self.channel_layer.group_send)(
                    dest, {"type": "robot_command", "message": message}
                )
            elif message.get("destination") == "RCS":
                    sender = "robot_%s" % message["sender"]
                    async_to_sync(self.channel_layer.group_send)(
                            self.user_group_name, {"type": "send_result", "message": message}
                        )
                    async_to_sync(self.channel_layer.group_send)(
                    sender,
                    {
                        "type": "robot_command",
                        "message": self.create_error_response(
                            True,
                            self.robot_name,
                            message["command_id"],
                        ),
                    },
                )
            else:
                print("Destination not found in groups.")
                sender = "robot_%s" % message["sender"]
                async_to_sync(self.channel_layer.group_send)(
                    sender,
                    {
                        "type": "robot_command",
                        "message": self.create_error_response(
                            "DESTINATIONNOTFOUND",
                            self.robot_name,
                            message["command_id"],
                        ),
                    },
                )
        elif message["msgtype"] == "INITIAL_CONNECTION":
            r, c = Robot.objects.get_or_create(robot_name=self.robot_group_name)
            m, c = MapData.objects.get_or_create(map_id=message.get("map_id"))
            if c:
                m.save()
            print(m)
            r.map_id = m
            print(f"ROBOT UPDATE: {r}")
            r.save()


    def robot_command(self, event):
        # this function name is same as type that was sent

        msgtype = ""
        message = ""
        if "message" in event:
            message = event["message"]
            msgtype = "command"
        elif "navigation" in event:
            message = event["navigation"]
            message = self.coordinate_conversion(message, False)
            msgtype = "navigation"
        elif "waypoints" in event:
            message = event["waypoints"]
            message = self.coordinate_conversion(message,True)
            msgtype = "waypoints"
        print("sending to", self.robot_group_name)
        # Send message to WebSocket
        self.send(text_data=json.dumps({msgtype: message}))

    # Receive message from robot group
    # this function name is same as type that was sent
    def telemetry_update(self, event):
        message = event["message"]
        
        # Send message to WebSocket
        self.send(text_data=json.dumps({"message": message}))



    def coordinate_conversion(self, message, waypoints_flag):

        r = Robot.objects.get(robot_name=self.robot_group_name)
        if waypoints_flag:
            waypoints = json.loads(message["waypoints"])
            waypoints_list = []
            for waypoint in waypoints:
                real_x, real_y = convert_coordinates_image_to_map(
                    waypoint["x"],
                    waypoint["y"],
                    r.map_id.resolution,
                    r.map_id.origin_x,
                    r.map_id.origin_y,
                )
                waypoint["x"] = real_x
                waypoint["y"] = real_y
                waypoints_list.append(waypoint)
            message["waypoints"] = json.dumps(waypoints_list)
        else:
            real_x, real_y = convert_coordinates_image_to_map(
                message.get("x"),
                message.get("y"),
                r.map_id.resolution,
                r.map_id.origin_x,
                r.map_id.origin_y,
            )
            message["x"] = real_x
            message["y"] = real_y
        return message

    

    def create_error_response(self, error_code, dest, command_id):
        return {
            "sender": "RCS",
            "timestamp": time.time(),
            "msgtype": "RESPONSE",
            "destination": dest,
            "command_id": command_id,
            "result": error_code,
        }
