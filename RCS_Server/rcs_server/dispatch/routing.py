from django.urls import re_path

# from dispatch.Robot.RobotConsumer import RobotConsumer
from dispatch import consumers

websocket_urlpatterns = [
    re_path(
        r"ws/dispatch/robot/(?P<robot_name>\w+)/$", consumers.RobotConsumer.as_asgi()
    ),
    re_path(
        r"ws/dispatch/user/(?P<robot_name>\w+)/$", consumers.UserConsumer.as_asgi()
    ),
]
