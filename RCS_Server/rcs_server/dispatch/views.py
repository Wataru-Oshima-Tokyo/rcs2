from django.shortcuts import render, redirect
import redis
import time
from dispatch.models import Robot, MapData

# Create your views here.


def index(request):
    r = redis.Redis()
    active_maps = MapData.objects.all()
    active_locales = dict((n.map_id, 0) for n in active_maps)
    print(active_locales)
    online_robots = {}
    current_time = time.time()
    for key, val in r.hgetall("layers").items():
        online_robots.update(
            {key.decode()[6:]: round((current_time - float(val.decode())) / 60, 2)}
        )
    return render(
        request,
        "dispatch/index.html",
        {"online_robots": online_robots, "active_locales": active_locales},
    )


def robot(request, robot_name):
    # Pass robot name to page
    r = Robot.objects.get(robot_name=f"robot_{robot_name}")
    locale_name = r.map_id.map_id
    return render(
        request,
        "dispatch/locale.html",
        {
            "robot_names": [robot_name],
            "locale_name": locale_name,
            "single_bot": True,
        },
    )


def locale(request, locale_name):
    # pass in all active robots for this map_id
    m = MapData.objects.get(map_id=locale_name)
    active_robots_in_map = Robot.objects.filter(map_id=m)
    # Trim the robot group name "robot_" from the name before passing to user
    active_robots_in_map = [r.robot_name[6:] for r in active_robots_in_map]
    return render(
        request,
        "dispatch/locale.html",
        {"locale_name": locale_name, "robot_names": active_robots_in_map},
    )
