import random
import threading
import heapq
import numpy as np
import time
from dataclasses import dataclass
import json

ORIGINAL_MAP_SIZE = [1660, 1660]
NEW_MAP_SIZE = (100, 100)
NUM_WAYPOINTS = 40

@dataclass
class DUMMY_ROBOTS:
    robot_name: str
    current_x: int
    current_y: int
    current_th: int


@dataclass
class Robot:
    name: str
    x: int
    y: int
    th: int

@dataclass
class Waypoint:
    x: int
    y: int
    th: int

class WAYPOINT_DISTRIBUTOR:
    def __init__(self) -> None:
        self.robots = []
        self.waypoints = []

    def heuristic(self, robot, waypoint):
        return abs(robot.x - waypoint.x) + abs(robot.y - waypoint.y)

    def storeRobots(self, robots):
        self.robots = [Robot(name=robot.robot_name, x=robot.current_x, y=robot.current_y, th=robot.current_th) for robot in robots]

    def storeWaypoints(self, waypoints_list):
        self.waypoints = [Waypoint(x=waypoint["x"], y=waypoint["y"], th=waypoint["th"]) for waypoint in waypoints_list]

    def generate_random_name(self):
        """Generate a random string of three uppercase letters."""
        return "".join([self.generate_random_name() for _ in range(3)])

    def genrateRobots(self, num_of_robots):
        robots =[]
        for _ in range(num_of_robots):
            r = Robot(name=self.generate_random_name(), x=random.uniform(0, 100),y=random.uniform(0, 100),th=random.uniform(0, 100))
            robots.append(r)
        return robots

    def genrateWaypoints(self, num_of_waypoints):
        points =[]
        for _ in range(num_of_waypoints):
            r = Waypoint(x=random.uniform(0, 100),y=random.uniform(0, 100),th=random.uniform(0, 100))
            points.append(r)
        return points

    def create_robots(self, max_robots):
        num_robots = max_robots
        robot_positions = [tuple(np.random.randint(0, ORIGINAL_MAP_SIZE[i], num_robots).tolist()) for i in range(2)]
        robot_angles = [random.uniform(0, 2 * np.pi) for _ in range(num_robots)]
        return [DUMMY_ROBOTS(robot_name=f"Robot_{i+1}", current_x=x, current_y=y, current_th=th) for i, (x, y, th) in enumerate(zip(*robot_positions, robot_angles))]

    def create_random_waypoints(self, number_of_waypoints):
        waypoints_list = []
        for _ in range(number_of_waypoints):
            waypoint = np.random.randint(0, ORIGINAL_MAP_SIZE[0], size=2).tolist()
            waypoint = {"x": waypoint[0], "y": waypoint[1], "th": random.uniform(0, 2 * np.pi)}
            waypoints_list.append(waypoint)
        return waypoints_list

    def main(self):
        dummy_robots = self.create_robots(5)
        self.storeRobots(dummy_robots)
        dummy_waypoints = self.create_random_waypoints(100)
        self.storeWaypoints(dummy_waypoints)
        robot_paths = [[] for _ in range(len(self.robots))]
        while len(self.waypoints) > 0:
            for R, r in enumerate(self.robots):
                S = 100000
                closest_waypoint_index = None
                for w, p in enumerate(self.waypoints):
                    distance = self.heuristic(r, p)
                    
                    if distance < S:
                        S = distance
                        closest_waypoint_index = w
                if closest_waypoint_index is not None:
                    closest_waypoint = self.waypoints.pop(closest_waypoint_index)
                    robot_paths[R].append(closest_waypoint)
                    print(f"ROBOT {r.name} move to ({closest_waypoint.x}, {closest_waypoint.y}, {closest_waypoint.th}) from ({r.x}, {r.y}, {r.th})")
                    r.x = closest_waypoint.x
                    r.y = closest_waypoint.y
                    r.th = closest_waypoint.th
        for i, path in enumerate(robot_paths):
            print(f"ROBOT {self.robots[i].name}: {path}")
    
    def startSimulation(self):
        robot_waypoints = {robot.name: [] for robot in self.robots}
        faraway = [False] * len(self.robots)
        while len(self.waypoints) > 0:
            sub = {}
            for R, r in enumerate(self.robots):
                S = 100000
                closest_waypoint_index = None

                for w, p in enumerate(self.waypoints):
                    distance = self.heuristic(r, p)
                    if distance < S:
                        S = distance
                        closest_waypoint_index = w
                if S >=500:
                    faraway[R] = True
                    sub[R] = closest_waypoint_index

                if closest_waypoint_index is not None and faraway[R] != True:
                    closest_waypoint = self.waypoints.pop(closest_waypoint_index)
                    robot_waypoints[r.name].append(closest_waypoint)
                    print(f"ROBOT {r.name} move to ({closest_waypoint.x}, {closest_waypoint.y}, {closest_waypoint.th}) from ({r.x}, {r.y}, {r.th})")
                    r.x = closest_waypoint.x
                    r.y = closest_waypoint.y
                    r.th = closest_waypoint.th

            if all(faraway) and len(self.waypoints) >0:
                print("All elements in faraway are True.")
                nearest_ROBOT = None
                _distance = 1000000
                waypoint_index = None
                for R, r in enumerate(self.robots):
                    distance =  self.heuristic(r, self.waypoints[sub[R]])
                    if distance < _distance :
                        nearest_ROBOT = self.robots[R]
                        _distance = distance
                        waypoint_index = sub[R]
                closest_waypoint = self.waypoints.pop(waypoint_index)
                robot_waypoints[nearest_ROBOT.name].append(closest_waypoint)
                print(f"ROBOT {nearest_ROBOT.name} move to ({closest_waypoint.x}, {closest_waypoint.y}, {closest_waypoint.th}) from ({r.x}, {r.y}, {r.th})")
                nearest_ROBOT.x = closest_waypoint.x
                nearest_ROBOT.y = closest_waypoint.y
                nearest_ROBOT.th = closest_waypoint.th
                faraway = [False] * len(faraway)
            else:
                faraway = [False] * len(faraway)

        return robot_waypoints



    def waypointsFormatConverter(self, waypoints):
        waypoints_list = [{"x": waypoint.x, "y": waypoint.y, "th": waypoint.th} for waypoint in waypoints]
        return json.dumps(waypoints_list)

    def getPath(self):
        return self.startSimulation()

if __name__ == "__main__":
    wd = WAYPOINT_DISTRIBUTOR()
    dummy_robots = wd.create_robots(5)
    wd.storeRobots(dummy_robots)
    dummy_waypoints = wd.create_random_waypoints(100)
    wd.storeWaypoints(dummy_waypoints)
    waypoints = wd.getPath()
    # for key in waypoints.keys():
    #     print(f"ROBOT {key}: \n{waypoints[key]}")