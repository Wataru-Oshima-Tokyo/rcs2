from django.db import models
from django.urls import reverse
import yaml
from os.path import join
import os

class MapData(models.Model):
    map_id = models.CharField(max_length=30)
    origin_x = models.FloatField(default=0.0)
    origin_y = models.FloatField(default=0.0)
    resolution = models.FloatField(default=0.0)

    def save(self, *args, **kwargs):
        # Creating a new map, check yamls to see if we have this here?
        yaml_file = join("./static", "mapyamls", f"{self.map_id}.yaml")
        if os.path.exists(yaml_file):
            with open(yaml_file, mode='r', encoding="utf-8") as file:
                yaml_dat = yaml.safe_load(file)
            self.resolution = float(yaml_dat["resolution"])
            self.origin_x = float(yaml_dat["origin"][0])
            self.origin_y = float(yaml_dat["origin"][1])
        super(MapData, self).save(*args, **kwargs)

    def __str__(self):
        return f"{self.map_id} : ({self.origin_x}, {self.origin_y})->{self.resolution}"


class Robot(models.Model):
    robot_name = models.CharField(max_length=30)
    map_id = models.ForeignKey(MapData, on_delete=models.SET_NULL, null=True)
    current_x = models.FloatField(default=0.0)
    current_y = models.FloatField(default=0.0)
    current_th = models.FloatField(default=0.0)
    def __str__(self):
        return "%s @ %s" % (self.robot_name, self.map_id)
