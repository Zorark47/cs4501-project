#!/usr/bin/env python
import rospy
import tf2_ros
import time
import copy
import math
from geometry_msgs.msg import Vector3, Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

class LocalPlanner:

    def __init__(self):
        self.grid = OccupancyGrid()
        self.lidar_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.get_lidar, queue_size=1)
        