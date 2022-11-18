#!/usr/bin/env python
import rospy
import tf2_ros
import time
import copy
import math
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Quaternion
from tf2_geometry_msgs import do_transform_point

class GlobalPlanner:

    def __init__(self):
        self.dog_pos_sub = rospy.Subcriber("/cell_tower/position", PoseStamped, self.get_pos, queue_size=1)
        self.goal = 0
        self.transform_stamped = TransformStamped()

    def get_pos(self, msg):
        self.goal = msg