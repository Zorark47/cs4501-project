#!/usr/bin/env python
import rospy
import tf2_ros
import time
import copy
import math
from geometry_msgs.msg import Vector3, Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class LocalPlanner:

    def __init__(self):
        self.gps = None
        self.lidar = None
        self.lidar_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.get_lidar, queue_size=1)
        self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps, queue_size=1)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.width = rospy.get_param('/environment_controller/map_width')
        self.height = rospy.get_param('/environment_controller/map_height')
        self.grid = OccupancyGrid(data = [0], info = MapMetaData(width=self.width, height=self.height))

        self.mainloop()
        
    def get_gps(self, msg):
        self.gps = msg

    def get_lidar(self, msg):
        self.lidar = msg

    def update_grid(self):
        roll, pitch, yaw = euler_from_quaternion((self.gps.pose.orientation.x, self.gps.pose.orientation.y, self.gps.pose.orientation.z, self.gps.pose.orientation.w))
        for i in range(len(self.lidar.ranges)):
            if self.lidar.ranges[i] > self.lidar.range_max:
                continue
            angle = self.lidar.angle_min + (i * self.lidar.angle_increment) + yaw
            point_x = int(round(((self.lidar.ranges[i] * math.sin(angle) * -1) + self.gps.pose.position.x + (self.width / 2))))
            point_y = int(round(((self.lidar.ranges[i] * math.cos(angle)) + self.gps.pose.position.y + (self.height / 2))))
            if point_x < self.width and point_y < self.height:
                self.grid.data[point_x][point_y] += 5

    def mainloop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.gps != None and self.lidar != None:
                self.update_grid()
                self.map_pub.publish(self.grid)
        rate.sleep()

if __name__ == '__main__':
  rospy.init_node('local_planner')
  try:
    local_planner = LocalPlanner()
  except rospy.ROSInterruptException:
    pass
