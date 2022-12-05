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
import mission_planner

class LocalPlanner:

    def __init__(self):
        time.sleep(5)

        self.gps = None
        self.lidar = None
        self.lidar_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.get_lidar, queue_size=1)
        self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps, queue_size=1)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.width = rospy.get_param('/environment_controller/map_width')
        self.height = rospy.get_param('/environment_controller/map_height')
        self.grid = OccupancyGrid(data = [50] * (self.width * self.height))
        self.grid.info.width = self.width
        self.grid.info.height = self.height
        self.grid.info.origin.position.x = float(self.width) / -2.0
        self.grid.info.origin.position.y = float(self.height) / -2.0
        self.opened_doors = []
        self.success = False
        self.mission_planner = mission_planner.MissionPlanner()

        #hardcode
        self.delay = 0
        

        self.o_index_grid = [[0 for x in range(self.width)]for y in range(self.height)]
        num = 0
        for x in range(self.width):
            for y in range(self.height):
                self.o_index_grid[x][y] = num
                num += 1
        
        # self.grid.data[self.o_index_grid[0][0]] = 0                                 #bottom left
        # self.grid.data[self.o_index_grid[self.width - 1][self.height - 1]] = 100    #top right
        # self.grid.data[self.o_index_grid[self.width - 1][0]] = 25                   #bottom right left
        # self.grid.data[self.o_index_grid[0][self.height - 1]] = 75                  #top left

        self.mainloop()
        
    def get_gps(self, msg):
        self.gps = msg
        
        # convert gps to the correct coordinats of the 
        self.gps.pose.position.x += float(self.width) / 2
        self.gps.pose.position.y += float(self.height) / 2

    def get_lidar(self, msg):
        self.lidar = msg

    # def check_door(self):

    def update_grid(self):
        roll, pitch, yaw = euler_from_quaternion((self.gps.pose.orientation.x, self.gps.pose.orientation.y, self.gps.pose.orientation.z, self.gps.pose.orientation.w))

        # hardcoded door
        if self.success == False:
            self.grid.data[self.o_index_grid[6][5]] = -1
        else:
            self.grid.data[self.o_index_grid[6][5]] = -2
            # rospy.loginfo(self.success)

        for i in range(len(self.lidar.ranges)):
            if self.lidar.ranges[i] > self.lidar.range_max:
                continue
            # store the yaw of the drone from (clockwise from top center)
            roll, pitch, yaw = euler_from_quaternion((self.gps.pose.orientation.x, self.gps.pose.orientation.y, self.gps.pose.orientation.z, self.gps.pose.orientation.w))
        
            # lidar scan point (clockwise from top center) NOT FROM DRONE'S FRAME
            angle = self.lidar.angle_min + (i * self.lidar.angle_increment) + yaw

            # set the point of where the lidar is bouncing back froom
            point_x = int(math.ceil((self.lidar.ranges[i] * math.sin(angle)) + self.gps.pose.position.x + .5)) \
                if self.lidar.ranges[i] * math.sin(angle) + self.gps.pose.position.x > (float(self.width) / 2) \
                else int(math.floor((self.lidar.ranges[i] * math.sin(angle)) + self.gps.pose.position.x - .5)) 
            
            point_y = int(math.ceil((self.lidar.ranges[i] * math.cos(angle)) + self.gps.pose.position.y + .5)) \
                if self.lidar.ranges[i] * math.cos(angle) + self.gps.pose.position.y > (float(self.height) / 2) \
                else int(math.floor((self.lidar.ranges[i] * math.cos(angle)) + self.gps.pose.position.y - .5)) 
            
            # door hardcode
            if point_x == 6 and point_y == 5:
                self.delay += 1
                # rospy.loginfo("door detected at 6, 5 from bottom left corner")
                if [point_x, point_y] not in self.opened_doors and self.delay > 15000:
                    self.opened_doors.append([point_x, point_y])
                    self.success = self.mission_planner.use_keyClient(Point(1, 0, 0))
                    rospy.loginfo("door opened")
                continue

            # increase prob at lidar range
            if(point_x < self.width) and (point_x > 0) and (point_y < self.height) and (point_x > 0): #check if valid location
                if(self.grid.data[self.o_index_grid[point_x][point_y]] < 100): # 
                    self.grid.data[self.o_index_grid[point_x][point_y]] += .03
                    continue

            # reduce prob at points between drone and lidar range
            for d in range(int(math.ceil(self.lidar.ranges[i]))):
                inb_point_x = int(math.floor(d * math.sin(angle) + self.gps.pose.position.x))
                inb_point_y = int(math.floor(d * math.cos(angle) + self.gps.pose.position.y))

                if(inb_point_x < self.width) and (inb_point_x > 0) and (inb_point_y < self.height) and (inb_point_x > 0): # check if valid locaiton
                    if(self.grid.data[self.o_index_grid[inb_point_x][inb_point_y]] > 0): # make sure prob is not already 0
                        self.grid.data[self.o_index_grid[inb_point_x][inb_point_y]] -= .10

    def mainloop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            # rospy.loginfo(self.o_index_grid)
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
