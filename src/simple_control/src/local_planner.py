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
        self.gps = None
        self.lidar = None
        self.goal = None
        self.lidar_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.get_lidar, queue_size=1)
        self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps, queue_size=1)
        self.goal_sub = rospy.Subscriber('/uav/goal', Vector3, self.get_goal, queue_size=1)
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
        #self.prev_scan = [0 for x in range(self.lidar.ranges)]

        #hardcode
        self.delay = 0

        # find door globals
        self.door_count = 0
        self.has_run = False
        self.prev_scan = [0 for x in range(16)]
        self.list_of_doors = [(-1, -1) for x in range(3)]
        
        self.o_index_grid = [[0 for x in range(self.width)]for y in range(self.height)]
        self.o_confirm_grid = [[0 for x in range(self.width)]for y in range(self.height)]

        num = 0
        for x in range(self.width):
            for y in range(self.height):
                self.o_index_grid[x][(self.height - y) - 1] = num
                num += 1
        
        self.grid.data[self.o_index_grid[0][0]] = 0                                 #bottom left
        # self.grid.data[self.o_index_grid[self.width - 1][self.height - 1]] = 100    #top right
        # self.grid.data[self.o_index_grid[self.width - 1][0]] = 25                   #bottom right left
        # self.grid.data[self.o_index_grid[0][self.height - 1]] = 75                  #top left

        self.mainloop()

    def get_goal(self, msg):
        self.goal = msg
        self.grid.data[self.o_index_grid[int(self.goal.x)][int(self.goal.y)]] = -3
        
    def get_gps(self, msg):
        self.gps = msg
        
        # convert gps to the correct coordinats of the 
        self.gps.pose.position.x += ((float(self.width) / 2) - 0.5)
        self.gps.pose.position.y += ((float(self.height) / 2) - 0.5)
        #print("x: " + str(self.gps.pose.position.x))
        #print("y: " + str(self.gps.pose.position.y))

    def get_lidar(self, msg):
        self.lidar = msg
    
    def setDoor(self, i, x, y):
        min_noise = 0.1
        if self.has_run and (self.door_count < 3) and  (abs(self.prev_scan[i] - self.lidar.ranges[i]) > min_noise):
            # check the door hasn't been set and there isn't already 3 doors
            if (x, y) not in self.list_of_doors and self.door_count < 3:
                # check that there is no door directly next to it
                if ((x + 1, y) not in self.list_of_doors) and ((x - 1, y) not in self.list_of_doors) and ((x, y + 1) not in self.list_of_doors) and ((x - 1, y) not in self.list_of_doors):
                    print("setting door at: (" + str(x) + ", " + str(y) + ") index: " + str(i))
                    self.grid.data[self.o_index_grid[x][y]] = -1
                    self.list_of_doors[self.door_count] = (x, y)
                    self.door_count += 1


        self.prev_scan[i] = self.lidar.ranges[i]
        if i == 15:
            self.has_run = True



    # def check_door(self):

    def update_grid(self):
        roll, pitch, yaw = euler_from_quaternion((self.gps.pose.orientation.x, self.gps.pose.orientation.y, self.gps.pose.orientation.z, self.gps.pose.orientation.w))
        

        for i in range(len(self.lidar.ranges)):
            distance = self.lidar.ranges[i]
            if (self.lidar.ranges[i] >= self.lidar.range_max):
                distance = 6
            else:
                distance = self.lidar.ranges[i]
                #print("max range at: " + str(i))
                
            # store the yaw of the drone from (clockwise from top center)
            roll, pitch, yaw = euler_from_quaternion((self.gps.pose.orientation.x, self.gps.pose.orientation.y, self.gps.pose.orientation.z, self.gps.pose.orientation.w))
            #yaw = yaw % (2 * math.pi)#((-1 * yaw) + (2 * math.pi)) % (2 * math.pi)
            # lidar scan point (clockwise from top center) NOT FROM DRONE'S FRAME
            angle = ((0.448798954487) + (i * self.lidar.angle_increment) + (( math.pi) / 2)) % (2 * math.pi)

            # set the point of where the lidar is bouncing back froom
            point_x = ((distance) * math.sin(angle)) + self.gps.pose.position.x
            point_y = ((distance) * math.cos(angle)) + self.gps.pose.position.y
            

                

            # starting the new better code here
            # assuming I have a very good lidar
            mid =.3

            if(angle > 5.49779) or (angle <= 0.785398):
                # upper 90
                if (angle < (mid / 2)) or (angle > ((2 * math.pi) - (mid / 2))):
                    #print("upper" + str(i))
                    # middle
                    o_point_x = int(round(point_x))
                    e_point_x = int(round(point_x))
                    self.setDoor(i, o_point_x, int(math.ceil(point_y)))
                elif(angle >= 0) and (angle <= 0.785398):
                    # right
                    o_point_x = int(math.ceil(point_x))
                    e_point_x = int(math.floor(point_x))
                else:
                    # left
                    o_point_x = int(math.floor(point_x))
                    e_point_x = int(math.ceil(point_x))
                o_point_y = int(math.ceil(point_y))
                e_point_y = int(math.floor(point_y))
            elif(angle > 0.785398) and (angle <= 2.35619):
                # right 90
                if abs(angle - (math.pi / 2)) < mid:
                    #print("right " + str(i))
                    # middle
                    o_point_y = int(round(point_y))
                    e_point_y = int(round(point_y))
                    self.setDoor(i, int(math.ceil(point_x)), o_point_y)
                elif angle >= math.pi / 2:
                    # down
                    o_point_y = int(math.floor(point_y))
                    e_point_y = int(math.ceil(point_y))
                else:
                    # up
                    o_point_y = int(math.ceil(point_y))
                    e_point_y = int(math.floor(point_y))
                o_point_x = int(math.ceil(point_x))
                e_point_x = int(math.floor(point_x))
            elif(angle > 2.35619) and (angle <= 3.92699):
                # bottom 90
                if abs(angle - math.pi) < mid:
                    #print("bottom " + str(i))

                    # middle
                    o_point_x = int(round(point_x))
                    e_point_x = int(round(point_x))
                    self.setDoor(i, o_point_x, int(math.floor(point_y)))
                elif angle >= math.pi:
                    # left
                    o_point_x = int(math.floor(point_x))
                    e_point_x = int(math.ceil(point_x))
                else:
                    # right
                    o_point_x = int(math.ceil(point_x))
                    e_point_x = int(math.floor(point_x))
                o_point_y = int(math.floor(point_y))
                e_point_y = int(math.ceil(point_y))
            elif(angle > 3.92699) and (angle <= 5.49779):
                # left 90
                if abs(angle - 4.71239) < mid:
                    #print("left " + str(i))
                    # middle 
                    o_point_y = int(round(point_y))
                    e_point_y = int(round(point_y))
                    self.setDoor(i, int(math.floor(point_x)), o_point_y)
                elif angle >= (3 * math.pi) / 2:
                    # up 
                    o_point_y = int(math.ceil(point_y))
                    e_point_y = int(math.floor(point_y))
                else:
                    # down
                    o_point_y = int(math.floor(point_y))
                    e_point_y = int(math.ceil(point_y))
                o_point_x = int(math.floor(point_x))
                e_point_x = int(math.ceil(point_x))
                
            # if i == 75: #angle > 5.8 or angle < 0.2:
            #     print("angle: " + str(angle))
            #     print("lidar index: " + str(i))
            #     print("distance : " + str(self.lidar.ranges[i]))
            #     print("x: " + str(point_x))
            #     print("y: " + str(point_y))
            #     print("int  ensi: " + str(self.lidar.intensities[i]))
            #     print("intensit7: " + str(self.lidar.intensities[11]))
            #     print("x: " + str(o_point_x))
            #     print("y: " + str(o_point_y))

            # increase obstance point
            inc = .3
            if(o_point_x >= 0) and (o_point_x < self.width) and (o_point_y >= 0) and (o_point_y < self.height):
                if (distance < self.lidar.range_max) and (self.grid.data[self.o_index_grid[o_point_x][o_point_y]] <= (100 - inc)) and (self.grid.data[self.o_index_grid[o_point_x][o_point_y]] > 0):
                    self.grid.data[self.o_index_grid[o_point_x][o_point_y]] += inc
                elif (self.grid.data[self.o_index_grid[o_point_x][o_point_y]] > (100 -inc)) and (self.o_confirm_grid[o_point_x][o_point_y] == 0) and (distance < 1):
                    self.o_confirm_grid[o_point_x][o_point_y] = 1

            
            # decrease furthest empty point
            dec = .6
            if(distance > 1) and (e_point_x >= 0) and (e_point_x < self.width) and (e_point_y >= 0) and (e_point_y < self.height):
                if self.grid.data[self.o_index_grid[e_point_x][e_point_y]] >= dec and (self.o_confirm_grid[e_point_x][e_point_y] == 0): #and self.grid.data[self.o_index_grid[e_point_x][e_point_y]] < 99:
                    self.grid.data[self.o_index_grid[e_point_x][e_point_y]] -= dec
            
            # decrease in between points
            step = 0.3
            distance -= step
            while distance > 0:
                if(int(round((distance * math.sin(angle)) + self.gps.pose.position.x)) != e_point_x) or (int(round((distance * math.cos(angle)) + self.gps.pose.position.y)) != e_point_y):
                    e_point_x = int(round((distance * math.sin(angle)) + self.gps.pose.position.x))
                    e_point_y = int(round((distance * math.cos(angle)) + self.gps.pose.position.y))
                    if (e_point_x >= 0) and (e_point_x < self.width) and (e_point_y >= 0) and (e_point_y < self.height):
                        if self.grid.data[self.o_index_grid[e_point_x][e_point_y]] >= dec and (self.o_confirm_grid[e_point_x][e_point_y] == 0):#self.grid.data[self.o_index_grid[e_point_x][e_point_y]] < 99:
                            self.grid.data[self.o_index_grid[e_point_x][e_point_y]] -= dec
                distance -= step                        

            # # increase prob at lidar range                for i in range(len(self.lidar.ranges)):
            #         angle = ((self.lidar.angle_min) + (i * self.lidar.angle_increment)) % (2 * math.pi)
            #         print("lidar angles i: " + str(i) + "angle: " + str(angle))oint_x][point_y]] < 100): # 
            #         self.grid.data[self.o_index_grid[point_x][point_y]] += .05
            #         continue

            # # reduce prob at points between drone and lidar range
            # for d in range(int(math.ceil(self.lidar.ranges[i]))):
            #     point_x = int(round(d * math.sin(angle)) + self.gps.pose.position.x)
            #     point_y = int(round(d * math.cos(angle)) + self.gps.pose.position.y)

            #     if(point_x < self.width) and (point_x > 0) and (point_y < self.height) and (point_x > 0): # check if valid locaiton
            #         if(self.grid.data[self.o_index_grid[point_x][point_y]] > 0): # make sure prob is not already 0
            #             self.grid.data[self.o_index_grid[point_x][point_y]] -= .10
            
        

    def mainloop(self):
        rate = rospy.Rate(2)


        while not rospy.is_shutdown():
            # rospy.loginfo(self.o_index_grid)
            if self.gps != None and self.lidar != None:
                
                self.update_grid()
                
                #self.setDoor()

                self.map_pub.publish(self.grid)
        rate.sleep()

if __name__ == '__main__':
  rospy.init_node('local_planner')
  try:
    local_planner = LocalPlanner()
  except rospy.ROSInterruptException:
    pass
