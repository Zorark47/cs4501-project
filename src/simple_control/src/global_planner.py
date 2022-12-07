#!/usr/bin/env python
import rospy
import tf2_ros
import time
import copy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, Point, PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf2_geometry_msgs import do_transform_point
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
from astar_class import AStarPlanner
import numpy as np

class GlobalPlanner:

    def __init__(self):
        time.sleep(10)
        self.gps = Vector3()
        self.goal = None
        self.map = None
        self.occ_map = None
        self.at_waypoint = True
        self.got_goal = False
        self.facing = 0
        self.next_point = None
        self.door_open = False
        self.current_point = Vector3()
        self.final_gps = Vector3()
        self.width = rospy.get_param('/environment_controller/map_width')
        self.height = rospy.get_param('/environment_controller/map_height')

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.dog_pos_sub = rospy.Subscriber("/cell_tower/position", Point, self.transform_tower, queue_size=1)
        self.gps_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.get_gps, queue_size=1)
        self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.get_map, queue_size=1)
        self.final_path_pub = rospy.Publisher('uav/final_path', Int32MultiArray, queue_size=1)

        self.goal_pub = rospy.Publisher('/uav/goal', Vector3, queue_size=1)
        self.moving_pub = rospy.Publisher('/uav/moving', Bool, queue_size=1)

        self.doors_sub = rospy.Subscriber('/map/doors', Vector3, self.get_doors, queue_size=1)

        self.list_of_doors = set()
        self.doors_visited = []
        

        self.mainloop()

    def get_doors(self, msg):
        if msg.x > 0:
            self.list_of_doors.add((msg.x-(self.width/2), (msg.y-(self.height/2))*-1))
        else:
            self.list_of_doors.add((msg.x-(self.width/2), msg.y-(self.height/2)))

    def transform_tower(self, msg):
        if not self.goal and not self.got_goal:
            self.goal = msg
        if self.goal and not self.got_goal:
            try: 
                transform = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time())
                point = PointStamped(header=None, point=Point(x=self.goal.x, y=self.goal.y, z=0))
                new_point = do_transform_point(point, transform)
                self.goal = Vector3(x=new_point.point.x+(float(self.width)/2), y=self.height-(new_point.point.y+(float(self.height)/2)), z=0)
                rospy.loginfo(self.goal)
                self.got_goal = True
                self.goal_pub.publish(self.goal)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('tf2 exception, continuing')

        if self.got_goal:
            self.goal_pub.publish(self.goal)

    def get_gps(self, msg):
        self.gps.x = msg.pose.position.x + float(self.width/2)
        self.gps.y = msg.pose.position.y + float(self.height/2)
        self.gps.z = 0
        self.current_point.x = int(round(msg.pose.position.x))
        self.current_point.y = int(round(msg.pose.position.y))
        self.current_point.z = 0
        self.final_gps.x = msg.pose.position.x + float(self.width/2)
        self.final_gps.y = self.height - (msg.pose.position.y + float(self.height/2))

    def get_map(self, msg):
        self.map = np.reshape(msg.data, (self.width, self.height))

    def bug(self):
        next_point = None
        print("following wall")
        # check right
        if self.facing in [-360, 0, 360]: # north
            if self.map[int(round(self.gps.x)) + 1][int(round(self.gps.y))] in [-3, -2, 0]: # check right wall (right)
                next_point = Vector3(x=self.current_point.x+1, y=self.current_point.y, z=0)
                self.facing += 90
            elif self.map[int(round(self.gps.x))][int(round(self.gps.y)) + 1] in [-3, -2, 0]:
                next_point = Vector3(x=self.current_point.x, y=self.current_point.y+1, z=0)
            else:
                self.facing -= 90
                return self.current_point

        elif self.facing in [-270, 90, 450]: # right
            if self.map[int(round(self.gps.x))][int(round(self.gps.y))-1] in [-3, -2, 0]: # check right wall (down) not there
                next_point = Vector3(x=self.current_point.x, y=self.current_point.y-1, z=0)
                self.facing += 90
            elif self.map[int(round(self.gps.x)) + 1][int(round(self.gps.y))] in [-3, -2, 0]: # right wall is there
                next_point = Vector3(x=self.current_point.x + 1, y=self.current_point.y, z=0)
            else:
                self.facing -= 90
                return self.current_point

        elif self.facing in [-180, 180, 540]: # down
            if self.map[int(round(self.gps.x))-1][int(round(self.gps.y))] in [-3, -2, 0]: # check right wall (left) not there
                next_point = Vector3(x=self.current_point.x-1, y=self.current_point.y, z=0)
                self.facing += 90
            elif self.map[int(round(self.gps.x))][int(round(self.gps.y))-1] in [-3, -2, 0]: # right wall is there
                next_point = Vector3(x=self.current_point.x, y=self.current_point.y-1, z=0)
            else:
                self.facing -= 90
                return self.current_point

        elif self.facing in [-90, 270, 630]: # left
            if self.map[int(round(self.gps.x))][int(round(self.gps.y))+1] in [-3, -2, 0]: # check right wall (up) not there
                next_point = Vector3(x=self.current_point.x, y=self.current_point.y+1, z=0)
                self.facing += 90
            elif self.map[int(round(self.gps.x)) - 1][int(round(self.gps.y))] in [-3, -2, 0]: # right wall is there, check left
                next_point = Vector3(x=self.current_point.x - 1, y=self.current_point.y, z=0)
            else:
                self.facing -= 90
                return self.current_point

        # if self.map[int(next_point.x)][int(next_point.y)] not in [-3, -2, 0]:
        #     return self.current_point
        return next_point   

    def go_to_door(self):
        rospy.loginfo("go for door")
        rospy.loginfo(self.list_of_doors)
        rospy.loginfo(self.doors_visited)
        for door in self.list_of_doors:
            if door not in self.doors_visited:
                door_coords = Vector3(x=door[0], y=door[1], z=0)
                if self.map[int(door[0]+(self.width/2))][int((door[1])+self.height/2)] != -2:
                    if door_coords.x < self.current_point.x: # left
                        self.next_point = Vector3(x=door[0]+1, y=door[1], z=0)
                        self.facing = 270
                    elif door_coords.x > self.current_point.x: # right
                        self.next_point = Vector3(x=door[0]-1, y=door[1], z=0)
                        self.facing = 90
                    elif door_coords.y > self.current_point.y: # up
                        self.next_point = Vector3(x=door[0], y=door[1]-1, z=0)
                        self.facing = 0
                    elif door_coords.y < self.current_point.y: #down
                        self.next_point = Vector3(x=door[0], y=door[1]+1, z=0)
                        self.facing = 180
                    self.position_pub.publish(self.next_point)
                    self.moving_pub.publish(True)
                    time.sleep(2)
                elif self.map[int(door[0]+(self.width/2))][int((door[1])+self.height/2)] == -2:
                    if door_coords.x < self.current_point.x: # left
                        self.facing = 270
                    elif door_coords.x > self.current_point.x: # right
                        self.facing = 90
                    elif door_coords.y > self.current_point.y: # up
                        self.facing = 0
                    elif door_coords.y < self.current_point.y: #down
                        self.facing = 180
                    self.doors_visited.append(door)
                    self.next_point = door_coords
                    self.position_pub.publish(self.next_point)
                    self.moving_pub.publish(True)
                    time.sleep(2)
                return

    def final_path(self):
        a_star = AStarPlanner()
        origin = [int(self.width/2), int(self.height/2)]
        goal = [int(self.goal.x), int(self.height - self.goal.y)]
        path = a_star.plan(self.map, origin, goal)
        path_transformed = []
        for point in path:
            path_transformed.append([point[0]-self.width/2, point[1]-self.height/2])
        p_path = Int32MultiArray()
        p_path.data = np.reshape(path_transformed,-1)
        print(path_transformed)
        self.final_path_pub.publish(p_path)
        return

    def mainloop(self):
        rate = rospy.Rate(2)
        self.moving_pub.publish(False)
        self.moving_pub.publish(False)
        time.sleep(1)
        self.moving_pub.publish(False)
        time.sleep(1)
        # While ROS is still running
        first = True
        while not rospy.is_shutdown():
            cur_x = int(self.current_point.x+(self.width/2))
            goal_x = int(self.goal.x)
            cur_y = int(self.height - (self.current_point.y+(self.height/2))-1)
            goal_y = int(self.goal.y)
            # rospy.loginfo("cur x: %d, goal x: %d, cur y: %d, goal y: %d", cur_x, goal_x, cur_y, goal_y)
            if self.current_point == self.next_point:
                self.moving_pub.publish(False)
                time.sleep(5)
            if cur_x == goal_x and cur_y == goal_y:
                print("success!")
                self.final_path()
            elif len(self.list_of_doors) > len(self.doors_visited):
                self.moving_pub.publish(True)
                time.sleep(3)
                self.go_to_door()
            elif (self.current_point == self.next_point) or first:
                self.moving_pub.publish(False)
                time.sleep(4)
                self.next_point = self.bug()
                if self.next_point:
                    self.moving_pub.publish(True)
                    self.position_pub.publish(self.next_point)
                    rospy.loginfo("got next point")
                    time.sleep(3)
            else:
                self.moving_pub.publish(True)
            rospy.loginfo(self.next_point)
            first = False
        rate.sleep()

if __name__ == '__main__':
  rospy.init_node('global_planner')
  try:
    gp = GlobalPlanner()
  except rospy.ROSInterruptException:
    pass
