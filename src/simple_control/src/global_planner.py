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
        self.at_waypoint = True
        self.got_goal = False
        self.facing = 0
        self.next_point = None
        self.current_point = Vector3()
        self.next_goal = Vector3(5,8,0)
        self.width = rospy.get_param('/environment_controller/map_width')
        self.height = rospy.get_param('/environment_controller/map_height')

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.dog_pos_sub = rospy.Subscriber("/cell_tower/position", Point, self.transform_tower, queue_size=1)
        self.gps_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.get_gps, queue_size=1)
        self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.get_map, queue_size=1)
        self.path_pub = rospy.Publisher('uav/path', Int32MultiArray, queue_size=1)

        self.goal_pub = rospy.Publisher('/uav/goal', Vector3, queue_size=1)
        self.moving_pub = rospy.Publisher('/uav/moving', Bool, queue_size=1)

        self.doors_sub = rospy.Subscriber('/map/doors', Vector3, self.get_doors, queue_size=1)

        self.list_of_doors = set()
        self.doors_visited = []


        self.mainloop()

    def get_doors(self, msg):
        rospy.loginfo(Vector3(x=msg.x-(float(self.width)/2), y=self.height-(msg.y-(float(self.height)/2)), z=0))
        self.list_of_doors.add((msg.x, msg.y))

    def transform_tower(self, msg):
        if not self.goal and not self.got_goal:
            self.goal = msg
        if self.goal and not self.got_goal:
            try: 
                transform = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time())
                point = PointStamped(header=None, point=Point(x=self.goal.x, y=self.goal.y, z=0))
                new_point = do_transform_point(point, transform)
                self.goal = Vector3(x=new_point.point.x+(float(self.width)/2), y=self.height-(new_point.point.y+(float(self.height)/2)), z=0)
                self.got_goal = True
                rospy.loginfo(self.goal)
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

    def get_map(self, msg):
        self.map = np.reshape(msg.data, (self.width, self.height))

    def bug(self):
        next_point = None
        # check right
        if self.facing == 0 or self.facing == -360: # north
            if self.map[int(round(self.gps.x)) + 1][int(round(self.gps.y))] in [-2, 0]: # check right wall (right)
                next_point = Vector3(x=self.current_point.x+1, y=self.current_point.y, z=0)
                self.facing += 90
            elif self.map[int(round(self.gps.x))][int(round(self.gps.y)) + 1] in [-2, 0]:
                next_point = Vector3(x=self.current_point.x, y=self.current_point.y+1, z=0)
            else:
                self.facing -= 90
                return self.current_point

        elif self.facing == 90 or self.facing == -270: # right
            if self.map[int(round(self.gps.x))][int(round(self.gps.y))-1] in [-2, 0]: # check right wall (down) not there
                next_point = Vector3(x=self.current_point.x, y=self.current_point.y-1, z=0)
                self.facing += 90
            elif self.map[int(round(self.gps.x)) + 1][int(round(self.gps.y))] in [-2, 0]: # right wall is there
                next_point = Vector3(x=self.current_point.x + 1, y=self.current_point.y, z=0)
            else:
                self.facing -= 90
                return self.current_point

        elif self.facing == 180 or self.facing == -180: # down
            if self.map[int(round(self.gps.x))-1][int(round(self.gps.y))] in [-2, 0]: # check right wall (left) not there
                next_point = Vector3(x=self.current_point.x-1, y=self.current_point.y, z=0)
                self.facing += 90
            elif self.map[int(round(self.gps.x))][int(round(self.gps.y))-1] in [-2, 0]: # right wall is there
                next_point = Vector3(x=self.current_point.x, y=self.current_point.y-1, z=0)
            else:
                self.facing -= 90
                return self.current_point

        elif self.facing == 270 or self.facing == -90: # left
            if self.map[int(round(self.gps.x))][int(round(self.gps.y))+1] in [-2, 0]: # check right wall (up) not there
                next_point = Vector3(x=self.current_point.x, y=self.current_point.y+1, z=0)
                self.facing += 90
            elif self.map[int(round(self.gps.x)) - 1][int(round(self.gps.y))] in [-2, 0]: # right wall is there
                next_point = Vector3(x=self.current_point.x - 1, y=self.current_point.y, z=0)
            else:
                self.facing -= 90
                return self.current_point

        return next_point   

    def go_to_door(self):
        for door in self.list_of_doors:
            if door not in self.doors_visited:
                self.doors_visited.append(door)
                rospy.loginfo(Vector3(x=door[0]-(float(self.width)/2), y=self.height-(door[1]-(float(self.height)/2)), z=0))
                self.next_point = Vector3(x=door[0]-(self.width/2), y=door[1]-(self.height/2))
                self.position_pub.publish(self.next_point)

    def mainloop(self):
        rate = rospy.Rate(2)
        self.moving_pub.publish(False)
        time.sleep(25)
        self.moving_pub.publish(False)
        time.sleep(1)
        self.moving_pub.publish(False)
        time.sleep(1)
        # While ROS is still running
        first = True
        while not rospy.is_shutdown():
            if self.list_of_doors:
                self.go_to_door()
            if (self.current_point == self.next_point) or first:
                time.sleep(3)
                self.moving_pub.publish(False)
                time.sleep(3)
                self.next_point = self.bug()
                if self.next_point:
                    self.moving_pub.publish(True)
                    time.sleep(1)
                    self.position_pub.publish(self.next_point)
                    rospy.loginfo("got next point")
            else:
                self.moving_pub.publish(True)
            

            first = False
            rate.sleep()

if __name__ == '__main__':
  rospy.init_node('global_planner')
  try:
    gp = GlobalPlanner()
  except rospy.ROSInterruptException:
    pass
