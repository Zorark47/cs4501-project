#!/usr/bin/env python
import rospy
import tf2_ros
import time
import copy
import math
from geometry_msgs.msg import Vector3, Point, PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf2_geometry_msgs import do_transform_point
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


        self.mainloop()

    def transform_tower(self, msg):
        if not self.goal and not self.got_goal:
            self.goal = msg
        if self.goal and not self.got_goal:
            try: 
                    #TODO: Lookup the tower to world transform
                transform = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time())

                    #TODO: Convert the goal to a PointStamped
                point = PointStamped(header=None, point=Point(x=self.goal.x, y=self.goal.y, z=0))

                    #TODO: Use the do_transform_point function to convert the point using the transform
                new_point = do_transform_point(point, transform)

                    #TODO: Convert the point back into a vector message containing integers
                self.goal = Vector3(x=new_point.point.x+(float(self.width)/2), y=new_point.point.y+(float(self.height)/2), z=0)
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
        self.current_point.x = int(msg.pose.position.x)
        self.current_point.y = int(msg.pose.position.y)
        self.current_point.z = 0

    def get_map(self, msg):
        self.map = np.reshape(msg.data, (self.width, self.height))

    def bug(self):
        next_point = None
        # check right
        rospy.loginfo(self.current_point)
        if self.map[int(self.gps.x) + 1][int(self.gps.y)] == 0:
            next_point = Vector3(x=self.current_point.x+1, y=self.current_point.y, z=0)
            rospy.loginfo(next_point)
        return next_point   

    def mainloop(self):
        rate = rospy.Rate(2)
        time.sleep(5)
        # While ROS is still running
        while not rospy.is_shutdown():
            next_point = None
            rospy.loginfo(self.current_point)
            if self.at_waypoint:
                next_point = self.bug()
                if next_point:
                    self.position_pub.publish(next_point)
                    self.at_waypoint = False
            if self.current_point == next_point:
                self.at_waypoint = True
            
    #     rate.sleep()

if __name__ == '__main__':
  rospy.init_node('global_planner')
  try:
    gp = GlobalPlanner()
  except rospy.ROSInterruptException:
    pass
