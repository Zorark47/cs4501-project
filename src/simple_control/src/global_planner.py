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
        self.gps = None
        self.goal = None
        self.map = None
        self.next_goal = Point(0,3,0)
        self.width = rospy.get_param('/environment_controller/map_width')
        self.height = rospy.get_param('/environment_controller/map_height')

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.dog_pos_sub = rospy.Subscriber("/cell_tower/position", Point, self.transform_tower, queue_size=1)
        self.gps_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.get_gps, queue_size=1)
        self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.get_map, queue_size=1)
        self.path_pub = rospy.Publisher('uav/path', Path, queue_size=1)

        self.mainloop()

    def transform_tower(self, msg):
        self.goal = msg

    def get_gps(self, msg):
        self.gps = msg

    def get_map(self, msg):
        self.map = np.reshape(msg.data, (self.width, self.height))

    def mainloop(self):
        rate = rospy.Rate(2)
        self.have_plan = False
        sent_position = False

        # Create the path publish message
        p_path = Int32MultiArray()

        # While ROS is still running
        while not rospy.is_shutdown():

        # If you dont have a plan wait for a map, current position, and a goal
            if not self.have_plan:
                # If we have received the data
                time.sleep(15)
                rospy.loginfo(str(rospy.get_name()) + ": Planning path")
                astar = AStarPlanner(safe_distance=0)
                path = astar.plan(self.map, [int(self.gps.pose.position.x), int(self.gps.pose.position.y)], [int(self.next_goal.x), int(self.next_goal.y)])
                if path != None:
                    path = np.array(path)
                    self.have_plan = True
                    # path[:, 0] = path[:, 0] + self.origin_x
                    # path[:, 1] = path[:, 1] + self.origin_y
                    rospy.loginfo(str(rospy.get_name()) + ": Executing path")   
                    rospy.loginfo(str(rospy.get_name()) + ": " + str(path)) 
                else:
                    rospy.loginfo(str(rospy.get_name()) + ": path not found, try another goal")  
            # We have a plan, execute it
            else:    

                # Publish the path
                if len(p_path.data) != len(np.reshape(path,-1)):
                    p_path.data = np.reshape(path,-1)
                    p_path = Path(p_path)
                    self.path_pub.publish(p_path)

                    # Publish the current waypoint
                    if sent_position == False or np.shape(path)[0] < 0:
                        msg = Vector3()
                        msg.x = path[0][0]
                        msg.y = path[0][1]
                        msg.z = 0
                        self.position_pub.publish(msg)
                        sent_position = True
                else:
                    path = path[1:]
                    sent_position = False

                # If we are done wait for next goal
                if np.shape(path)[0] <= 0:
                    self.have_plan = False
                    self.gps = copy.deepcopy(self.goal_position)
                    self.goal_position = []
                    continue

            # if not self.goal:
            #     try: 
            #         #TODO: Lookup the tower to world transform
            #         transform = self.tfBuffer.lookup_transform('world', 'cell_tower', rospy.Time())

            #         #TODO: Convert the goal to a PointStamped
            #         point = PointStamped(header=None, point=Point(x=self.goal.x, y=self.goal.y, z=self.goal.z))

            #         #TODO: Use the do_transform_point function to convert the point using the transform
            #         new_point = do_transform_point(point, transform)

            #         #TODO: Convert the point back into a vector message containing integers
            #         self.goal = Vector3(x=new_point.point.x, y=new_point.point.y, z=new_point.point.z)
            #         rospy.loginfo(self.goal)

            #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #         print('tf2 exception, continuing')
            #         continue
            
        rate.sleep()

if __name__ == '__main__':
  rospy.init_node('tower_to_map')
  try:
    tom = GlobalPlanner()
  except rospy.ROSInterruptException:
    pass
