#!/usr/bin/env python
import rospy
import tf2_ros
import time
import copy
import math
from geometry_msgs.msg import Vector3, Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Quaternion
from tf2_geometry_msgs import do_transform_point

class GlobalPlanner:

    def __init__(self):
        self.goal = None

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.dog_pos_sub = rospy.Subcriber("/cell_tower/position", PoseStamped, self.get_pos, queue_size=1)

        self.mainloop()

    def get_pos(self, msg):
        self.goal = msg

    def mainloop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.goal:
                try: 
                    #TODO: Lookup the tower to world transform
                    transform = self.tfBuffer.lookup_transform('world', 'tower', rospy.Time())

                    #TODO: Convert the goal to a PointStamped
                    point = PointStamped(header=None, point=Point(x=self.goal.x, y=self.goal.y, z=self.goal.z))

                    #TODO: Use the do_transform_point function to convert the point using the transform
                    new_point = do_transform_point(point, transform)

                    #TODO: Convert the point back into a vector message containing integers
                    point = Vector3(x=new_point.point.x, y=new_point.point.y, z=new_point.point.z)

                    #TODO: Publish the vector
                    rospy.loginfo(str(rospy.get_name()) + ": Publishing Transformed Goal {}".format([point.x, point.y]))
                    self.pub.publish(point)

                    # The tower will automatically send you a new goal once the drone reaches the requested position.
                    #TODO: Reset the goal
                    self.goal = None

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print('tf2 exception, continuing')
                    continue
            
        rate.sleep()

if __name__ == '__main__':
  rospy.init_node('tower_to_map')
  try:
    tom = TowerToMap()
  except rospy.ROSInterruptException:
    pass
