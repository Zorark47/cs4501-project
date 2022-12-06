#!/usr/bin/env python
import rospy
from environment_controller.srv import use_key, use_keyResponse

class MissionPlanner:
    def __init__(self):
        self.keys = 4
        # self.service = rospy.Service('use_key', use_key, self.use_key)

    def use_keyClient(self, point):
        rospy.wait_for_service('use_key')
        try:
            if self.keys > 0:
                use_key_ = rospy.ServiceProxy('use_key', use_key)
                succ = use_key_(point)
                self.keys -= 1
                rospy.loginfo(self.keys)
                return succ
            else: 
                return False
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)