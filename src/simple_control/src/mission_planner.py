#!/usr/bin/env python
import rospy
from environment_controller.srv import use_key, use_keyResponse

class MissionPlanner:
    def __init__(self):
        self.opened_doors = 0
        self.service = rospy.Service('use_key', use_key, self.openDoor)

        self.mainloop()

    def openDoor(self, request):
        if request.success:
            self.opened_doors += 1
        return use_keyResponse(request.door_loc)

    def mainloop():
        rate = rospy.Rate(2)
        # while not rospy.is_shutdown():
            
        rate.sleep()