#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse

class LastTargetService:
    def __init__(self):
        rospy.init_node('last_target_service')
        self.last_target = (0.0, 0.0)

        # Service
        rospy.Service('/get_last_target', Empty, self.get_last_target)

    def set_target(self, x, y):
        self.last_target = (x, y)

    def get_last_target(self, req):
        rospy.loginfo(f"Returning last target: {self.last_target}")
        return EmptyResponse()

if __name__ == '__main__':
    service = LastTargetService()
    rospy.spin()

