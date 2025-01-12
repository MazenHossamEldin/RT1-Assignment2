#!/usr/bin/env python
import rospy
from assignment_2_2024.srv import GetLastTarget, GetLastTargetResponse


class TargetServiceNode:
    def __init__(self):
        rospy.init_node('target_service_node')
        self.last_target = None
        self.service = rospy.Service('/get_last_target', GetLastTarget, self.handle_service)
        rospy.loginfo("Target Service is ready.")

    def set_target(self, x, y):
        self.last_target = (x, y)
        rospy.loginfo(f"Target updated to: {self.last_target}")

    def handle_service(self, req):
        if self.last_target:
            rospy.loginfo(f"Returning last target: {self.last_target}")
            return GetLastTargetResponse(x=self.last_target[0], y=self.last_target[1])
        else:
            rospy.loginfo("No target set, returning default (0.0, 0.0).")
            return GetLastTargetResponse(x=0.0, y=0.0)

if __name__ == "__main__":
    TargetServiceNode()

    rospy.spin()