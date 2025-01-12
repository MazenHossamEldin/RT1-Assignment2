#!/usr/bin/env python 

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PositionVelocity
from assignment_2_2024.srv import GetLastTarget

class ActionClientNode:
    def __init__(self):
        rospy.init_node('action_client_node')
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.pub_pos_vel = rospy.Publisher('/robot_position_velocity', PositionVelocity, queue_size=10)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Service client to update the last target
        self.target_service_client = rospy.ServiceProxy('/get_last_target', GetLastTarget)

        self.current_odom = None
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server ready!")

    def send_goal(self, x, y):
        goal = PlanningGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo(f"Sending goal: ({x}, {y})")
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

    def cancel_goal(self):
        rospy.loginfo("Canceling goal...")
        self.client.cancel_goal()

    def feedback_callback(self, feedback):
        rospy.loginfo(f"Feedback received: {feedback}")

    def odom_callback(self, msg):
        self.current_odom = msg
        if msg:
            pos_vel_msg = PositionVelocity()
            pos_vel_msg.x = msg.pose.pose.position.x
            pos_vel_msg.y = msg.pose.pose.position.y
            pos_vel_msg.vel_x = msg.twist.twist.linear.x
            pos_vel_msg.vel_z = msg.twist.twist.angular.z
            self.pub_pos_vel.publish(pos_vel_msg)

    def run(self):
        rospy.loginfo("Action Client Node running...")
        while not rospy.is_shutdown():
            cmd = input("Enter 's' to send a goal, 'c' to cancel, or 'q' to quit: ")
            if cmd == 's':
                x = float(input("Enter target x: "))
                y = float(input("Enter target y: "))
                self.send_goal(x, y)
            elif cmd == 'c':
                self.cancel_goal()
            elif cmd == 'q':
                break

if __name__ == "__main__":
    node = ActionClientNode()
    node.run()
    rospy.spin()