#!/usr/bin/env python 

import rospy
import actionlib
from assignment_2_2024.action import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PositionVelocity
from assignment_2_2024.srv import GetLastTarget, GetLastTargetResponse
from geometry_msgs.msg import Twist
from assignment_2_2024.srv import GoalStatistics, GoalStatisticsResponse

class ActionClientNode:
    def __init__(self):
        rospy.init_node('action_client_node') # initializing the node
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction) # client for the action server
        self.pub_pos_vel = rospy.Publisher('/robot_position_velocity', PositionVelocity, queue_size=10) # publisher for the position and velocity
        
        self.pub_act_vel=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Defining the goal as an attribute of the class 
        self.goal = PlanningGoal() 
        
        # Setting up get_last_target service
        self.service = rospy.Service('/get_last_target', GetLastTarget, self.handle_service) 
        
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Service client to update the last target
        self.target_service_client = rospy.ServiceProxy('/get_last_target', GetLastTarget)

        self.current_odom = None
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server ready!")

        # initializing the goals reached and cancelled
        self.goals_reached = 0
        self.goals_cancelled = 0

        # Add service for goal statistics
        self.stats_service = rospy.Service('/goal_statistics', GoalStatistics, self.handle_stats_service)

    def send_goal(self, x, y):
	
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo(f"Sending goal: ({x}, {y})")
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback, done_cb=self.goal_done_callback)

    def cancel_goal(self):
        rospy.loginfo("Canceling goal...")
        self.client.cancel_goal()
        self.goals_cancelled += 1

    def goal_done_callback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
            self.goals_reached += 1

    def handle_stats_service(self, req):
        return GoalStatisticsResponse(
            goals_reached=self.goals_reached,
            goals_cancelled=self.goals_cancelled
        )

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

            # publishing the robot actual velocity
            vel_msg = Twist()
            vel_msg.linear.x = msg.twist.twist.linear.x * 3.6
            vel_msg.linear.y = msg.twist.twist.linear.y * 3.6
            self.pub_act_vel.publish(vel_msg)


    def handle_service(self,req):
        if self.goal:
            #rospy.loginfo(f"Returning last target: {self.goal.last_target}")
            
            return GetLastTargetResponse(x=self.goal.target_pose.pose.position.x ,y=self.goal.target_pose.pose.position.y )
        else:
            rospy.loginfo("No target set, returning default (0.0, 0.0).")
            return GetLastTargetResponse(x=0.0, y=0.0)               


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

