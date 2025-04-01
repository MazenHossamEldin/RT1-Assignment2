#!/usr/bin/env python
"""
.. module:: action_client
   :platform: Unix
   :synopsis: ROS node acting as an action client to send planning goals and publish robot's position and velocity.

.. moduleauthor:: Mazen Madbouly <mazen.madbouly01@gmail.com>
"""
import rospy
import actionlib
from assignment_2_2024.action import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PositionVelocity
from assignment_2_2024.srv import GetLastTarget, GetLastTargetResponse

class ActionClientNode:
    """
    .. class:: ActionClientNode

    :brief: Encapsulates a ROS node that acts as an action client.
    
    This class initializes a ROS node that communicates with a planning action server to send goals, 
    receives feedback, and publishes the robot's position and velocity. Additionally, it sets up a service 
    to return the last goal target.
    
    **ROS Topics:**
    
      - Publishes to: ``/robot_position_velocity`` (PositionVelocity)
      - Subscribes to: ``/odom`` (Odometry)
    
    **ROS Services:**
    
      - Advertises: ``/get_last_target`` (GetLastTarget)
    
    **ROS Action Server:**
    
      - Uses action server: ``/reaching_goal`` (PlanningAction)
    """
    
    def __init__(self):
        """
        Initializes the ActionClientNode.

        Sets up the ROS node, action client, publisher, subscriber, and service. Also waits for the action server
        to become available.
        """
        rospy.init_node('action_client_node')  # Initializing the node
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)  # Client for the action server
        self.pub_pos_vel = rospy.Publisher('/robot_position_velocity', PositionVelocity, queue_size=10)  # Publisher for position and velocity
        self.goal = PlanningGoal()  # Defining the goal as an attribute of the class 
        
        # Setting up the get_last_target service
        self.service = rospy.Service('/get_last_target', GetLastTarget, self.handle_service)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Service client to update the last target
        self.target_service_client = rospy.ServiceProxy('/get_last_target', GetLastTarget)
        
        self.current_odom = None
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server ready!")

    def send_goal(self, x, y):
        """
        Sends a goal to the action server.

        Updates the goal with the specified x and y coordinates, stamps it with the current time,
        and sends it to the action server. The goal is defined in the "map" coordinate frame.

        :param x: Target x-coordinate (float)
        :param y: Target y-coordinate (float)
        """
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo(f"Sending goal: ({x}, {y})")
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def cancel_goal(self):
        """
        Cancels the currently active goal.

        Logs the cancellation and sends a cancel request to the action server.
        """
        rospy.loginfo("Canceling goal...")
        self.client.cancel_goal()

    def feedback_callback(self, feedback):
        """
        Callback function to handle feedback from the action server.

        Logs the feedback received.

        :param feedback: Feedback message from the action server.
        """
        rospy.loginfo(f"Feedback received: {feedback}")

    def odom_callback(self, msg):
        """
        Callback function for odometry data.

        Receives odometry messages, extracts position and velocity information, and publishes it
        as a PositionVelocity message.

        :param msg: Odometry message received from the ``/odom`` topic.
        """
        self.current_odom = msg
        if msg:
            pos_vel_msg = PositionVelocity()
            pos_vel_msg.x = msg.pose.pose.position.x
            pos_vel_msg.y = msg.pose.pose.position.y
            pos_vel_msg.vel_x = msg.twist.twist.linear.x
            pos_vel_msg.vel_z = msg.twist.twist.angular.z
            self.pub_pos_vel.publish(pos_vel_msg)

    def handle_service(self, req):
        """
        Service handler to return the last target goal.

        If a goal has been set, returns its x and y coordinates. Otherwise, returns default values (0.0, 0.0).

        :param req: Service request (unused).
        :return: GetLastTargetResponse with x and y coordinates of the last target.
        """
        if self.goal:
            return GetLastTargetResponse(x=self.goal.target_pose.pose.position.x,
                                         y=self.goal.target_pose.pose.position.y)
        else:
            rospy.loginfo("No target set, returning default (0.0, 0.0).")
            return GetLastTargetResponse(x=0.0, y=0.0)

    def run(self):
        """
        Runs the ActionClientNode.

        Enters a continuous loop that waits for user input from the console to control the robot's behavior.
        The loop continues until ROS is shutdown or the user chooses to quit.
        
        Command options:
          - 's': Prompts the user to enter x and y coordinates for a new target position,
                 then calls send_goal() to send this target to the action server.
          - 'c': Calls cancel_goal() to abort the current navigation goal if one is active.
          - 'q': Breaks out of the loop, effectively terminating the node's main function.
        
        The function uses ROS's is_shutdown() check to ensure proper termination when ROS
        signals are received.
        """
        rospy.loginfo("Action Client Node running...")
        while not rospy.is_shutdown():
            cmd = input("Enter 's' to send a goal, 'c' to cancel, or 'q' to quit: ")
            if cmd == 's':
                x = float(input("Enter target x: "))  # Convert user input to float for x coordinate
                y = float(input("Enter target y: "))  # Convert user input to float for y coordinate
                self.send_goal(x, y)  # Call method to send the goal to the action server
            elif cmd == 'c':
                self.cancel_goal()  # Call method to cancel the current goal
            elif cmd == 'q':
                break  # Exit the loop and terminate the run method

if __name__ == "__main__":
    node = ActionClientNode()
    node.run()
    rospy.spin()
