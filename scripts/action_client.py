import rospy
import actionlib
from assignment_2_2024.msg import MoveToTargetAction, MoveToTargetGoal, PositionVelocity
from nav_msgs.msg import Odometry


class ActionClientNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('action_client_node')

        # Action client to interact with the action server
        self.client = actionlib.SimpleActionClient('move_to_target', MoveToTargetAction)
        self.client.wait_for_server()

        # Publisher for the custom message
        self.pub = rospy.Publisher('/position_velocity', PositionVelocity, queue_size=10)

        # Subscriber to the /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Store robot velocity
        self.current_velocity = (0.0, 0.0)

    def send_target(self, x, y):
        # Send a target goal to the action server
        goal = MoveToTargetGoal(target_x=x, target_y=y)
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

    def cancel_target(self):
        # Cancel the current goal
        self.client.cancel_goal()
        rospy.loginfo("Target canceled.")

    def feedback_callback(self, feedback):
        # Handle feedback from the action server
        rospy.loginfo(f"Feedback: Current position ({feedback.feedback_x}, {feedback.feedback_y})")

    def odom_callback(self, msg):
        # Handle /odom messages and publish the custom message
        self.current_velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        position_velocity = PositionVelocity(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            vel_x=self.current_velocity[0],
            vel_z=self.current_velocity[1]
        )
        self.pub.publish(position_velocity)


if __name__ == '__main__':
    node = ActionClientNode()
    try:
        node.send_target(5.0, 5.0)  # Example target
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

