import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot')

        # Publisher to send velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to receive odometry data
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Timer to periodically ask for user input
        self.timer = self.create_timer(1.0, self.get_user_input)

        # Store odometry data
        self.current_odom = None

        self.get_logger().info('MoveRobotNode has started!')

    def get_user_input(self):
        """
        Get user input for the robot's linear and angular velocities.
        """
        try:
            # Ask for user input to control the robot's movement
            linear_speed = float(input("Enter linear speed (m/s): "))
            angular_speed = float(input("Enter angular speed (rad/s): "))
        except ValueError:
            self.get_logger().warn("Invalid input. Please enter numeric values.")
            return

        # Create a Twist message with user input values
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        # Publish the velocity command
        self.publisher.publish(twist)
        self.get_logger().info(f"Publishing command: linear.x={linear_speed}, angular.z={angular_speed}")

    def odom_callback(self, msg):
        """
        Callback function that processes incoming odometry messages.
        """
        self.current_odom = msg
        # Log position and orientation from odometry
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        #self.get_logger().info(f"Odometry: Position - x: {position.x}, y: {position.y}, z: {position.z}")
        #self.get_logger().info(f"Orientation: x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

