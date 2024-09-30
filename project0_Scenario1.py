import rclpy  # ROS2 Python client library
from rclpy.node import Node  # Node class from ROS2
from geometry_msgs.msg import Twist  # Message type for velocity control
import time  # Time module to track time duration

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('tb_openLoop')  # Initialize the node with the name 'tb_openLoop'

        # Add logging to verify that the node starts
        self.get_logger().info("TurtleBotController node has started")  # Logging info

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # Create publisher to '/cmd_vel' topic
        self.timer = self.create_timer(0.1, self.move_robot)  # Timer calls 'move_robot' every 0.1 seconds
        self.start_time = time.time()  # Record the start time
        self.target_time = 10  # Define the duration for movement in seconds
        self.velocity = 0.1  # Define constant velocity in m/s (adjust as necessary)
        self.moving = True  # Boolean to track if the robot is moving

        # Add logging to confirm movement setup
        self.get_logger().info("Movement setup: Moving at velocity {} m/s for {} seconds".format(self.velocity, self.target_time))

    def move_robot(self):
        current_time = time.time() - self.start_time  # Calculate elapsed time since movement started

        if current_time < self.target_time and self.moving:  # If still within the target time
            twist = Twist()  # Create Twist message to define velocity commands
            twist.linear.x = self.velocity  # Set the linear velocity (forward movement)
            twist.angular.z = 0.0  # No angular movement (no rotation)
            self.publisher_.publish(twist)  # Publish the velocity command to '/cmd_vel'

            # Add logging to confirm velocity publishing
            self.get_logger().info("Publishing velocity: {} m/s".format(self.velocity))
        else:
            self.stop_robot()  # Stop the robot once the target time is reached

    def stop_robot(self):
        twist = Twist()  # Create a new Twist message to stop the robot
        twist.linear.x = 0.0  # Set linear velocity to zero (stop forward motion)
        twist.angular.z = 0.0  # No angular movement
        self.publisher_.publish(twist)  # Publish the stop command
        self.moving = False  # Set 'moving' to False to indicate the robot has stopped

        # Add logging to confirm the robot has stopped
        self.get_logger().info("Robot has stopped moving")

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 communication
    node = TurtleBotController()  # Create an instance of the TurtleBotController class
    rclpy.spin(node)  # Keep the node running until stopped manually
    node.destroy_node()  # Destroy the node once done
rclpy.shutdown()  # Shutdown the ROS2 system

if __name__ == '__main__': 
    main()