import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class TurtleBotOpenLoop(Node):

    def __init__(self):
        super().__init__('tb_openLoop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer to call the function every 0.1 seconds
        self.start_time = time.time()  # Record the start time
        self.velocity_message = Twist()

        # Parameters for scenario 2
        self.acceleration = 0.05  # Acceleration rate
        self.max_velocity = 0.3   # Maximum velocity
        self.current_velocity = 0.0  # Current velocity
        self.state = 'acceleration'  # Initial state: acceleration phase
        self.acceleration_duration = 2.0  # Duration of acceleration phase (in seconds)
        self.deceleration_duration = 2.0  # Duration of deceleration phase (in seconds)
        self.steady_duration = 5.0  # Duration of constant velocity phase (in seconds)

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time  # Calculate elapsed time

        if self.state == 'acceleration':
            self.move_with_acceleration(elapsed_time)
        elif self.state == 'steady':
            self.move_with_constant_velocity(elapsed_time)
        elif self.state == 'deceleration':
            self.move_with_deceleration(elapsed_time)

    def move_with_acceleration(self, elapsed_time):
        """Acceleration phase"""
        if elapsed_time <= self.acceleration_duration:  # Continue accelerating
            self.current_velocity = self.acceleration * elapsed_time  # Velocity = acceleration * time
            if self.current_velocity > self.max_velocity:
                self.current_velocity = self.max_velocity
            self.get_logger().info(f"Accelerating, current velocity: {self.current_velocity:.2f} m/s")
        else:
            self.current_velocity = self.max_velocity  # Reached max velocity
            self.state = 'steady'  # Switch to constant velocity phase
            self.start_time = time.time()  # Reset the timer
            self.get_logger().info(f"Entering steady phase, velocity: {self.current_velocity:.2f} m/s")

        self.velocity_message.linear.x = self.current_velocity
        self.publisher_.publish(self.velocity_message)

    def move_with_constant_velocity(self, elapsed_time):
        """Constant velocity phase"""
        if elapsed_time <= self.steady_duration:  # Continue at constant velocity
            self.velocity_message.linear.x = self.current_velocity
            self.get_logger().info(f"Moving at constant velocity, speed: {self.current_velocity:.2f} m/s")
        else:
            self.state = 'deceleration'  # Switch to deceleration phase
            self.start_time = time.time()  # Reset the timer
            self.get_logger().info("Entering deceleration phase")

        self.publisher_.publish(self.velocity_message)

    def move_with_deceleration(self, elapsed_time):
        """Deceleration phase"""
        if elapsed_time <= self.deceleration_duration:  # Continue decelerating
            self.current_velocity = self.max_velocity - self.acceleration * elapsed_time  # Decrease velocity
            if self.current_velocity < 0:
                self.current_velocity = 0
            self.get_logger().info(f"Decelerating, current velocity: {self.current_velocity:.2f} m/s")
        else:
            self.current_velocity = 0.0  # Stop the TurtleBot
            self.get_logger().info("TurtleBot has stopped")

        self.velocity_message.linear.x = self.current_velocity
        self.publisher_.publish(self.velocity_message)


def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtleBotOpenLoop()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()