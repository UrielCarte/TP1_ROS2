import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

class SquarePathNode(Node):
    def __init__(self):
        super()._init__('square_path_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.000001, self.move_square)  # Increased timer frequency

        self.linear_velocity = 0.5    # Linear velocity in m/s
        self.angular_velocity = 0.5   # Angular velocity in rad/s
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9  # Start time in seconds
        self.elapsed_time = 0
        self.state = 'MOVE_STRAIGHT'  # Initial state

    def move_square(self):
        current_time = self.get_clock().now().nanoseconds / 1e9  # Current time in seconds
        self.elapsed_time = current_time - self.state_start_time
        print(f"State: {self.state}, Elapsed Time: {self.elapsed_time:.6f}")  # Debug information

        if self.state == 'MOVE_STRAIGHT':
            if self.elapsed_time < 4:
                self.move_straight()
            else:
                self.stop_robot()
                self.state_start_time = self.get_clock().now().nanoseconds / 1e9
                self.elapsed_time = 0  # Reset elapsed time
                self.state = 'TURN'

        elif self.state == 'TURN':
            # Duration of the turn to achieve 90 degrees (π/2 radians), with a buffer
            turn_duration = (pi / 2) / self.angular_velocity           
            if self.elapsed_time < turn_duration:
                self.turn()
            else:
                print(f"Turn completed in: {self.elapsed_time:.6f} seconds")  # Debug information
                self.stop_robot()
                self.state_start_time = self.get_clock().now().nanoseconds / 1e9
                self.elapsed_time = 0  # Reset elapsed time
                self.state = 'MOVE_STRAIGHT'

    def move_straight(self):
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def turn(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_velocity
        self.publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquarePathNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()