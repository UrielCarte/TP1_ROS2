import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SquarePathNode(Node):
    def __init__(self):
        super().__init__('square_path_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_square)
        
        self.linear_velocity = 0.5    # Velocidad lineal en m/s
        self.angular_velocity = 0.5   # Velocidad angular en rad/s
        self.state_start_time = self.get_clock().now().seconds_nanoseconds()
        self.elapsed_time = 0
        self.state = 'MOVE_STRAIGHT'  # Estado inicial

    def move_square(self):
        current_time = self.get_clock().now().seconds_nanoseconds()
        self.elapsed_time = (current_time[0] - self.state_start_time[0]) + (current_time[1] - self.state_start_time[1]) / 1e9

        if self.state == 'MOVE_STRAIGHT':
            if self.elapsed_time < 4:
                self.move_straight()
            else:
                self.stop_robot()
                self.state_start_time = self.get_clock().now().seconds_nanoseconds()
                self.state = 'TURN'

        elif self.state == 'TURN':
            # Duración del giro para alcanzar 90 grados (π/2 radianes)
            turn_duration = (3.14 / 2) / self.angular_velocity
            if self.elapsed_time < turn_duration:
                self.turn()
            else:
                self.stop_robot()
                self.state_start_time = self.get_clock().now().seconds_nanoseconds()
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
