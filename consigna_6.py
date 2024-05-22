import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


linear_velocity = 0.5    # Linear velocity in m/s
angular_velocity = 0.5   # Angular velocity in rad/s
state_start_time = time.time()

while True:
    current_time = time.time()
    elapsed_time = current_time - state_start_time
    def move_straight():
        twist = Twist()
        twist.linear.x = linear_velocity  # Move forward with defined linear velocity
        twist.angular.z = 0.0
        publisher_.publish(twist)
        state_start_time = time.time()
        if (elapsed_time >= 4):
            state_start_time = current_time
            stop_robot()           

    def turn():
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_velocity  # Turn with defined angular velocity
        publisher_.publish(twist)
        state_start_time = time.time()
        if (elapsed_time >= (3.14 / 4)):
            state_start_time = current_time
            stop_robot()

    def stop_robot():
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        publisher_.publish(twist)
        state_start_time = time.time()


class SquarePathNode(Node):
    def __init__(self):
        super().__init__('square_path')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_square)  # Check more frequently
        self.state = 0
        self.states = [('move_straight', 4.0),   # Move straight for 4 seconds (assuming 0.5 m/s * 4s = 2m)
                       ('turn', 3.14 / 4)]         # Turn for the time required to achieve 90 degrees at given angular velocity
        self.linear_velocity = 0.5    # Linear velocity in m/s
        self.angular_velocity = 0.5   # Angular velocity in rad/s
        self.state_start_time = time.time()

    def move_square(self):
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time
        
        if self.state % 2 == 0:  # Moving straight
            self.move_straight()
        else:  # Turning
            self.turn()
        
        if (self.state % 2 == 0 and elapsed_time >= self.states[self.state % 2][1]) or \
           (self.state % 2 == 1 and elapsed_time >= (self.states[self.state % 2][1] / self.angular_velocity)):
            self.state = (self.state + 1) % 8  # We have 4 sides (2 states * 4)
            self.state_start_time = current_time
            self.stop_robot()

    def move_straight(self):
        twist = Twist()
        twist.linear.x = self.linear_velocity  # Move forward with defined linear velocity
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def turn(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_velocity  # Turn with defined angular velocity
        self.publisher_.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquarePathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
