#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random

class RandomFishMovement(Node):

    def __init__(self):
        super().__init__('random_fish_movement')

        self.publisher = self.create_publisher(Twist, '/fish/cmd_vel', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.timer_calls = 0
        self.timer_calls_threshold = 10

        self.cur_twist = Twist()

    def timer_callback(self):
        if self.timer_calls == self.timer_calls_threshold:
            self.cur_twist.linear.x = random.uniform(0.5, 1.0)
            self.cur_twist.angular.z = random.uniform(-1.0, 1.0)
            self.get_logger().info(f'Changing direction: linear={self.cur_twist.linear.x:.2f}, angular={self.cur_twist.angular.z:.2f}')
            self.timer_calls = 0

        self.publisher.publish(self.cur_twist)

def main():
    rclpy.init()
    node = RandomFishMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()