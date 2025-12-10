#!/usr/bin/env python3
import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer


class PenguinListener(Node):

    def __init__(self):
        super().__init__('penguin_tf_listener')

        self.target_frame = self.declare_parameter(
            'target_frame', 'fish').get_parameter_value().string_value
        
        # Assuming the penguin's base frame is 'base_link'
        # If you are using a namespace, you might need to adjust this or rely on TF tree
        self.source_frame = self.declare_parameter(
            'source_frame', 'base_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = self.source_frame

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}')
            return

        msg = Twist()
        scale_forward_speed = 0.5
        scale_rotation_speed = 1.0

        x = t.transform.translation.x
        y = t.transform.translation.y
        
        distance = math.sqrt(x ** 2 + y ** 2)

        # Only move if distance is greater than a small threshold to avoid jitter
        if distance > 0.1:
            msg.angular.z = scale_rotation_speed * math.atan2(y, x)
            msg.linear.x = scale_forward_speed * distance
            
            # Cap the speed
            if msg.linear.x > 1.0:
                msg.linear.x = 1.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = PenguinListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
