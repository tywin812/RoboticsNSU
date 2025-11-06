import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Turtle2FrameListener(Node):

    def __init__(self):
        super().__init__('turtle2_frame_listener')

        self.target_frame = self.declare_parameter(
          'target_frame', 'turtle1').get_parameter_value().string_value

        self.turtlename = self.declare_parameter(
          'turtlename', 'turtle2').get_parameter_value().string_value
        
        self.delay = self.declare_parameter('delay', 5.0).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.turtle_spawning_service_ready = False
        self.turtle_spawned = False

        self.publisher = self.create_publisher(Twist, f'{self.turtlename}/cmd_vel', 1)

        self.timer = self.create_timer(0.05, self.on_timer)

    def on_timer(self):

        from_frame_rel = self.target_frame
        to_frame_rel = self.turtlename

        when = self.get_clock().now() - rclpy.time.Duration(seconds=self.delay)

        try:
            t = self.tf_buffer.lookup_transform_full(
                target_frame=to_frame_rel,     
                source_frame=from_frame_rel,    
                source_time=when,              
                target_time=rclpy.time.Time(),      
                fixed_frame='world',
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        msg = Twist()
        scale_rotation_rate = 1.0
        msg.angular.z = scale_rotation_rate * math.atan2(
            t.transform.translation.y,
            t.transform.translation.x)

        scale_forward_speed = 0.5
        msg.linear.x = scale_forward_speed * math.sqrt(
            t.transform.translation.x ** 2 +
            t.transform.translation.y ** 2)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = Turtle2FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()