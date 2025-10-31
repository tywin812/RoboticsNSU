import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose


class TargetSwitcher(Node):
    
    def __init__(self):
        super().__init__('target_switcher')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_frames_callback)

        self.radius1 = self.declare_parameter('carrot1_radius', 1.0).get_parameter_value().double_value
        self.direction1 = self.declare_parameter('carrot1_direction', 1).get_parameter_value().integer_value
        self.angle1 = 0.0

        self.radius2 = self.declare_parameter('carrot2_radius', 1.0).get_parameter_value().double_value
        self.direction2 = self.declare_parameter('carrot2_direction', 1).get_parameter_value().integer_value
        self.angle2 = 0.0

    def broadcast_frames_callback(self):

        self.angle1 += self.direction1 * 0.1
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'turtle1'
        t1.child_frame_id = 'carrot1'
        t1.transform.translation.x = self.radius1 * math.cos(self.angle1)
        t1.transform.translation.y = self.radius1 * math.sin(self.angle1)
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t1)
    
        self.angle2 += self.direction2 * 0.1
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'turtle3'
        t2.child_frame_id = 'carrot2'
        t2.transform.translation.x = self.radius2 * math.cos(self.angle2)
        t2.transform.translation.y = self.radius2 * math.sin(self.angle2)
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)

        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'world'
        t3.child_frame_id = 'static_target'
        t3.transform.translation.x = 8.0
        t3.transform.translation.y = 2.0
        t3.transform.translation.z = 0.0 
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t3)

def main():
    rclpy.init()
    node = TargetSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()