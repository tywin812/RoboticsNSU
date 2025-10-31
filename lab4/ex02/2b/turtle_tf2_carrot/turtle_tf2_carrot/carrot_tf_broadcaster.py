import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class CarrotFramePublisher(Node):

    def __init__(self):
        super().__init__('carrot_tf_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.parent_frame = self.declare_parameter(
          'parent_frame', 'turtle1').get_parameter_value().string_value

        self.child_frame = self.declare_parameter(
          'child_frame', 'carrot1').get_parameter_value().string_value
        
        self.radius = self.declare_parameter('radius', 1.0).get_parameter_value().double_value
        self.direction = self.declare_parameter('direction_of_rotation', 1).get_parameter_value().integer_value
                
        self.angle = 0.0

        self.timer = self.create_timer(0.1, self.handle_carrot_pose)


    def handle_carrot_pose(self):
        
        self.angle += self.direction * 0.1
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = self.radius * math.cos(self.angle)
        t.transform.translation.y = self.radius * math.sin(self.angle)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = CarrotFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()