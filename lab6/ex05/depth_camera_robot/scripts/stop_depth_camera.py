#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np


class StopDepthCamera(Node):
    def __init__(self):
        super().__init__('stop_depth_camera')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera/depth_image',
            self.depth_callback,
            10)

        self.timer = self.create_timer(0.1, self.drive_forward)
        self.min_dist = 10.0
        self.stop_distance = 1.0
        self.is_blocked = False
        
        self.bridge = CvBridge()

    def depth_callback(self, depth_msg):
        try:
            depth_image = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
                depth_msg.height, depth_msg.width)
            
            height, width = depth_image.shape
            
            center_y = height // 2
            center_x = width // 2
            roi_size = 10
            
            roi = depth_image[
                center_y - roi_size : center_y + roi_size,
                center_x - roi_size : center_x + roi_size
            ]
                        
            valid_depths = roi[np.isfinite(roi) & (roi > 0.01) & (roi < 100.0)]
            
            if len(valid_depths) > 0:
                self.min_dist = np.min(valid_depths)
            else:
                self.min_dist = 10.0
            
            if self.min_dist < self.stop_distance:
                self.is_blocked = True
            else:
                self.is_blocked = False
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')
            self.min_dist = 10.0

    def drive_forward(self):
        msg = Twist()

        if self.is_blocked:
            msg.linear.x = 0.0
            self.get_logger().warn(f'STOP! Obstacle at {self.min_dist:.3f}m')
        else:
            msg.linear.x = 0.3

        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StopDepthCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
