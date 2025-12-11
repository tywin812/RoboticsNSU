#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

def get_yaw_from_quat(q):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)

class ClosedLoopFigureEight(Node):
    def __init__(self):
        super().__init__('figure_eight_controller')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.t = 0.0
        self.scale = 1.0 
        self.speed_scale = 0.8 
        
        self.kx = 2.0
        self.ky = 20.0 
        self.kth = 4.0  
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = get_yaw_from_quat(msg.pose.pose.orientation)
        
        if not self.odom_received:
            self.start_x = self.x
            self.start_y = self.y
            self.start_yaw = self.yaw
            self.odom_received = True

    def control_loop(self):
        if not self.odom_received:
            return
        
        t_now = self.t * self.speed_scale
        
        xd_ref = self.scale * math.sin(t_now)
        yd_ref = self.scale * math.sin(t_now) * math.cos(t_now)
        
        xd_dot = self.scale * math.cos(t_now) * self.speed_scale
        yd_dot = self.scale * math.cos(2 * t_now) * self.speed_scale
        
        yaw_ref = math.atan2(yd_dot, xd_dot)
        
        v_ref = math.sqrt(xd_dot**2 + yd_dot**2)
        w_ref = (xd_dot * (-self.scale * 2 * math.sin(2*t_now) * self.speed_scale**2) - 
                 yd_dot * (-self.scale * math.sin(t_now) * self.speed_scale**2)) / (v_ref**2 + 1e-6)

        ca = math.cos(self.start_yaw)
        sa = math.sin(self.start_yaw)
        
        x_target = self.start_x + (xd_ref * ca - yd_ref * sa)
        y_target = self.start_y + (xd_ref * sa + yd_ref * ca)
        yaw_target = yaw_ref + self.start_yaw
        
        dx = x_target - self.x
        dy = y_target - self.y
        
        e_theta = yaw_target - self.yaw
        while e_theta > math.pi: e_theta -= 2*math.pi
        while e_theta < -math.pi: e_theta += 2*math.pi
        
        c_r = math.cos(self.yaw)
        s_r = math.sin(self.yaw)
        e_x = dx * c_r + dy * s_r
        e_y = -dx * s_r + dy * c_r

        linear_cmd = v_ref * math.cos(e_theta) + self.kx * e_x
        
        angular_cmd = w_ref + self.ky * e_y + self.kth * math.sin(e_theta)
        
        msg = Twist()
        msg.linear.x = linear_cmd
        msg.angular.z = angular_cmd
        self.publisher_.publish(msg)
        
        self.t += self.dt

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopFigureEight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
