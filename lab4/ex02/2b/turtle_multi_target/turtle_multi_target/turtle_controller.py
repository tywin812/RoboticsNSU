import math
from geometry_msgs.msg import Twist
from turtle_multi_target_interfaces.msg import CurrentTarget
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import sys
import select

from turtlesim.srv import Spawn

class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')

        self.switch_threshold = self.declare_parameter(
          'switch_threshold', 1.0).get_parameter_value().double_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.turtle_publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)
        self.target_publisher = self.create_publisher(CurrentTarget, '/current_target', 1)

        self.targets = ['carrot1', 'carrot2', 'static_target']
        self.cur_target = 0

        self.timer = self.create_timer(0.1, self.switch_target_callback)


    def switch_target_callback(self):

        to_frame_rel = 'turtle2'
        current_target = self.targets[self.cur_target]

        if self._is_switch_pressed():
            self.cur_target = (self.cur_target + 1) % len(self.targets)
            current_target = self.targets[self.cur_target]
            self.get_logger().info(f'Switch button pressed. Switching to next target: {current_target}')

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                current_target, 
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().info( 
                f'Could not transform {to_frame_rel} to {current_target}: {ex}')
            return
        
        x = t.transform.translation.x
        y = t.transform.translation.y
        distance_to_target = math.sqrt(x**2 + y**2)

        if distance_to_target < self.switch_threshold:
            self.cur_target = (self.cur_target + 1) % len(self.targets)
            current_target = self.targets[self.cur_target]
            self.get_logger().info(f'Switching to target: {current_target}')
            
        twist_msg = Twist()
        scale_forward_speed = 1.0
        twist_msg.linear.x = scale_forward_speed * distance_to_target
        scale_rotation_rate = 1.0
        twist_msg.angular.z = scale_rotation_rate * math.atan2(y, x)
        self.turtle_publisher.publish(twist_msg)

        target_msg = CurrentTarget()
        target_msg.target_name = current_target
        target_msg.target_x = x
        target_msg.target_y = y
        target_msg.distance_to_target = distance_to_target
        self.target_publisher.publish(target_msg)

    def _is_switch_pressed(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            return line.lower() == ''
        return False
        
def main():
    rclpy.init()
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()