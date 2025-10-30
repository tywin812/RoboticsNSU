#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from action_cleaning_robot.action import CleaningTask
import math
import time


class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_server')
        
        self._action_server = ActionServer(
            self,
            CleaningTask,
            'cleaning_task',
            self.execute_callback
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.current_pose = Pose()
        self.pose_received = False
        
        self.get_logger().info('Cleaning Action Server started')

    def pose_callback(self, msg):
        self.current_pose = msg
        self.pose_received = True

    def execute_callback(self, goal_handle):
        while not self.pose_received:
            time.sleep(0.1)
        
        task_type = goal_handle.request.task_type
        feedback_msg = CleaningTask.Feedback()
        
        self.get_logger().info(f'Starting task: {task_type}')
        
        if task_type == "clean_square":
            area_size = goal_handle.request.area_size
            self.get_logger().info(f'Cleaning square: {area_size}x{area_size} meters')
            result = self.clean_square(area_size, goal_handle, feedback_msg)
        elif task_type == "return_home":
            target_x = goal_handle.request.target_x
            target_y = goal_handle.request.target_y
            self.get_logger().info(f'Returning home to ({target_x}, {target_y})')
            result = self.return_home(target_x, target_y, goal_handle, feedback_msg)
        else:
            self.get_logger().error(f'Unknown task type: {task_type}')
            result = self.create_result(False, 0, 0.0)
            goal_handle.abort()
            return result
        
        self.get_logger().info(f'Task completed! Success: {result.success}, '
                             f'Cleaned points: {result.cleaned_points}, '
                             f'Distance: {result.total_distance:.2f}m')
        
        goal_handle.succeed()
        return result

    def clean_square(self, side_length, goal_handle, feedback_msg):
        start_x = self.current_pose.x
        start_y = self.current_pose.y
        stripe_width = 0.05
        num_stripes = max(1, int(side_length / stripe_width))
        
        self.get_logger().info(f'Square cleaning: start=({start_x:.2f}, {start_y:.2f}), '
                             f'side_length={side_length:.2f}m, stripes={num_stripes}')
        
        cleaned_points = 0
        total_distance = 0.0
        
        for i in range(1, num_stripes + 1):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Square cleaning cancelled')
                self.stop_robot()
                return self.create_result(False, cleaned_points, total_distance)
            
            current_stripe_y = start_y + i * stripe_width
            
            if current_stripe_y > start_y + side_length:
                current_stripe_y = start_y + side_length
            
            if i % 2 == 1:
                total_distance += self.move_to_point(start_x + side_length, current_stripe_y, goal_handle)
            else:
                total_distance += self.move_to_point(start_x, current_stripe_y, goal_handle)
            
            cleaned_points = int((i / num_stripes) * side_length * side_length * 10)
            
            feedback_msg.progress_percent = min(int((i / num_stripes) * 100), 100)
            feedback_msg.current_cleaned_points = cleaned_points
            feedback_msg.current_x = self.current_pose.x
            feedback_msg.current_y = self.current_pose.y
            goal_handle.publish_feedback(feedback_msg)
        
        self.stop_robot()
        cleaned_points = int(side_length * side_length * 10)
        self.get_logger().info(f'Square cleaned: {cleaned_points} points, {total_distance:.2f}m')
        return self.create_result(True, cleaned_points, total_distance)

    def return_home(self, target_x, target_y, goal_handle, feedback_msg):
        self.get_logger().info(f'Moving from ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}) '
                             f'to home ({target_x:.2f}, {target_y:.2f})')
        
        distance = self.move_to_point(target_x, target_y, goal_handle)
        
        feedback_msg.progress_percent = 100
        feedback_msg.current_cleaned_points = 0
        feedback_msg.current_x = self.current_pose.x
        feedback_msg.current_y = self.current_pose.y
        goal_handle.publish_feedback(feedback_msg)
        
        self.stop_robot()
        self.get_logger().info(f'Arrived home! Distance traveled: {distance:.2f}m')
        return self.create_result(True, 0, distance)

    def move_to_point(self, target_x, target_y, goal_handle):
        FIELD_MIN = 0.5
        FIELD_MAX = 10.5
        
        target_x = max(FIELD_MIN, min(FIELD_MAX, target_x))
        target_y = max(FIELD_MIN, min(FIELD_MAX, target_y))
        
        self.get_logger().info(f'Moving to ({target_x:.2f}, {target_y:.2f}), clamped to field bounds')
        
        twist = Twist()
        
        while True:
            if goal_handle.is_cancel_requested:
                break
            
            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            
            target_angle = math.atan2(dy, dx)
            angle_diff = (target_angle - self.current_pose.theta) % (2*math.pi)

            if abs(angle_diff) < 0.05:
                break
            
            twist.linear.x = 0.0
            twist.angular.z = 1.5 if angle_diff > 0 else -1.5
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
                
        self.get_logger().info(f'Starting forward movement')
        prev_distance = float('inf')
        no_progress_count = 0
        
        while True: 
            if goal_handle.is_cancel_requested:
                break
            
            dx = target_x - self.current_pose.x
            dy = target_y - self.current_pose.y
            distance = math.sqrt(dx**2 + dy**2)
                        
            if distance < 0.1:
                self.get_logger().info(f'Target reached! Final distance: {distance:.4f}m')
                break
            
            if abs(prev_distance - distance) < 0.001:
                no_progress_count += 1
                if no_progress_count > 30: 
                    self.get_logger().info(f'No progress detected. Distance: {distance:.4f}m')
                    break
            else:
                no_progress_count = 0
            
            prev_distance = distance

            twist.linear.x = 1.5

            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        self.get_logger().info('Stopping')
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rclpy.spin_once(self, timeout_sec=0.05)
        
        final_dx = target_x - self.current_pose.x
        final_dy = target_y - self.current_pose.y
        final_distance = math.sqrt(final_dx**2 + final_dy**2)
        self.get_logger().info(f'Movement complete. Final position: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}), Distance to target: {final_distance:.4f}m')
        
        return final_distance

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.2)

    def create_result(self, success, cleaned_points, total_distance):
        result = CleaningTask.Result()
        result.success = success
        result.cleaned_points = cleaned_points
        result.total_distance = total_distance
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CleaningActionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()