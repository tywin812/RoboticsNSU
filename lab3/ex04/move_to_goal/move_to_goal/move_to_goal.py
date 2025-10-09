import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class MoveToGoal(Node):
    def __init__(self, goal_x, goal_y, goal_theta):
        super().__init__('move_to_goal')
        
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.pose = None
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None:
            return
        
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        distance = math.sqrt(dx*dx + dy*dy)

        angle_to_goal = math.atan2(dy, dx)
        angle_error = (angle_to_goal - self.pose.theta) % (2*math.pi)

        msg = Twist()

        if distance > 0.05:
            if abs(angle_error) > 0.1: 
                msg.angular.z = angle_error
            else:
                msg.linear.x = distance
                msg.angular.z = angle_error
        else:
            theta_error = (self.goal_theta - self.pose.theta) % (2*math.pi)
            if abs(theta_error) > 0.05:
                msg.angular.z = theta_error
            else:
                self.get_logger().info("Goal reached!")
                self.cmd_pub.publish(Twist())  
                rclpy.shutdown()
                return
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()

    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal move_to_goal X Y THETA")
        return

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_theta = float(sys.argv[3])

    node = MoveToGoal(goal_x, goal_y, goal_theta)
    rclpy.spin(node)


if __name__ == '__main__':
    main()