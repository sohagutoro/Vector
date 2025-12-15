#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math

class TrajectoryPublisher(Node):
    def __init__(self, robot_type='mecanum'):
        super().__init__('trajectory_publisher')
        self.robot_type = robot_type
        
        # Для 2SWD нужны отдельные топики управления
        if robot_type == 'swerve':
            self.cmd_pub = self.create_publisher(Twist, f'/{robot_type}_robot/cmd_vel', 10)
        else:
            self.cmd_pub = self.create_publisher(Twist, f'/{robot_type}_robot/cmd_vel', 10)
        
        self.timer = self.create_timer(0.01, self.publish_trajectory)  # 100 Hz
        self.start_time = self.get_clock().now()
        
        # Параметры тестовой траектории из диссертации
        self.radius = 2.0  # м
        self.omega = 0.5   # рад/с
        self.target_orientation = math.pi / 4
        
    def publish_trajectory(self):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        if current_time > 25.0:  # Остановка через 25 секунд
            twist = Twist()
            self.cmd_pub.publish(twist)
            return
        
        # Расчет желаемой скорости по формуле (5) из диссертации
        # x(t) = 2*sin(0.5*t), y(t) = 2*cos(0.5*t)
        vx = self.radius * self.omega * math.cos(self.omega * current_time)
        vy = -self.radius * self.omega * math.sin(self.omega * current_time)
        
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = 0.0  # Ориентация постоянная
        
        self.cmd_pub.publish(twist)
        
        # Логирование каждую секунду
        if int(current_time) != int(current_time - 0.01):
            self.get_logger().info(
                f"Time: {current_time:.2f}s, "
                f"vx: {vx:.3f} m/s, "
                f"vy: {vy:.3f} m/s"
            )

def main(args=None):
    rclpy.init(args=args)
    
    import sys
    robot_type = 'mecanum'
    if len(sys.argv) > 1:
        robot_type = sys.argv[1]
    
    node = TrajectoryPublisher(robot_type)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()