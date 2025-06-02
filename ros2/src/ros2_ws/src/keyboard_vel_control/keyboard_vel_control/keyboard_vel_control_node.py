#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios
import tty
import sys
import select

class KeyboardVelControl(Node):
    def __init__(self):
        super().__init__('keyboard_vel_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("Keyboard velocity control node started")
        self.get_logger().info("Use WASD to control the robot:")
        self.get_logger().info("  W/S: Increase/Decrease linear velocity")
        self.get_logger().info("  A/D: Increase/Decrease angular velocity")
        self.get_logger().info("  Space: Stop the robot")
        self.get_logger().info("  Q: Quit")
        
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.last_published_vel = Twist()
        self.publish_velocity()
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def process_key(self, key):
        if key == 'w':
            self.linear_vel += 0.1
        elif key == 's':
            self.linear_vel -= 0.1
        elif key == 'a':
            self.angular_vel += 0.1
        elif key == 'd':
            self.angular_vel -= 0.1
        elif key == ' ':
            self.linear_vel = 0.0
            self.angular_vel = 0.0
        elif key == 'q':
            return False
        
        # Ограничиваем скорости
        self.linear_vel = max(min(self.linear_vel, 1.0), -1.0)
        self.angular_vel = max(min(self.angular_vel, 1.0), -1.0)
        
        self.get_logger().info(f"Linear: {self.linear_vel:.1f}, Angular: {self.angular_vel:.1f}")
        return True
    
    def publish_velocity(self):
        key = self.get_key()
        if key:
            if not self.process_key(key):
                self.cleanup()
                rclpy.shutdown()
                return
        
        # Публикуем только если скорость изменилась
        current_vel = Twist()
        current_vel.linear.x = self.linear_vel
        current_vel.angular.z = self.angular_vel
        
        if (current_vel.linear.x != self.last_published_vel.linear.x or
            current_vel.angular.z != self.last_published_vel.angular.z):
            self.publisher.publish(current_vel)
            self.last_published_vel = current_vel
    
    def cleanup(self):
        # Останавливаем робота при выходе
        stop_vel = Twist()
        self.publisher.publish(stop_vel)
        self.get_logger().info("Stopping the robot and shutting down")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardVelControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()

if __name__ == '__main__':
    main()
