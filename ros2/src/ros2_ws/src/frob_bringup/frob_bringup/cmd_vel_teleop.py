import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Teleop Keyboard Node Initialized!')
        self.run()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        msg = Twist()
        self.get_logger().info("Use 'WASD' to move, 'Q' to quit.")

        while rclpy.ok():
            key = self.get_key()
            if key == 'w':
                msg.linear.x = 0.012
                msg.angular.z = 0.0
            elif key == 's':
                msg.linear.x = -0.012
                msg.angular.z = 0.0
            elif key == 'a':
                msg.linear.x = 0.0
                msg.angular.z = 0.07
            elif key == 'd':
                msg.linear.x = 0.0
                msg.angular.z = -0.07
            elif key == 'q':
                self.get_logger().info("Exiting...")
                exit(0)
                break
            elif key == 'r':
                msg.linear.x = 0.0
                msg.angular.z = 0.0


            self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TeleopKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
