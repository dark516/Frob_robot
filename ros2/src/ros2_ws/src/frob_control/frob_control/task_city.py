import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist

class AnsCmdVelNode(Node):
    def __init__(self):
        super().__init__('city_node')

        self.camera_ans = self.create_subscription(
            Int32,
            '/camera/ans',
            self.camera_callback,
            10
        )

        self.left_distance = self.create_subscription(
            Float32,
            '/left_distance',
            self.left_dist_callback,
            10
        )
        
        self.right_distance = self.create_subscription(
            Float32,
            '/right_distance',
            self.right_dist_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.kP = 0.4 #TODO: подобрать
        #self.timer = self.create_timer(0.5, self.timer_callback)
        self.cmd_vel_msg = Twist()
        self.last_camera_ans = None
        self.left_dist = None
        self.right_dist = None

    def camera_callback(self, msg):
        self.get_logger().info(f' /camera/ans: {msg.data}')
        self.last_camera_ans = msg.data

    def left_dist_callback(self, msg):
        #self.get_logger().info(f' left_dist: {msg.data}')
        self.left_dist = msg.data
        self.callback()
    def right_dist_callback(self, msg):
        #self.get_logger().info(f' right_dist: {msg.data}')
        self.right_dist = msg.data
#        self.callback()
    
    def callback(self):
        # self.get_logger().info('No data from /ans, executing default behavior.')
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.angular.z = (0.4 - self.left_dist) * self.kP
        
        self.cmd_vel_msg.linear.x = 0.3 #TODO: подобрать
        self.get_logger().info(f'{self.left_dist}')
        self.cmd_vel_pub.publish(self.cmd_vel_msg)


        #self.get_logger().info('Sent default values to /cmd_vel (linear.x=0.0, angular.z=0.0)')

def main(args=None):
    rclpy.init(args=args)
    node = AnsCmdVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
