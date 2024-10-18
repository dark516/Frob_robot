import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist

class AnsCmdVelNode(Node):
    def __init__(self):
        super().__init__('city_node')

        self.subscription = self.create_subscription(
            Int32,
            '/camera/ans',
            self.camera_callback,
            10
        )

        self.subscription = self.create_subscription(
            Float32,
            '/laser/left_distance',
            self.left_dist_callback,
            10
        )
        
        self.subscription = self.create_subscription(
            Float32,
            '/laser/right_distance',
            self.right_dist_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.kP = 0.02 #TODO: подобрать
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.last_camera_ans = None
        self.left_dist = None
        self.right_dist = None

    def camera_callback(self, msg):
        self.get_logger().info(f' /camera/ans: {msg.data}')
        self.last_camera_ans = msg.data

    def left_dist_callback(self, msg):
        #self.get_logger().info(f' left_dist: {msg.data}')
        self.left_dist = msg.data

    def right_dist_callback(self, msg):
        #self.get_logger().info(f' right_dist: {msg.data}')
        self.right_dist = msg.data

    def timer_callback(self):
       # self.get_logger().info('No data from /ans, executing default behavior.')
        if max(self.left_dist, self.right_dist) > 0.8:
            if min(self.left_dist, self.right_dist) != 0: #Если не нули и не напротив дома
                cmd_vel_msg = Twist()
                cmd_vel_msg.angular.z =(0,4 - min(self.left_dist, self.right_dist)) * self.kP
                self.cmd_vel_msg.linear.x = 0.2 #TODO: подобрать
                self.publisher.publish(cmd_vel_msg)


        self.get_logger().info('Sent default values to /cmd_vel (linear.x=0.0, angular.z=0.0)')

def main(args=None):
    rclpy.init(args=args)
    node = AnsCmdVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
