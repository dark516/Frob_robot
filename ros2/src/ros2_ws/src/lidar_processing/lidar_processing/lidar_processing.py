import numpy as np
import rclpy
from std_msgs.msg import Float32
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ScanSubscriber(Node):

    def __init__(self):
        super().__init__('scan_subscriber')

        # Определяем профиль QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT  # Политика "наилучшее усилие"

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )


        self.subscription  # чтобы предотвратить удаление подписки
        self.scan_angle = 30 # 0.5 percent is 1
        self.left_range = Float32()
        self.right_range = Float32()
        self.forward_range = Float32()

        self.min_left_forward_range = Float32()
        self.min_right_forward_range = Float32()

        self.min_left_forward_range_publisher = self.create_publisher(Float32, "/min_left_forward_range", 10)
        self.min_right_forward_range_publisher = self.create_publisher(Float32, "/min_right_forward_range", 10)

        self.left_publisher = self.create_publisher(Float32, '/left_distance', 10)
        self.right_publisher = self.create_publisher(Float32, '/right_distance', 10)
        self.forward_publisher = self.create_publisher(Float32, '/forward_distance', 10)

    def log(self, msg:LaserScan):
        with open(r"/home/robot/Frob_robot/ros2/src/ros2_ws/src/lidar_processing/lidar_processing/scanlog.txt", "w") as f:
            f.write(f"ai {msg.angle_increment}\namax{msg.angle_max}\namin{msg.angle_min}\nheader{msg.header}\nint{msg.intensities}\nrmax{msg.range_max}\nrmin{msg.range_min}\nRANGES{msg.ranges}\nst{msg.scan_time}\nti{msg.time_increment}\nRANGES_COUNT{len(msg.ranges)}")

    def scan_callback(self, msg: LaserScan):
        self.left_range = float(np.median(msg.ranges[0:0+self.scan_angle]))
        self.forward_range = float(np.median(msg.ranges[178-self.scan_angle:178 + self.scan_angle]))
        self.right_range = float(np.median(msg.ranges[357-self.scan_angle:357]))

        self.min_left_forward_range = min(filter(lambda x: x > .0, msg.ranges[178:178+90]), default=-1.)
        self.min_right_forward_range = min(filter(lambda x: x > .0, msg.ranges[178-90:178]), default=-1.)
        self.publish_distance()

    def publish_distance(self):
        left_range = Float32(data=self.left_range)
        right_range = Float32(data=self.right_range)
        forward_range = Float32(data=self.forward_range)

        left_forward_min_range = Float32(data=self.min_left_forward_range)
        right_forward_min_range = Float32(data=self.min_right_forward_range)

        self.left_publisher.publish(left_range)
        self.forward_publisher.publish(forward_range)
        self.right_publisher.publish(right_range)

        self.min_left_forward_range_publisher.publish(left_forward_min_range)
        self.min_right_forward_range_publisher.publish(right_forward_min_range)


# 1 лево
# 2 перед
# 3 право
# 4 зад

def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

