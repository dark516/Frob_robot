import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PointStamped

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        self.filtered_pub = self.create_publisher(LaserScan, '/scan/filtered', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Параметры трансформации
        self.declare_parameter('source_frame', 'laser')  # Фрейм лидара из сообщения
        self.declare_parameter('target_frame', 'base_footprint')  # Целевой фрейм
        self.declare_parameter('filter_radius', 0.15)  # Радиус в метрах

    def scan_callback(self, msg):
        try:
            # Получаем трансформацию между laser и base_footprint через всю цепочку
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('source_frame').value,  # Исходный фрейм (laser)
                self.get_parameter('target_frame').value,  # Целевой фрейм (base_footprint)
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warning(f"Transform error: {str(e)}")
            return

        # Позиция base_footprint в системе координат laser
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        
        filtered_ranges = list(msg.ranges)
        radius = self.get_parameter('filter_radius').value
        radius_sq = radius ** 2
        
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max):
                continue
            
            # Вычисляем координаты точки относительно laser
            angle = msg.angle_min + i * msg.angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            
            # Вычисляем расстояние до base_footprint
            dx = x - tx
            dy = y - ty
            dist_sq = dx**2 + dy**2
            
            if dist_sq < radius_sq:
                filtered_ranges[i] = 0.0  # Помечаем как invalid

        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = msg.intensities
        
        self.filtered_pub.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
