import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Int32
import math
from tf2_ros import TransformBroadcaster

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        
        # Объявление параметров с значениями по умолчанию
        self.declare_parameters(
            namespace='',
            parameters=[
                ('meters_per_tick', 0.00058),
                ('wheel_base', 0.3),
                ('update_rate', 10.0),
                ('publish_tf', True),
                ('odom_frame_id', 'odom'),
                ('base_frame_id', 'base_link'),
                ('encoder_min', -32768),
                ('encoder_max', 32767)
            ]
        )
        
        # Загрузка параметров
        self.meters_per_tick = self.get_parameter('meters_per_tick').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.update_rate = self.get_parameter('update_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.encoder_min = self.get_parameter('encoder_min').value
        self.encoder_max = self.get_parameter('encoder_max').value
        
        # Инициализация состояния
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_x = 0.0
        self.prev_theta = 0.0
        
        # Подписки на энкодеры
        self.create_subscription(Int32, '/left_motor/encoder/delta', self.left_callback, 10)
        self.create_subscription(Int32, '/right_motor/encoder/delta', self.right_callback, 10)
        
        # Таймер с динамической частотой
        self.create_timer(1.0 / self.update_rate, self.update_odometry)
        
        # Публикаторы
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

    def handle_encoder_overflow(self, value):
        """Обработка переполнения энкодеров"""
        if value > self.encoder_max:
            return value - (self.encoder_max - self.encoder_min)
        if value < self.encoder_min:
            return value + (self.encoder_max - self.encoder_min)
        return value

    def left_callback(self, msg):
        self.left_ticks += self.handle_encoder_overflow(msg.data)

    def right_callback(self, msg):
        self.right_ticks += self.handle_encoder_overflow(msg.data)

    def update_odometry(self):
        # Рассчет пройденного расстояния
        left_dist = self.left_ticks * self.meters_per_tick
        right_dist = self.right_ticks * self.meters_per_tick
        
        # Сброс счетчиков
        self.left_ticks = 0
        self.right_ticks = 0
        
        # Обновление позиции
        delta_distance = (left_dist + right_dist) / 2
        delta_theta = (right_dist - left_dist) / self.wheel_base
        
        self.theta += delta_theta
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        
        # Публикация данных
        self.publish_odometry()
        if self.publish_tf:
            self.publish_transform()

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        # Позиция
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = self.quaternion_from_yaw(self.theta)
        
        # Скорость
        delta_time = 1.0 / self.update_rate
        odom_msg.twist.twist.linear.x = (self.x - self.prev_x) / delta_time
        odom_msg.twist.twist.angular.z = (self.theta - self.prev_theta) / delta_time
        
        self.odom_pub.publish(odom_msg)
        self.prev_x = self.x
        self.prev_theta = self.theta

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = self.quaternion_from_yaw(self.theta)
        
        self.tf_broadcaster.sendTransform(transform)

    def quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
