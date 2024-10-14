import rclpy
from rclpy.node import Node
from frob_interfaces.srv import TurnByEncoder
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class TurnByEncoderService(Node):
    def __init__(self):
        super().__init__('turn_by_encoder_service')

        # Создаем сервис
        self.srv = self.create_service(TurnByEncoder, 'turn_by_encoder', self.turn_callback)
        
        # Переменные для хранения данных энкодеров
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.target_ticks = 0  # Целевое количество тиков
        self.moving = False  # Флаг движения

        # Подписка на топики энкодеров
        self.left_encoder_sub = self.create_subscription(
            Int32, '/left_motor/encoder/delta', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(
            Int32, '/right_motor/encoder/delta', self.right_encoder_callback, 10)

        # Издатель для управления движением
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def left_encoder_callback(self, msg):
        if self.moving:
            self.left_encoder_ticks += msg.data
            self.check_if_done()

    def right_encoder_callback(self, msg):
        if self.moving:
            self.right_encoder_ticks += msg.data
            self.check_if_done()

    def check_if_done(self):
        # Проверяем, достигли ли энкодеры целевого количества тиков
        if abs(self.left_encoder_ticks) >= self.target_ticks and abs(self.right_encoder_ticks) >= self.target_ticks:
            self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)  # Останавливаем робота
        self.moving = False
        self.get_logger().info('Turn completed successfully')

    def turn_callback(self, request, response):
        self.get_logger().info(f'Received request to turn by {request.ticks} encoder ticks')

        # Инициализируем цель и обнуляем текущие тики
        self.target_ticks = abs(request.ticks)
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.moving = True

        # Поворачиваем робота
        twist = Twist()
        twist.angular.z = 0.3 if request.ticks > 0 else -0.3
        self.cmd_vel_pub.publish(twist)

        # Пока двигается, сервис возвращает успешный ответ
        response.success = True
        response.message = 'Turning initiated'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurnByEncoderService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
