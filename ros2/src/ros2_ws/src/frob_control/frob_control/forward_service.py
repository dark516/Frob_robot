import rclpy
from rclpy.node import Node
from frob_interfaces.srv import DriveByEncoder
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class DriveByEncoderServer(Node):
    def __init__(self):
        super().__init__('drive_by_encoder_server')
        self.srv = self.create_service(DriveByEncoder, 'drive_by_encoder', self.drive_by_encoder_callback)

        # Подписываемся на данные энкодеров
        self.left_encoder_sub = self.create_subscription(Int32, '/left_motor/encoder/delta', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Int32, '/right_motor/encoder/delta', self.right_encoder_callback, 10)

        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.target_ticks = 0
        self.driving = False
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.response = None  # Обязательно инициализируем response

    def left_encoder_callback(self, msg):
        if self.driving:
            self.left_encoder_ticks += msg.data
            self.check_ticks()

    def right_encoder_callback(self, msg):
        if self.driving:
            self.right_encoder_ticks += msg.data
            self.check_ticks()

    def check_ticks(self):
        if abs(self.left_encoder_ticks) >= abs(self.target_ticks) and abs(self.right_encoder_ticks) >= abs(self.target_ticks):
            self.stop_robot()
            self.get_logger().info("Target ticks reached, stopping")

            # Убедитесь, что response уже создан, чтобы завершить выполнение сервиса
            if self.response is not None:
                self.response.success = True
                self.response.message = "Movement completed successfully"
#                self.srv.success(self.response)
                self.driving = False

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)  # Останавливаем робота
        self.moving = False
        self.get_logger().info('Turn completed successfully')

    def drive_by_encoder_callback(self, request, response):
        self.get_logger().info(f"Received request to drive {request.ticks} ticks forward")

        # Сбрасываем счетчики энкодеров
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.target_ticks = request.ticks
        self.response = response  # Сохраняем response для использования в check_ticks

        # Включаем флаг движения
        self.driving = True

        # Отправляем команду двигателям для начала движения
        twist = Twist()
        twist.linear.x = 0.3
        self.cmd_vel_pub.publish(twist)

        return response  # Возвращаем response после начала движения
def main(args=None):
    rclpy.init(args=args)

    drive_by_encoder_server = DriveByEncoderServer()

    rclpy.spin(drive_by_encoder_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
