import rclpy
from rclpy.node import Node
from frob_interfaces.srv import TurnByEncoder, DriveByEncoder

class MultiServiceClient(Node):
    def __init__(self):
        super().__init__('multi_service_client')
        # Создание клиентов для обоих сервисов
        self.drive_client = self.create_client(DriveByEncoder, 'drive_by_encoder')

        # Ожидаем, пока сервис станет доступен
        while not self.drive_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

   #     self.turn_client = self.create_client(TurnByEncoder, 'turn')
    #    while not self.turn_client.wait_for_service(timeout_sec=1.0):
     #        self.get_logger().info('Service not available, waiting again...')

        # Пример использования сервисов
        self.send_drive_request(2300)  # Движение вперед на 200 тиков
     #   self.send_turn_request(186)    # Поворот на 90 градусов

    def send_drive_request(self, ticks):
        request = DriveByEncoder.Request()
        request.ticks = ticks
        self.get_logger().info(f'Отправка запроса на движение на {ticks} тиков вперед...')
        
        # Отправка запроса и получение ответа
        future = self.drive_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Ответ от drive_forward: {future.result().success}')
        else:
            self.get_logger().error('Ошибка при вызове сервиса drive_forward.')

    def send_turn_request(self, angle):
        request = TurnByEncoder.Request()
        request.angle = angle
        self.get_logger().info(f'Отправка запроса на поворот на {angle} градусов...')
        
        # Отправка запроса и получение ответа
        future = self.turn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Ответ от turn: {future.result().success}')
        else:
            self.get_logger().error('Ошибка при вызове сервиса turn.')

def main(args=None):
    rclpy.init(args=args)
    multi_service_client = MultiServiceClient()
    rclpy.spin(multi_service_client)
    multi_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
