import rclpy
from rclpy.node import Node
from frob_interfaces.srv import Turn, Drive

class SingleRequestClient(Node):

    def __init__(self):
        super().__init__('single_request_client')
        self.rotate_client = self.create_client(Turn, 'rotate_robot')
        self.forward_client = self.create_client(Drive, 'move_forward')

        # Ожидание доступности сервиса
        while not self.rotate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service 1...')

        while not self.forward_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service 2...')

        # Инициализируем запрос
        self.rot = Turn.Request()
        self.mov = Drive.Request()

    def rotate_robot(self, time, dir):
        self.rot.time = time
        self.rot.dir = dir
        future = self.rotate_client.call_async(self.rot)
        rclpy.spin_until_future_complete(self, future)

    def forward_robot(self, time):
        self.mov.time = time
        self.future = self.forward_client.call_async(self.mov)
        rclpy.spin_until_future_complete(self, self.future)


def main(args=None):
    rclpy.init(args=args)

    client = SingleRequestClient()

    # Пример значений для запроса
#    client.rotate_robot(a)
    # Отправка запроса
    #client.forward_robot(7.0)
    client.rotate_robot(1.08, True)
#    client.forward_robot(5.6)
    # Завершение работы клиента
    rclpy.shutdown()

if __name__ == '__main__':
    main()
