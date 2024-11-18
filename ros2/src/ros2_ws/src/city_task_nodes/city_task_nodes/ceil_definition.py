import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import time

class DistanceCounterNode(Node):
    def __init__(self):
        super().__init__('distance_counter_node')

        # Параметры
        self.target_distance = 2.0  # Пороговое расстояние в метрах
        self.curr_ceil = 0
        self.prev_large_distance = None
        self.transition_confirmations = 0
        self.confirmation_threshold = 3  # Количество подтверждений для изменения состояния
        self.min_transition_interval = 2.0  # Минимальное время между переходами (в секундах)
        self.last_transition_time = time.time()

        # Подписка на топик /left_distance
        self.subscription = self.create_subscription(
            Float32,
            '/left_distance',
            self.distance_callback,
            10
        )

        # Публикация в топик /curr_ceil
        self.publisher = self.create_publisher(Int32, '/curr_ceil', 10)

    def distance_callback(self, msg):
        current_distance = msg.data

        # Определяем, является ли текущее расстояние "большим"
        is_large_distance = current_distance > self.target_distance or current_distance == 0.0

        # Если это первый вызов callback, инициализируем предыдущее состояние
        if self.prev_large_distance is None:
            self.prev_large_distance = is_large_distance
            return

        # Проверка устойчивого перехода (фильтрация)
        if is_large_distance != self.prev_large_distance:
            self.transition_confirmations += 1
        else:
            self.transition_confirmations = 0

        # Если переход подтверждён несколько раз подряд
        if self.transition_confirmations >= self.confirmation_threshold:
            # Проверка минимального интервала между переходами
            current_time = time.time()
            if current_time - self.last_transition_time >= self.min_transition_interval:
                # Увеличиваем счётчик и публикуем значение
                self.curr_ceil += 1
                self.get_logger().info(f'Переход в относительную клетку №{self.curr_ceil}')

                ceil_msg = Int32()
                ceil_msg.data = self.curr_ceil
                self.publisher.publish(ceil_msg)

                # Обновляем время последнего перехода
                self.last_transition_time = current_time

            # Обновляем предыдущее состояние и сбрасываем подтверждения
            self.prev_large_distance = is_large_distance
            self.transition_confirmations = 0

def main(args=None):
    rclpy.init(args=args)
    node = DistanceCounterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

