import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Параметры PID-регулятора
        self.target_distance = 0.5  # Целевое расстояние до стены (в метрах)
        self.kp = 0.03             # Пропорциональный коэффициент
        self.ki = 0.008             # Интегральный коэффициент
        self.kd = 0.004             # Дифференциальный коэффициент

        # Инициализация переменных
        self.current_distance = None
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.integral_max = 0.05      # Ограничение для интегральной части
        self.previous_time = time.time()  # Время предыдущего обновления ошибки

        # Подписка на топик расстояния
        self.distance_subscriber = self.create_subscription(
            Float32,
            '/left_distance',
            self.distance_callback,
            10
        )

        # Публикация скорости
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def distance_callback(self, msg):
        # Получаем текущее значение расстояния
        self.current_distance = msg.data

        # Вызываем функцию обновления скорости
        self.update_velocity()

    def update_velocity(self):
        # Проверка, что расстояние получено
        if self.current_distance is None:
            return

        # Вычисляем ошибку и текущее время
        error = self.target_distance - self.current_distance
        current_time = time.time()
        delta_time = current_time - self.previous_time

        if delta_time <= 0.0:
            return  # Избегаем деления на ноль

        # Пропорциональная составляющая
        proportional = self.kp * error

        # Интегральная составляющая
        self.integral_error += error * delta_time
        self.integral_error = max(min(self.integral_error, self.integral_max), -self.integral_max)
        integral = self.ki * self.integral_error

        # Дифференциальная составляющая
        derivative = self.kd * (error - self.previous_error) / delta_time

        # Расчет управляющего воздействия
        angular_z = proportional + integral + derivative

        # Сохраняем текущее значение ошибки и времени для следующего цикла
        self.previous_error = error
        self.previous_time = current_time

        # Создаем и публикуем сообщение Twist
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.01    # Постоянная линейная скорость
        cmd_msg.angular.z = angular_z

        # Публикация сообщения
        self.cmd_vel_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
