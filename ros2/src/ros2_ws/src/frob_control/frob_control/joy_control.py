import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        
        # Загрузка параметров
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_linear_speed', 0.5),
                ('max_angular_speed', 1.0),
                ('step_linear', 0.1),
                ('step_angular', 0.1),
                ('axis_linear', 1),
                ('axis_angular', 3),
                ('button_linear_up', 4),
                ('button_linear_down', 6),
                ('button_angular_up', 5),
                ('button_angular_down', 7),
                ('deadzone', 0.1),
                ('frequency', 20.0),
            ]
        )
        
        # Текущие состояния
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # История для подавления спама
        self.last_published_linear = 0.0
        self.last_published_angular = 0.0
        self.last_publish_time = self.get_clock().now()
        
        # Состояния кнопок
        self.prev_linear_up = 0
        self.prev_linear_down = 0
        self.prev_angular_up = 0
        self.prev_angular_down = 0
        
        # Инициализация коммуникаций
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info("Joystick control node started!")

    def joy_callback(self, msg):
        # Обработка линейной скорости (L1/L2)
        button_linear_up = msg.buttons[self.get_parameter('button_linear_up').value]
        button_linear_down = msg.buttons[self.get_parameter('button_linear_down').value]
        
        # L1: Увеличить линейную скорость
        if button_linear_up == 1 and self.prev_linear_up == 0:
            self.current_linear_speed = round(
                min(
                    self.current_linear_speed + self.get_parameter('step_linear').value,
                    self.get_parameter('max_linear_speed').value
                ),
                4
            )
        # L2: Уменьшить линейную скорость
        if button_linear_down == 1 and self.prev_linear_down == 0:
            self.current_linear_speed = round(
                max(
                    self.current_linear_speed - self.get_parameter('step_linear').value,
                    0.0
                ),
                4
            )
        self.prev_linear_up = button_linear_up
        self.prev_linear_down = button_linear_down
        
        # Обработка угловой скорости (R1/R2)
        button_angular_up = msg.buttons[self.get_parameter('button_angular_up').value]
        button_angular_down = msg.buttons[self.get_parameter('button_angular_down').value]
        
        # R1: Увеличить угловую скорость
        if button_angular_up == 1 and self.prev_angular_up == 0:
            self.current_angular_speed = round(
                min(
                    self.current_angular_speed + self.get_parameter('step_angular').value,
                    self.get_parameter('max_angular_speed').value
                ),
                4
            )
        # R2: Уменьшить угловую скорость
        if button_angular_down == 1 and self.prev_angular_down == 0:
            self.current_angular_speed = round(
                max(
                    self.current_angular_speed - self.get_parameter('step_angular').value,
                    0.0
                ),
                4
            )
        self.prev_angular_up = button_angular_up
        self.prev_angular_down = button_angular_down
        
        # Расчет скоростей по стикам
        axis_linear = msg.axes[self.get_parameter('axis_linear').value]
        axis_angular = msg.axes[self.get_parameter('axis_angular').value]
        deadzone = self.get_parameter('deadzone').value
        
        # Линейная составляющая
        if abs(axis_linear) < deadzone:
            linear = 0.0
        else:
            linear = self.current_linear_speed * (1.0 if axis_linear > 0 else -1.0)
            linear = round(linear, 4)
        
        # Угловая составляющая
        if abs(axis_angular) < deadzone:
            angular = 0.0
        else:
            angular = self.current_angular_speed * (-1.0 if axis_angular > 0 else 1.0)
            angular = round(angular, 4)
        
        # Фильтрация микро-значений
        epsilon = 1e-5
        self.current_linear = 0.0 if abs(linear) < epsilon else linear
        self.current_angular = 0.0 if abs(angular) < epsilon else angular
        
        # Попытка публикации
        self.try_publish()

    def try_publish(self):
        # Проверка изменений
        linear_changed = abs(self.current_linear - self.last_published_linear) > 1e-5
        angular_changed = abs(self.current_angular - self.last_published_angular) > 1e-5
        if not linear_changed and not angular_changed:
            return

        # Проверка частоты
        current_time = self.get_clock().now()
        time_since_last = (current_time - self.last_publish_time).nanoseconds * 1e-9
        min_interval = 1.0 / self.get_parameter('frequency').value
        if time_since_last < min_interval:
            return

        # Публикация
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.publisher.publish(twist)
        
        # Обновление истории
        self.last_published_linear = self.current_linear
        self.last_published_angular = self.current_angular
        self.last_publish_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
