import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Подписка на топик /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Настройка серийного порта
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f'Ошибка при открытии серийного порта: {e}')
            raise

        # Максимальное значение скорости для моторов
        self.max_motor_speed = 13

        # Хранение предыдущих скоростей для сравнения
        self.last_left_motor_speed = None
        self.last_right_motor_speed = None

    def cmd_vel_callback(self, msg):
        # Извлечение линейной и угловой скорости из сообщения Twist
        linear = msg.linear.x
        angular = msg.angular.z

        # Рассчитываем скорость для каждого мотора
        left_motor_speed = linear - angular
        right_motor_speed = linear + angular

        # Ограничиваем скорость в пределах [-13, 13]
        left_motor_speed = max(min(int(left_motor_speed * self.max_motor_speed), self.max_motor_speed), -self.max_motor_speed)
        right_motor_speed = max(min(int(right_motor_speed * self.max_motor_speed), self.max_motor_speed), -self.max_motor_speed)

        # Проверяем, отличаются ли текущие значения от предыдущих
        if (left_motor_speed != self.last_left_motor_speed or right_motor_speed != self.last_right_motor_speed):
            self.get_logger().info(f'Left Motor: {left_motor_speed}, Right Motor: {right_motor_speed}')
            
            # Упаковываем данные в struct и отправляем по serial
            data = struct.pack('hh', left_motor_speed, right_motor_speed)
            try:
                self.serial_port.write(data)
            except serial.SerialException as e:
                self.get_logger().error(f'Ошибка при отправке данных по серийному порту: {e}')

            # Обновляем предыдущие значения скоростей
            self.last_left_motor_speed = left_motor_speed
            self.last_right_motor_speed = right_motor_speed

    def destroy_node(self):
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    rclpy.spin(motor_controller)

    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
