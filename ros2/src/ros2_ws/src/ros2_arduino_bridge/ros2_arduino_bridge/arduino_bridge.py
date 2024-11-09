from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import rclpy
from serial import Serial
from time import sleep
import rclpy
import sys

from frob_interfaces.srv import Turn
from ros2_arduino_bridge.connection import ArduinoConnection

class Arduino_bridge(Node):
    MAX_MOTOR_SPEED_TICKS = 10
    """Максимальное значение скорости для моторов"""

    @classmethod
    def clamp_motor_speed(cls, motor_speed):
        return max(min(int(motor_speed * cls.MAX_MOTOR_SPEED_TICKS), cls.MAX_MOTOR_SPEED_TICKS), -cls.MAX_MOTOR_SPEED_TICKS)

    def __init__(self, connection: ArduinoConnection):
        super().__init__('arduino_bridge')

        self._connect = connection

        self.srv = self.create_service(Turn, 'rotate_robot', self.handle_rotate_request)
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10)

        self.left_speed_pub = self.create_publisher(Int32, 'left_motor/encoder/delta', 10)
        self.right_speed_pub = self.create_publisher(Int32, 'right_motor/encoder/delta', 10)

        self.wheel_base = 0.175  # Расстояние между колесами в метрах
        self.wheel_radius = 0.0325  # Радиус колес в метрах

        self.last_left_motor_speed = None
        self.last_right_motor_speed = None
        
        self.data_request_timer = self.create_timer(0.05, self.data)  # Запрос данных раз в 1 секунду

    def handle_rotate_request(self, request, response):
        print(request.angle)
        self._connect.turn_robot(request.angle)
        response.success = True
        return response
    
    def cmd_vel_callback(self, msg):
        # Извлечение линейной и угловой скорости из сообщения Twist
        linear = msg.linear.x
        angular = msg.angular.z

        left_motor_speed = (linear - angular * self.wheel_base / 2) / self.wheel_radius   #TODO ПОСТАВИТЬ ЭТИ 2 СТРОЧКИ, когда робот будет ехать с реальной скоростью
        right_motor_speed = (linear + angular * self.wheel_base / 2) / self.wheel_radius

        # Ограничиваем скорость в пределах [-13, 13]
        left_motor_speed = self.clamp_motor_speed(left_motor_speed)
        right_motor_speed = self.clamp_motor_speed(right_motor_speed)

        if left_motor_speed != self.last_left_motor_speed or right_motor_speed != self.last_right_motor_speed:
            self._connect.setSpeeds(left_motor_speed, right_motor_speed)
            self.last_left_motor_speed = left_motor_speed
            self.last_right_motor_speed = right_motor_speed

    def data(self):
        #Отправка данных
        arduino_data = self._connect.get_data()
        self.left_speed_pub.publish(Int32(data=arduino_data.left))
        self.right_speed_pub.publish(Int32(data=arduino_data.right))

    def shutdown(self) -> None:
        self._connect.close()
        super().shutdown()


def main(args=None):
    rclpy.init(args=args)

    serial_port = '/dev/ttyACM0'  # Значение по умолчанию
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]

    arduino_bridge = Arduino_bridge(ArduinoConnection(Serial(serial_port, 115200)))
    sleep(2)  # TODO

    rclpy.spin(arduino_bridge)

    arduino_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#self.publisher1.publish(Int32(data=number_for_topic1))
#self._connect.setSpeeds(left_motor_speed, right_motor_speed)

