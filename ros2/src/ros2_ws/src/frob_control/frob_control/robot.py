import rclpy
from std_msgs.msg import Int32
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians, pi, sqrt, atan2
from time import sleep
from frob_interfaces.srv import RotateRobot, MoveForward  # Импортируем интерфейсы сервисов

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control_service')


        self.create_subscription(Int32, '/left_motor/encoder/delta', self.left_encoder_callback, 10)
        self.create_subscription(Int32, '/right_motor/encoder/delta', self.right_encoder_callback, 10)

        # Инициализация подписки на одометрию и публикации команд движения
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.left_encoder_delta = 0
        self.right_encoder_delta = 0

        # Создание сервисов
        self.rotate_service = self.create_service(RotateRobot, 'rotate_robot', self.rotate_robot_callback)
        self.move_service = self.create_service(MoveForward, 'move_forward', self.move_forward_callback)

        # Хранение текущего состояния одометрии
        self.current_pose = None

    def left_encoder_callback(self, msg):
        self.left_encoder_delta += msg.data

    def right_encoder_callback(self, msg):
        self.right_encoder_delta += msg.data

    def rotate_robot_callback(self, request, response):
        tick = request.tick  # Угол в радианах
        print(tick)
        twist = Twist()
        twist.angular.z = 0.4 if tick > 0 else -0.4

        # Пока робот не достигнет целевого угла
        while abs(self.left_encoder_delta) < tick and abs(self.right_encoder_delta) < tick:
            print(self.left_encoder_delta)
            self.publisher.publish(twist)
            sleep(0.1)
#            rclpy.spin_once(self)
        print("Как я тут")
        # Останавливаем вращение
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.left_encoder_delta = 0
        self.right_encoder_delta = 0
        response.success = True
        return response

    def move_forward_callback(self, request, response):
        """Сервис для движения робота на заданное расстояние"""
        if self.current_pose is None:
            response.success = False
            self.get_logger().error("Нет данных об одометрии!")
            return response

        # Текущее положение
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y

        target_distance = request.distance
        current_distance = 0.0

        # Команда на движение вперед
        twist = Twist()
        twist.linear.x = 0.2

        while current_distance < target_distance:
            self.publisher.publish(twist)
            sleep(0.1)
            # Вычисляем текущее пройденное расстояние
            current_distance = self.calculate_distance(start_x, start_y, self.current_pose.position.x, self.current_pose.position.y)

        # Останавливаем движение
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        
        response.success = True
        return response

    def calculate_distance(self, x1, y1, x2, y2):
        """Вычисление евклидова расстояния между двумя точками"""
        return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_yaw_from_quaternion(self, orientation):
        """Получаем угол поворота вокруг оси Z из кватерниона"""
        q_x = orientation.x
        q_y = orientation.y
        q_z = orientation.z
        q_w = orientation.w

        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        return atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Нормализуем угол в диапазон [-pi, pi]"""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def is_angle_reached(self, current_angle, target_angle):
        """Проверка, достиг ли робот целевого угла с небольшой погрешностью"""
        return abs(current_angle - target_angle) < 1


def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
