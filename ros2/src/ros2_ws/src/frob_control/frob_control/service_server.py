import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians, pi, sqrt, atan2
from time import sleep
from frob_interfaces.srv import Turn, Drive  # Импортируем интерфейсы сервисов

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control_service')
        
        # Инициализация подписки на одометрию и публикации команд движения
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Создание сервисов
        self.rotate_service = self.create_service(Turn, 'rotate_robot', self.rotate_robot_callback)
        self.move_service = self.create_service(Drive, 'move_forward', self.move_forward_callback)
        
        # Хранение текущего состояния одометрии
        self.current_pose = None

    def rotate_robot_callback(self, request, response):
        twist = Twist()
        twist.angular.z = 0.4 if request.dir > 0 else -0.4
        # Пока робот не достигнет целевого угла
        self.publisher.publish(twist)
        sleep(request.time)
        # Останавливаем вращение
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
        response.success = True
        return response

    def move_forward_callback(self, request, response):
        twist = Twist()
        twist.linear.x = 0.5

        self.publisher.publish(twist)
        sleep(request.time)

        twist.linear.x = 0.0
        self.publisher.publish(twist)
        
        response.success = True
        return response

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
