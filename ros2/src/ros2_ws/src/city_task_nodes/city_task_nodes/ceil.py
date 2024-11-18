import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist
import time

class CellDefinitionNode(Node):
    def __init__(self):
        super().__init__('cell_defenition')

        self.target_distance = 0.5  # Целевое расстояние до стены (в метрах)
        self.distance_subscriber = self.create_subscription(
            Float32,
            '/left_distance',
            self.distance_callback,
            10
        )
        self.ceil = 0
        self.update_flag = 1
        self.current_distance = None
        self.ceil_publisher = self.create_publisher(Int32, '/curr_ceil', 10)

    def distance_callback(self, msg):
        # Получаем текущее значение расстояния
        self.current_distance = msg.data
        self.update_ceil()

    def update_ceil(self):
        if self.current_distance is None or self.current_distance == 0:
            return

        if self.current_distance >= self.target_distance * 2 and self.update_flag == 1:
            self.ceil += 1
            self.ceil_publisher.publish(Int32(data=self.ceil))
            self.update_flag = 0
        else:
            self.update_flag = 1


def main(args=None):
    rclpy.init(args=args)
    node = CellDefinitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
