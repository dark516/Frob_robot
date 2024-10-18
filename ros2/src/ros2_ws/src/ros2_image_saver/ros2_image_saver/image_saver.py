import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import argparse

class ImageSaverNode(Node):
    def __init__(self, save_dir):
        super().__init__('image_saver_node')
        self.subscription = self.create_subscription(
            Image,
            '/ans_image',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.image_count = 0 
        self.save_dir = save_dir
        # Создаем папку по надобности
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def image_callback(self, msg):
        try:
            # ROS2 image to cv image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Сохраняем изображение в файл
            image_filename = os.path.join(self.save_dir, f'image_{self.image_count}.jpg')
            cv2.imwrite(image_filename, cv_image)
            self.get_logger().info(f'image save as {image_filename}')
            self.image_count += 1
        except Exception as e:
            self.get_logger().error(f'ERROR!!! {e}')

def main(args=None):
    # Парсер аргументов командной строки
    parser = argparse.ArgumentParser(description='Сохранение изображений из ROS2 топика.')
    parser.add_argument('--dir', type=str, default='images',
                        help='Папка для сохранения изображений (по умолчанию "images")')
    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = ImageSaverNode(cli_args.dir)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
