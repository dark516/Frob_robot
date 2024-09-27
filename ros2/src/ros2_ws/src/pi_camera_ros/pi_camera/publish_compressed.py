import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('pi_image_compressed_publisher')
        self._publisher = self.create_publisher(CompressedImage, '/camera/image/compressed', 1)

        time_in_seconds = 0.05
        self.timer = self.create_timer(time_in_seconds, self.frame_publishing_callback)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.get_logger().info('Publishing compressed image...')

    def frame_publishing_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
            self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    image_publisher = CompressedImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
