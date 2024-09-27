import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ViewCompressedImage(Node):
    def __init__(self):
        super().__init__('pi_image_compressed_viewer')
        self.bridge = CvBridge()
        self.create_subscription(CompressedImage, 
                                 '/camera/image/compressed', 
                                 self.image_callback, 
                                 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            cv2.imshow('PI Camera View', cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(f'Could not convert image: {e}')


def main(args=None):
    rclpy.init(args=args)
    viewer = ViewCompressedImage()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
