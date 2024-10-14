import sys
import rclpy
from rclpy.node import Node
from frob_interfaces.srv import DriveByEncoder

class DriveByEncoderClient(Node):
    def __init__(self):
        super().__init__('drive_by_encoder_client')
        self.cli = self.create_client(DriveByEncoder, 'drive_by_encoder')

        # Ожидаем, пока сервис станет доступен
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = DriveByEncoder.Request()

    def send_request(self, ticks):
        self.request.ticks = ticks
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    client = DriveByEncoderClient()

    if len(sys.argv) != 2:
        print("Usage: ros2 run <package_name> <client_node> <ticks>")
        sys.exit(1)

    ticks = int(sys.argv[1])
    response = client.send_request(ticks)

    if response.success:
        client.get_logger().info(f"Drive succeeded: {response.message}")
    else:
        client.get_logger().info(f"Drive failed: {response.message}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
