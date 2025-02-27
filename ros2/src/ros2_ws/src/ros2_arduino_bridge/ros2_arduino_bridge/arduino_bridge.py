from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
import rclpy
from serial import Serial, SerialException
from time import sleep
import sys
import glob
import uuid

from frob_interfaces.srv import Turn, Forward
from ros2_arduino_bridge.connection import ArduinoConnection

class Arduino_bridge(Node):
    MAX_LIN_SPEED = 0.4863  # [m/s]
    MAX_ANG_SPEED = 5.4     # [rad/s]

    def __init__(self, connection: ArduinoConnection):
        unique_id = str(uuid.uuid4())[:8]
        super().__init__(f'arduino_bridge_{unique_id}')
        self._connect = connection
        self._connected = True

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )
        self.left_delta_pub = self.create_publisher(Int32, 'left_motor/encoder/delta', 10)
        self.right_delta_pub = self.create_publisher(Int32, 'right_motor/encoder/delta', 10)

        self.left_speed_pub = self.create_publisher(Float32, 'left_motor/speed', 10)
        self.right_speed_pub = self.create_publisher(Float32, 'right_motor/speed', 10)   

        # Services
        self.rotate_srv = self.create_service(Turn, 'rotate_robot', self.handle_rotate_request)
        self.forward_srv = self.create_service(Forward, 'forward_robot', self.handle_forward_request)

        # State variables
        self.last_linear = None
        self.last_angular = None

        # Timers
        self.data_request_timer = self.create_timer(0.1, self.data)
        self.connection_check_timer = self.create_timer(1.0, self.check_connection)

    @classmethod
    def clamp_speed(cls, value, max_value):
        return max(min(value, max_value), -max_value)

    def handle_rotate_request(self, request, response):
        if not self._connected:
            self._print_error("Service call failed: No active connection!")
            response.success = False
            return response
            
        try:
            result = self._connect.turn_robot(request.angle, request.speed)
            response.success = bool(result)
            if result:
                self.get_logger().info(f"Rotated by {request.angle}° at {request.speed}% power")
        except SerialException as e:
            self._handle_disconnection()
            response.success = False
            self._print_error(f"Rotation failed: {str(e)}")
        return response

    def handle_forward_request(self, request, response):
        if not self._connected:
            self._print_error("Service call failed: No active connection!")
            response.success = False
            return response
            
        try:
            result = self._connect.go_dist(request.dist, request.speed)
            response.success = bool(result)
            if result:
                self.get_logger().info(f"Moved {request.dist}cm at {request.speed}% power")
        except SerialException as e:
            self._handle_disconnection()
            response.success = False
            self._print_error(f"Movement failed: {str(e)}")
        return response

    def cmd_vel_callback(self, msg):
        if not self._connected:
            return
            
        try:
            linear = float(self.clamp_speed(msg.linear.x, self.MAX_LIN_SPEED))
            angular = float(self.clamp_speed(msg.angular.z, self.MAX_ANG_SPEED))
            
            if linear != self.last_linear or angular != self.last_angular:
                self._connect.setSpeeds(linear, angular)
                self.last_linear = linear
                self.last_angular = angular
                self.get_logger().debug(f"New speeds: Lin={linear:.2f}m/s, Ang={angular:.2f}rad/s")
            
        except SerialException as e:
            self._handle_disconnection()

    def data(self):
        if not self._connected:
            return
            
        try:
            arduino_data = self._connect.get_data()
            self.left_delta_pub.publish(Int32(data=arduino_data.left_delta))
            self.right_delta_pub.publish(Int32(data=arduino_data.right_delta))
            self.left_speed_pub.publish(Float32(data=arduino_data.left_speed))
            self.right_speed_pub.publish(Float32(data=arduino_data.right_speed))
            print(f'left_delta {arduino_data.left_delta} right_delta {arduino_data.right_delta} left_speed {arduino_data.left_speed} left_speed {arduino_data.right_speed}')
        except SerialException as e:
            self._handle_disconnection()

    def check_connection(self):
        if not self._connected:
            self._try_reconnect()

    def _handle_disconnection(self):
        if self._connected:
            self._print_error("Connection lost! Check Arduino connection.")
            self._connected = False
            self._connect.close()

    def _try_reconnect(self):
        ports = self._find_arduino_ports()
        for port in ports:
            try:
                self.get_logger().info(f"Attempting connection to {port}...")
                connection = ArduinoConnection(Serial(port, 115200))
                sleep(2)
                
                if connection.is_arduino():
                    self._connect = connection
                    self._connected = True
                    self.get_logger().info(f"Successfully connected to Arduino at {port}")
                    return True
                else:
                    connection.close()
                    self.get_logger().warning(f"Invalid response from {port}")
                    
            except (SerialException, OSError) as e:
                self.get_logger().warning(f"Connection failed to {port}: {str(e)}")
                continue
            except Exception as e:
                self._print_error(f"Unexpected error: {str(e)}")
                continue
                
        self._print_error("No valid Arduino devices found!")
        return False

    def _find_arduino_ports(self):
        if sys.platform.startswith('win'):
            ports = [f'COM{i+1}' for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.usbmodem*') + glob.glob('/dev/tty.usbserial*')
        else:
            raise EnvironmentError('Unsupported OS')
        return ports

    def _print_error(self, message):
        self.get_logger().error('\033[1;31m' + message + '\033[0m')

    def shutdown(self):
        if self._connected:
            try:
                self._connect.close()
            except Exception as e:
                self._print_error(f"Connection closure error: {str(e)}")
        
        self.destroy_node()

def find_arduino_ports():
    if sys.platform.startswith('win'):
        ports = [f'COM{i+1}' for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.usbmodem*') + glob.glob('/dev/tty.usbserial*')
    else:
        raise EnvironmentError('Unsupported OS')
    return ports

def main(args=None):
    rclpy.init(args=args)
    arduino_bridge = None

    try:
        port = sys.argv[1] if len(sys.argv) > 1 else None
        ports = find_arduino_ports()

        if port:
            if port not in ports:
                print(f"\033[1;31mPort {port} not found!\033[0m")
                return
            ports = [port]

        for p in ports:
            try:
                print(f"\033[36mConnecting to {p}...\033[0m")
                connection = ArduinoConnection(Serial(p, 115200))
                sleep(2)
                
                if connection.is_arduino():
                    arduino_bridge = Arduino_bridge(connection)
                    print(f"\033[1;32mSuccessfully connected to Arduino at {p}\033[0m")
                    break
                else:
                    print(f"\033[33mDevice at {p} is not responding as Arduino\033[0m")
                    connection.close()
                    
            except Exception as e:
                print(f"\033[33mConnection to {p} failed: {str(e)}\033[0m")
                continue
        else:
            print("\033[1;31mNo valid Arduino devices found!\033[0m")
            return

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(arduino_bridge)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            arduino_bridge.shutdown()

    except Exception as e:
        print(f"\033[1;31mCritical error: {str(e)}\033[0m")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()