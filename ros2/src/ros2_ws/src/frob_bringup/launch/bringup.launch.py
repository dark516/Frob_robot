from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Путь к пакету с описанием робота
    pkg_path = get_package_share_directory('description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    # Аргумент для задания порта Arduino
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyUSB1',
        description='Порт, к которому подключен Arduino'
    )

    # Узел robot_state_publisher
    robot_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_pub',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # Узел joint_state_publisher
    joint_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_pub',
        output='screen'
    )

    # Узел ros2_arduino_bridge
    arduino_bridge = Node(
        package='ros2_arduino_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        arguments=[LaunchConfiguration('arduino_port')]
    )

    # Узел odometry_publisher
    odometry_publisher = Node(
        package='frob_odometry',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen'
    )

    # Включение launch файла YDLIDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
        )
    )

    return LaunchDescription([
        arduino_port_arg,
        robot_pub,
        joint_pub,
        arduino_bridge,
        odometry_publisher,
        lidar_launch
    ])

