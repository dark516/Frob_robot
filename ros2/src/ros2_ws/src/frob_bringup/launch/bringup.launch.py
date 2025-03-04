from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Узел ros2_arduino_bridge
    arduino_bridge = Node(
        package='ros2_arduino_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen'
    )

    #Lidar YDLIDAR X4
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
        )
    )

    #Imu mpu6050
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_mpu6050'), 'launch', 'ros2_mpu6050.launch.py')
        )
    )

    #Description
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('frob_description'), 'launch', 'description.launch.py')
        )
    )

    #Odometry
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('frob_odometry'), 'launch', 'odometry.launch.py')
        )
    )

    return LaunchDescription([
        arduino_bridge,
        lidar_launch,
        imu_launch,
        description_launch,
        odometry_launch
    ])
