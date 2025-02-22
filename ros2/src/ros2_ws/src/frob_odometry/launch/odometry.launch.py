import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Путь к файлу с параметрами
    params_file = os.path.join(
        get_package_share_directory('frob_odometry'),
        'config',
        'odometry_params.yaml'
    )

    return LaunchDescription([
        # Запуск ноды одометрии с параметрами
        Node(
            package='frob_odometry',
            executable='odometry_publisher',
            name='encoder_odometry',
            output='screen',
            parameters=[params_file]
        )
    ])
