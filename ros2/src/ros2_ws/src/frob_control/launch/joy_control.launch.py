from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Путь к конфигу (исправлено!)
    config_path = os.path.join(
        get_package_share_directory('frob_control'),
        'config',
        'joy_config.yaml'  # Убедитесь, что файл существует
    )

    return LaunchDescription([
        Node(
            package='frob_control',
            executable='joy_control',
            name='joy_control',
            output='screen',
            parameters=[config_path]  # Передаем путь напрямую
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        )
    ])
