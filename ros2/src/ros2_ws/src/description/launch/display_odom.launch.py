from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_path = get_package_share_directory('description')

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='pc_rviz',
            output='screen',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'display_odom.rviz')]
        )
    
    return LaunchDescription([
        rviz
    ])
