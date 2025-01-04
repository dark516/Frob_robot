from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_path = get_package_share_directory('description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

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

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='pc_rviz',
            output='screen',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'display_slam.rviz')]
        )

    return LaunchDescription([
        robot_pub,
        joint_pub,
        rviz
    ])
