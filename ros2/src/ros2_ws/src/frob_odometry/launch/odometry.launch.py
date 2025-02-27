import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Пути к конфигурационным файлам
    pkg_path = get_package_share_directory('frob_odometry')
    params_odom = os.path.join(pkg_path, 'config', 'odometry_params.yaml')
    params_ekf = os.path.join(pkg_path, 'config', 'ekf.yaml')

    return LaunchDescription([
        # Нода энкодерной одометрии
        Node(
            package='frob_odometry',
            executable='odometry_publisher',
            name='encoder_odometry',
            output='screen',
            parameters=[params_odom]
        ),
        # EKF-фильтр для слияния данных
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params_ekf]
        ),

        # Статический трансформ для IMU (если нужно)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_publisher',
            arguments=['0.0', '0.0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
        )
    ])
