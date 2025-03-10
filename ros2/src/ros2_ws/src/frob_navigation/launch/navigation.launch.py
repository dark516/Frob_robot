from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    map_file = os.path.join(
        get_package_share_directory('frob_navigation'),
        'maps',
        'map.yaml'
    )

    nav2_params_file = os.path.join(
        get_package_share_directory('frob_navigation'),
        'config',
        'nav2_params.yaml'
    )

    # Указываем путь к launch-файлу nav2_bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'), 
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file
        }.items()
    )

    return LaunchDescription([nav2_bringup_launch])
