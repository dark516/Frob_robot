from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration, NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('frob_navigation'), 'config', 'slam_toolbox_params.yaml')

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true')
    declare_use_lifecycle_manager = DeclareLaunchArgument('use_lifecycle_manager', default_value='false')
    declare_use_sim_time_argument = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_slam_params_file_cmd = DeclareLaunchArgument('slam_params_file', default_value=config_path)

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[slam_params_file, {'use_lifecycle_manager': use_lifecycle_manager, 'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='pc_slam',
        output='screen',
        namespace=''
    )

    configure_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
          transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node, start_state="configuring", goal_state="inactive",
            entities=[
                LogInfo(msg="SLAM STARTED"),
                EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node), transition_id=Transition.TRANSITION_ACTIVATE))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)


    return ld

