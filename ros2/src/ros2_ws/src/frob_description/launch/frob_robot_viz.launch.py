from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Указываем фиксированное имя пакета
    description_package = "frob_description"
    
    # Получение URDF через xacro с динамическим поиском xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # Поиск исполняемого файла xacro
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "frob.xacro"]
            ),
            " ",
            "sim_gazebo_classic:=true",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Путь к конфигурации RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "frob.rviz"]
    )

    # Узел для публикации состояний сочленений
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    # Узел для публикации состояния робота
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Узел для запуска RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
