from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot_patrol'),
        'rviz',
        'robot_patrol_config.rviz'  
    ])

    action_server_node = Node(
        package='robot_patrol',
        executable='action_server_node',
        name='action_server_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'service_server_node:=debug'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        action_server_node,
        rviz_node,
    ])
