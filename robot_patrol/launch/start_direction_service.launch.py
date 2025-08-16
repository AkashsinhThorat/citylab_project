from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():

    direction_server_node = Node(
        package='robot_patrol',
        executable='service_server_node',
        name='service_server_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'service_server_node:=debug'],
    )

    return LaunchDescription([
        direction_server_node,
    ])
