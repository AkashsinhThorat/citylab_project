from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    action_server_node = Node(
        package='robot_patrol',
        executable='action_server_node',
        name='action_server_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'service_server_node:=debug'],
    )

    return LaunchDescription([
        action_server_node,
    ])
