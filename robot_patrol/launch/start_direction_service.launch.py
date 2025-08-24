from launch import LaunchDescription
from launch_ros.actions import Node

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
