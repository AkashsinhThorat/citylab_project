from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    test_service_node = Node(
        package='robot_patrol',
        executable='test_service_node',
        name='test_service_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'service_server_node:=debug'],
    )

    return LaunchDescription([
        test_service_node,
    ])
