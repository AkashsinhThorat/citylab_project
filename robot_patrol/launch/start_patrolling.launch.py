from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='patrol_node',
            output='screen',
            emulate_tty = True,
            arguments=[
                '--ros-args',
                '--log-level', 'patrol_node:=debug'  # show RCLCPP_DEBUG(_THROTTLE)
            ],
        ),
    ])