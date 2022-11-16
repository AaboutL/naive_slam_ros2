from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='feature_tracking',
            namespace='feature_tracking',
            executable='feature_tracking_node',
            name='feature_tracking_node',
            arguments=['--ros-args', '--log-level', 'INFO'],
            # emulate_tty=True,
            # output='screen'
        ),
        Node(
            package='state_estimation',
            namespace='state_estimation',
            executable='state_estimation_node',
            name='state_estimation_node',
            arguments=['--ros-args', '--log-level', 'INFO'],
            emulate_tty=True,
            output='screen',
            # prefix=['gdb -ex run --args']
        )
    ])
