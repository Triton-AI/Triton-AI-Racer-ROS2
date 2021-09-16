from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop',
            namespace='teleop',
            executable='teleop_node',
            name='teleop_node'
        ),
        Node(
            package='joy',
            namespace='joy',
            executable='joy_node',
            name='joy_node'
        )
    ])