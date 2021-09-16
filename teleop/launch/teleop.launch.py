import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('teleop'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='teleop',
            namespace='teleop',
            executable='teleop_node',
            name='teleop_node',
            parameters = [config]
        ),
        Node(
            package='joy',
            namespace='joy',
            executable='joy_node',
            name='joy_node',
            parameters= [config]
        )
    ])