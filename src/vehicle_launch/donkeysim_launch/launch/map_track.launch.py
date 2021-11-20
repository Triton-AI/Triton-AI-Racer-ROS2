from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('donkeysim_launch'),
        'param',
        'map_track.yaml'
    )
    return LaunchDescription([
        Node(
            package='donkeysim_tai_interface',
            executable='donkeysim_client_node',
            name='donkeysim_client_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                config
            ]
        ),

        Node(
            package='js_tai_interface',
            executable='js_tai_interface_node',
            name='js_tai_interface_node',
            parameters=[
                config
            ]
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                config
            ]
        ),

        Node(
            package='simple_mapping',
            executable='simple_mapping_node',
            name='simple_mapping_node',
            parameters=[
                config
            ]
        ),

    ])