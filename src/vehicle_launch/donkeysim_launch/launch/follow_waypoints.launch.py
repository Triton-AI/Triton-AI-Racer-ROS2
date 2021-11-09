from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('donkeysim_launch'),
        'param',
        'follow_waypoints.yaml'
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
            package='waypoint_following',
            executable='follow_wp',
            name='follow_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                config
            ]
        )

    ])
