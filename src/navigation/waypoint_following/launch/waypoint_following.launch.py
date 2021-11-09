from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    waypoint_config = os.path.join(
        get_package_share_directory('waypoint_following'),
        'params',
        'wp_follow_config.yaml'
        )
    return LaunchDescription([
        Node(
            package='waypoint_following',
            namespace='waypoint',
            executable='follow_wp',
            name='follow_node',
            parameters=[
                waypoint_config
            ]           
        )

    ])