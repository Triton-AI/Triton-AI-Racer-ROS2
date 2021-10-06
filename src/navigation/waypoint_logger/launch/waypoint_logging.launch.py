from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    waypoint_config = os.path.join(
        get_package_share_directory('waypoint_logger'),
        'params',
        'wp_logger_config.yaml'
        )
    return LaunchDescription([
        Node(
            package='waypoint_logger',
            namespace='waypoint',
            executable='log_waypoints',
            name='log_node',
            parameters=[
                waypoint_config
            ],          
        )

    ])