from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():

    waypoint_config = os.path.join(
        get_package_share_directory('waypoint_logger'),
        'params',
        'wp_logger_config.yaml'
        )
    wp_remappings = [
        ("/fix", "/nmea/fix"),
        ("/vel", "/nmea/vel")
    ]
       
    return LaunchDescription([
        Node(
            package='waypoint_logger',
            namespace='waypoint',
            executable='log_waypoints',
            name='log_node',
            parameters=[
                waypoint_config
            ],
            remappings=wp_remappings       
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_share_file(
                    package_name='sensor_launch',
                    file_name='launch/nmea.launch.py'
                ),
            ]),
        ),

    ])