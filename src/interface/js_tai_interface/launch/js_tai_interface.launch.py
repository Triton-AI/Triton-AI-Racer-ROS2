from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    js_tai_interface_node_config = os.path.join(
        get_package_share_directory('js_tai_interface'),
        'param',
        'js_tai_interface.yaml'
        )
    return LaunchDescription([
        Node(
            package='js_tai_interface',
            executable='js_tai_interface_node',
            name='js_tai_interface_node',
            parameters=[
                js_tai_interface_node_config
            ]           
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        )

    ])