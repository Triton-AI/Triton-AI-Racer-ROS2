from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    
    donkeysim_client_node_config = os.path.join(
        get_package_share_directory('donkeysim_tai_interface'),
        'param',
        'donkeysim_node.yaml'
    )

    config_file = DeclareLaunchArgument(
        "config", default_value=donkeysim_client_node_config, description="Config File for DonkeySim"
    )

    return LaunchDescription([
        config_file,
        Node(
            package='donkeysim_tai_interface',
            executable='donkeysim_client_node',
            name='donkeysim_client_node',
            output='screen',
            emulate_tty=True,
            parameters=[donkeysim_client_node_config]           
        )

    ])