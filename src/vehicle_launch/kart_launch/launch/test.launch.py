from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from launch_ros.remap_rule_type import RemapRule


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    params_file_path = os.path.join(
        get_package_share_directory('kart_launch'),
        "params",
        "test.yaml"
    )

    # make sure the dbc file gets installed with the launch file
    dbc_file_path = get_package_share_directory('raptor_dbw_can') + \
        '/launch/New_Eagle_DBW_3.3.542.dbc'

    return LaunchDescription(
        [
            Node(
                package='raptor_dbw_can',
                executable='raptor_dbw_can_node',
                output='screen',
                parameters=[
                    {'dbw_dbc_file': dbc_file_path},
                    params_file_path
                ],
                namespace='raptor_dbw_interface',
                remappings=[
                    ("can_rx", "/to_can_bus"),
                    ("can_tx", "/from_can_bus"),
                ]

            ),

            Node(
                package='raptor_dbw_joystick',
                executable='raptor_dbw_joystick_node',
                output='screen',
                namespace='raptor_dbw_interface',
                parameters=[params_file_path],
                remappings=[
                    ("accelerator_pedal_cmd", "/raptor_tai_interface/accelerator_pedal_cmd_from_raptor"),
                    ("brake_cmd", "/raptor_tai_interface/brake_cmd_from_raptor"),
                    ("steering_cmd", "/raptor_tai_interface/steering_cmd_from_raptor"),
                ]
            ),

            Node(
                package='joy',
                executable='joy_node',
                output='screen',
                namespace='raptor_dbw_interface',
                parameters=[params_file_path]
            ),

            Node(
                package='raptor_tai_interface',
                executable='raptor_tai_interface_node',
                parameters=[params_file_path],
                namespace='raptor_tai_interface',
                remappings=[
                    ("accelerator_pedal_cmd_to_raptor", "/raptor_dbw_interface/accelerator_pedal_cmd"),
                    ("brake_cmd_to_raptor", "/raptor_dbw_interface/brake_cmd"),
                    ("steering_cmd_to_raptor", "/raptor_dbw_interface/steering_cmd"),
                    ("vehicle_cmd_to_raptor", "vehicle_cmd_from_raptor")
                ]
            )
            
        ])