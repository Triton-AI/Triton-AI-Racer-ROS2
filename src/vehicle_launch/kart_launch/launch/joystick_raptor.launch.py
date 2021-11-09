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
        "joystick_raptor.yaml"
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_share_file(
                        package_name='ros2_socketcan',
                        file_name='launch/socket_can_receiver.launch.py'
                    ),
                ]),
                launch_arguments={'interface': 'can0'}.items(),
            ),


            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_share_file(
                        package_name='ros2_socketcan',
                        file_name='launch/socket_can_sender.launch.py'

                    )
                ]),
                launch_arguments={'interface': 'can0'}.items()
            ),

            Node(
                package='raptor_dbw_joystick',
                executable='raptor_dbw_joystick_node',
                output='screen',
                namespace='raptor_dbw_interface',
                parameters=[params_file_path]
            ),

            Node(
                package='joy',
                executable='joy_node',
                output='screen',
                namespace='raptor_dbw_interface',
                parameters=[params_file_path]
            )
        ])
