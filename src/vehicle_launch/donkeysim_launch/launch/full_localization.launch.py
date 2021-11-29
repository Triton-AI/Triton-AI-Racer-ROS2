from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('donkeysim_launch'),
        'param',
        'full_localization.yaml'
    )
    urdf_path = os.path.join(
        get_package_share_directory('donkeysim_launch'),
        'param', 'donkeycar.urdf'
    )
    with open(urdf_path, 'r') as infp:
        donkeycar_urdf = infp.read()

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

        # Node(
        #     package='particle_filter_py',
        #     executable='particle_filter_py_node',
        #     name='particle_filter_py_node',
        #     parameters=[
        #         config
        #     ]
        # ),

        Node(
            package='odometry_localizer',
            executable='odometry_localizer_node',
            name='odometry_localizer_node',
            output='screen',
            parameters=[
                config
            ]
        ),

        Node(
            package='transform_manager',
            executable='transform_manager_node',
            name='transform_manager_node',
            output='screen',
            parameters=[
                config
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'publish_frequency': 1.0,
                'ignore_timestamp': False,
                'use_tf_static': True,
                'robot_description': donkeycar_urdf,
            }]
        )
    ])