from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    param_folder = os.path.join(
        get_package_share_directory('model_190_launch'),
        'param',
    )
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                os.path.join(param_folder, 'src/joystick.yaml')
            ]
        ),

        Node(
            package='js_tai_interface',
            executable='js_tai_interface_node',
            name='js_tai_interface_node',
            parameters=[
                os.path.join(param_folder, 'js_tai_interface.yaml')
            ]
        ),

        Node(
            package='vesc_tai_interface',
            executable='vesc_tai_interface_node',
            name='vesc_tai_interface_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                os.path.join(param_folder, 'interface/vesc_tai_interface.yaml')
            ]
        ),

        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                os.path.join(param_folder, 'driver/vesc.yaml')
            ]
        )
    ])
