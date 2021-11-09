import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('hri-raptor-joy-conversion')
    config_file = os.path.join(pkg_dir, 'config', 'conversion.yaml')

    joy_sub = LaunchConfiguration('joy_input_topic')

    joy_sub_cmd = DeclareLaunchArgument(
        'joy_input_topic',
        default_value='/joy_sub',
        description='Input Joy Topic'
    )

    joy_pub = LaunchConfiguration('joy_output_topic')

    joy_pub_cmd = DeclareLaunchArgument(
        'joy_output_topic',
        default_value='/joy_pub',
        description='Outputted Joy topic'
    )

    remappings_log = {
        '/joy_sub': joy_sub,
        '/joy_pub': joy_pub,
    }

    conversion_node = Node(
        package='hri-raptor-joy-conversion',
        executable='conversion_node',
        name='hri_raptor_conversion_node',
        output='screen',
        parameters=[config_file],
        remappings=remappings_log.items()
    )

    ld = LaunchDescription()
    ld.add_action(joy_sub_cmd)
    ld.add_action(joy_pub_cmd)
    ld.add_action(conversion_node)

    return ld
