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
            package='ros2_razor_imu',
            executable='imu_node',
            name='imu_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                os.path.join(param_folder, 'driver/imu.yaml')
            ]
        ),

    ])
