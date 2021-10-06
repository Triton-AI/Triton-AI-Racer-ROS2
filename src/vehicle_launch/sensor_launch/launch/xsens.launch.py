from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    ld.add_action(arg)

    parameters_file_path = Path(get_package_share_directory('sensor_launch'), 'param', 'xsens.yaml')
    xsens_mti_node = Node(
            package='bluespace_ai_xsens_mti_driver',
            namespace='xsens',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen',
            parameters=[parameters_file_path],
            arguments=[],
            remapping=[
                ("/imu/data", "imu_raw"),
            ]
            )
    ld.add_action(xsens_mti_node)

    return ld