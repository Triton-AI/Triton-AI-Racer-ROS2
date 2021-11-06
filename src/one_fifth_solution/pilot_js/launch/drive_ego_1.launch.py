from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    pilot_js_config = os.path.join(
        get_package_share_directory('pilot_js'),
        'params',
        'pilot_js_config.yaml'
        )
    return LaunchDescription([
        Node(
            package='joy',
            namespace='joy',
            executable='joy_node',
            name='js_node'
            
        ),

        Node(
            package='pilot_js',
            namespace='pilot_js',
            executable='pilot_js_node',
            name='pilot_js_node',
            parameters= [pilot_js_config],
            remappings=[
                ("/lgsvl/vehicle_control_cmd", "/lgsvl/ego_1/vehicle_control_cmd")
            ]
        ), 

    ])