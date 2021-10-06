from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulator time'
    )

    imu_topic = LaunchConfiguration('imu_topic')

    imu_topic_cmd = DeclareLaunchArgument(
        'imu_topic',
        default_value='/xsens/imu_raw',
        description='Raw IMU topic'
    )

    odom_topic = LaunchConfiguration('odom_topic')

    odom_topic_cmd = DeclareLaunchArgument(
        'odom_topic',
        default_value='/nmea/vel',
        description='Raw Odometry topic'
    )

    gps_topic = LaunchConfiguration('gps_topic')

    gps_topic_cmd = DeclareLaunchArgument(
        'gps_topic',
        default_value='/nmea/fix',
        description='Raw GPS topic'
    )


    localization_dir = get_package_share_directory('sensor_launch')

    parameters_file_path = os.path.join(
        localization_dir,
        'param', 
        'localization.yaml'
    )

    rl_local_ekf = {
        'odometry/filtered': '/odometry/local'
    }

    rl_global_ekf = {
        'odometry/filtered': '/odometry/global',
        'use_sim_time': use_sim_time,
        'odom0': odom_topic,
        'imu0': imu_topic
    }

    rl_navsat = {
        'imu': imu_topic,
        'gps/fix': gps_topic,
        'gps/filtered': '/gps/filtered',
        'odometry/gps': '/odometry/filtered_gps',
        'odometry/filtered': '/odometry/global'
    }

    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_filter_node',
        parameters = [parameters_file_path, {'use_sim_time': use_sim_time, 'odom0': odom_topic, 'imu0':imu_topic}],
        remappings = rl_local_ekf.items()
    )
    
    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_filter_node',
        parameters = [parameters_file_path],
        remappings = rl_global_ekf.items()
    )
    
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        parameters = [parameters_file_path, {'use_sim_time': use_sim_time}],
        remappings = rl_navsat.items()
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)
    ld.add_action(imu_topic_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(gps_topic_cmd)
    ld.add_action(local_ekf_node)
    ld.add_action(global_ekf_node)
    ld.add_action(navsat_node)

    return ld