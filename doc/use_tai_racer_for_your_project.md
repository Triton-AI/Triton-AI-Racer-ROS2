# Using Triton AI Racer for Your Project

## Install

Follow [Develop Triton AI Racer Packages on Host PC](install_host_pc.md) to install your working ROS2 environment for the development.

## Familarize Yourself with Triton AI Racer Repository

Digging into the `src` folder, you will see folders containing different groups of packages. The `src/interface` folder contains interfaces to different platforms. Aong them, `tai_interface` defines the custom control message that all vehicle interfaces are expecting (`tai_interface/msg/VehicleControl`). If you are working on a ROS2 node that would publish control, then this is the message definition to use. 

In `src/vehicle_launch` you will see packages containing example launch files for each platform.

In `deploy` folder you will see packaged, dockerized solution for deploying onto different platforms.

## Example: On a 1/10 Vehicle

Let's start with an example of assembling a drivable platform for a 1/10 vehicle. Instead of using self-driving algorithms, we will use a joystick to teleop the car.

The goal is to produce a launch file containing all the necessary packages to get the car rolling.

```python
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
```

The nodes are listed on the order that the control signal is propergated. First, the joystick node produces a `sensor_msgs/msg/Joy`, which is fed into the joystick-tai interface node that produces a `tai_interface/msg/VehicleControl`. This topic is subscribed by the vesc-tai interface nodes which converts it to corresponding `std_msgs/msg/Float64` messages that the vesc driver node is expecting, who finally translates them into hardware messages and sends them off to the VESC board over serial.

In this example, through two interfaces that both use `VehicleControl`, we manage to use a joystick to control a VESC. Now imagine instead of a joystick, you plug in a node that does self-driving algorithms, and outputs `VehicleControl`. The vesc-tai interface will happliy take that and handle the hardware complications from there.
