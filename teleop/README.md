# ROS Node for converting joystick messages to lgsvl commands

The teleop package contains teleop_node, a node that converts joystick messages from the [joy package](http://wiki.ros.org/joy) and converts them to messages to 
work with the [lgsvl simulator](https://www.svlsimulator.com/). 

## Published Topics

* lgsvl/vehicle_control_cmd ([lgsvl_msgs/VehicleControlData](https://github.com/lgsvl/lgsvl_msgs/blob/master/msg/VehicleControlData.msg)): outputs control commands for vehicle.

## Subscribed Topics

* joy/joy ([sensor_msgs/Joy](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)): reads commands from the joystick using joy package

## Services

* teleop/estop [teleop_interfaces/EmergencyStop]: Activates emergency stop, which disables all input and turns the brakes to maximum.

## Launch File

The node can be started by running:

`ros2 launch teleop teleop.launch.py`

You also need to run [lgsvl_bridge](https://github.com/lgsvl/ros2-lgsvl-bridge) so ROS is able to communicate with the simulator.

## Parameters

* accel (int, default: 5)
  * The axis on joystick which controls the acceleration of the car.

* braking (int, default: 2)
  * The axis on joystick which controls the braking.

* wheel (int, default: 0)
  * The axis on joystick which controls the turning of the car.

* estop (int, default: 5)
  * The button on the joystick which executes an emergengy stop. An emergency stop disables all input and turns the brakes to maximum.

* target_angular_rate (int, default: 150)
  * Determines the maximum velocity of the vehicle.

The mapping from index values to actual gamepad buttons and axes can be found on the [wiki for the `joy` package](http://wiki.ros.org/joy). 
