# Donkeysim <> Triton AI Racer Interface

## Purpose

This package contains `donkeysim_client_node` which interfaces between Triton AI Racer in ROS2 and the DonkeySim, an open-source autonomous vehicle simulator developed by the DonkeyCar community.

With the node, user can send ROS2 control messages defined in `tai_interface` package, and subscribe to sensor information from DonkeySim, including camera, lidar, odometry, and IMU.

## API

The `donkeysim_client_node` subscribes to:

| Topic         | Type                               | Description                                       | Note                                       |
|---------------|------------------------------------|---------------------------------------------------|--------------------------------------------|
| `vehicle_cmd` | `tai_interface/msg/VehicleControl` | Command the vehicle at a configurable frequency.  | Populate `steering_openloop` for steering. |

The `donkeysim_client_node` publishes to:

| Topic       | Type                                          | Description                  | Note                                             |
|-------------|-----------------------------------------------|------------------------------|--------------------------------------------------|
| `cam_front` | `sensor_msgs/msg/Image`                       | Camera image                 | Resolution and position are configurable         |
| `lidar`     | `sensor_msgs/msg/PointCloud2`                 | LiDAR scan                   | Resolution, level, and position are configurable |
| `imu/raw`   | `sensor_msgs/msg/Imu`                         | Imu data                     | angular acceleration is not yet available        |
| `pose`      | `geometry_msgs/msg/PoseWithCovarianceStamped` | Car position                 | Covariance is not populated                      |
| `odom`      | `nav_msgs/msg/Odometry`                       | Position and linear velocity | Angular velocity is not yet available            |

The node requires the following parameters which are not dynamically reconfigurable:

| Parameter Name             | Type     | Description                                                              | Note                                                                                   |
|----------------------------|----------|--------------------------------------------------------------------------|----------------------------------------------------------------------------------------|
| `client_config_pkg`        | `string` | The package where the client configuation yaml is located                | e.g. `donkeysim_tai_interface`                                                         |
| `client_config_file`       | `string` | The file path to the client configuation yaml within `client_config_pkg` | e.g. if the config is under `param` folder, it has to be `param/donkeysim_client.yaml` |
| `send_control_interval_ms` | `int`    | The interval (in ms) for sending the control command to the simulator    |                                                                                        |

## Inner-working

The `DonkeysimClientNode` contains a `GymInterface` which handles all the I/O work with the simulation server. The `DonkeysimClientNode` parses the inbound/outbound messages properly to/from ROS2.

`GymInterface` expects a parameter dictionary. `DonkeysimClientNode` uses `client_config_pkg` and `client_config_file` parameters to find the configuation yaml containing that dictionary.

On start, `GymInterface`, using the parameter dictionary, connects to the donkeysim, load the scene, and sends the vehilce confiugrations. Once the vehicle finishes loading, `GymInterface` will start to publish sensor messages and expect control commands to be sent.

`DonkeysimClientNode` will convert the control commands from other ROS2 nodes into a format that the DonkeySim is expecting and send them to the simulator. The rate of sending is configurable via `send_control_interval_ms` parameter, meaning control messages may be dropped. It will also parse sensor messages to ROS2 format, such as IMU, Pose, Odometry, Image, and PointCloud2, and publish them.

The processing of LiDAR data involves multiprocessing and C-type array to speed-up the process. 

## Known Issues and Future Plans

- "Ghost car", or left-over instances of client, is still a thing with the latest sim.
- The LiDAR processing may be further optimised to allow denser point clouds.