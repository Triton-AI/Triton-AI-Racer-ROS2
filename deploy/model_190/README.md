# Model 190

## Model Description

This deployment is based on the UC San Diego DSC 190 Robot (2021).

Onboard the vehicle we have

- Vehicle
  - BLDC motor
  - Flipsky VESC 6.6
  - Traxxas 1/10 chassis
  - Steering servo (came with chassis)
- Sensor
  - Intel&copy; Realsense&trade; D455 (3D camera)
  - (LiDAR option) LD06 (2D LiDAR)
  - (LiDAR option) SICK Tim 551 (2D LiDAR)
  - Sparkfun Artemis (IMU)
- Computation
  - Nvidia Jetson Xavier NX (Jetpack 4.5)
- Misc
  - 12-36VDC to 12VDC and 5VDC power distribution board

## External Drivers

We grabbed the following external ROS2 drivers:

- [ROS Wrapper for Intel&copy; Realsense&trade; Devices](https://github.com/IntelRealSense/realsense-ros) by Intel (installed as binary)
- [LD06 ROS2 Driver](https://github.com/linorobot/ldlidar) by Linorobot (built from source)
- [Razor IMU ROS2 Driver](https://github.com/klintan/ros2_razor_imu/tree/ros2) by Andreas Klintberg (built from source)

## Indoor Competition
