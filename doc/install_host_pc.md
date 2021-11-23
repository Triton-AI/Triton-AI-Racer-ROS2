# Develop Triton AI Racer Packages on Host PC

## Method 1 (Recommanded): on Ubuntu 20.04

You need one of the following:

- (Preferred) a standalone Ubuntu 20.04 installation on your PC
- A Ubuntu 20.04 virtual machine on your Windows or MacOS system.

If you only have access to a Ubuntu 18.04 machine, You need to proceed with method 3.

Steps:

1. [Install ROS2 Galactic via Debian Packages](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). Install desktop version where hinted.
2. [Install Colcon](https://colcon.readthedocs.io/en/released/user/installation.html): `sudo apt update && sudo apt install python3-colcon-common-extensions`.
3. Clone this repository: `git clone https://github.com/Triton-AI/Triton-AI-Racer-ROS2.git`. Cloning with SSH is easier when pushing.
4. Install ROS2 dependencies: 
```
sudo apt install python3-pip
pip install opencv-python
source /opt/ros/galactic/setup.bash
cd Triton-AI-Racer-ROS2
rosdep update
rosdep install --from-paths src --ignore-src -y -r
```
5. Build: 
```
source /opt/ros/galactic/setup.bash
cd Triton-AI-Racer-ROS2 # If you haven't done so
colcon build
```

## Method 2: on Windows 11 with WSL

## Method 3: With Docker