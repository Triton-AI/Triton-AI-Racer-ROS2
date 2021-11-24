# Develop Triton AI Racer Packages on Host PC

## Method 1 (Recommanded): on Ubuntu 20.04

You need one of the following:

- (Preferred) a standalone Ubuntu 20.04 system on your PC
- A Ubuntu 20.04 virtual machine on your Windows or MacOS system.

If you only have access to a Ubuntu 18.04 machine, You need to proceed with method 3.

Steps:

1. [Install ROS2 Galactic via Debian Packages](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). Install desktop version where hinted.
2. [Install Colcon](https://colcon.readthedocs.io/en/released/user/installation.html): `sudo apt update && sudo apt install python3-colcon-common-extensions`.
3. Fork the repository on GitHub.
4. Clone your folk: `git clone <url_to_your_folk>`. Cloning with SSH is easier when pushing, but you need to [set up SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) first
5. Install ROS2 dependencies: 
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

The recommanded editor for Ubuntu is Visual Studio Code. It has handy ROS extension for handling ROS dependencies for intellisense.

## Method 2: on Windows 11 with WSL

With the latest WSL2 shipped with Windows 11 or Windows 10 Preview, you can now run Linux GUI apps natively on Windows, bringing back your favourite apps such as RVIZ2 and RQT, powerful ROS2 visualization tools.

This method is recommended if you enjoy working with Windows, or if your PC does not have enough space for a virtual machine.

**IMPORTANT**: As of now WSL2 does not support USB device passthrough, so you cannot connect Joystick or VESC to it. In addition, networking in WSL2 is funky at the moment (which only does NAT through virtual switch), making it hard to run ROS across multiple devices. For example, it is hard to run a joystick node on another machine and listen to it in WSL.

Steps:
1. Follow [WSL2 GUI tutorial](https://docs.microsoft.com/en-us/windows/wsl/tutorials/gui-apps) to install Ubuntu and GUI support
2. Following method 1 in your Linux subsystem.
3. Execute `rviz2` and expect a Linux GUI to show up in Windows. Weird.

The best editor for WSL2 is Visual Studio Code. Install it in Windows and you can connect to your Linux subsystem, work on files, and install extension for ROS, Python, and C++ in it.

## Method 3: With Docker

This method is only recommended if the aforementioned two methods do not work for you, or you are on a Ubuntu 18.04 machine that is hard to go to 20.04 (like Jetson as of December 2021).

Triton-AI-Racer-ROS2 does use docker for deployment. Using it for devop is advised if you do not want to mess up with your system and Python packages and prefer a consistent developing environment.

Steps:
1. Install docker.
2. Prepare a dockerfile with the installation scripts.
3. Build the container.
4. Run and mount your local src folder into the src folder in the container. This way your source code changes are non-volatile.

If you are not familiar with the concepts above, be careful with this method.

The best editor for docker is Visual Studio Code. You can connect to your container, work on files, and install extension for ROS, Python, and C++ in it.

## Compare different methods

| Method           | Performance | USB Devices | Space Requirement         | Advantages                  |
|------------------|-------------|-------------|---------------------------|-----------------------------|
| On native Ubuntu | Fast        | Yes         | New disk partition < 40GB | Performing, most compatable |
| In Ubuntu VM     | Slow        | Yes         | A Ubuntu VM < 40GB        | Works with Windows / MacOS  |
| WSL2             | Fair        | No          | A Ubuntu Distro < 10GB    | Lightweight, HW accel GUI   |
| Docker in Ubuntu | Fair        | Yes         | A Ubuntu Image < 5GB      | Consistent environment      |