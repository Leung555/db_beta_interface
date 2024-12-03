# db_beta_interface
The dynamixel motors interface software for Hexapod robot in ROS 2

This project is developed based on the previous project in ROS 1 ([db_alpha_interface](https://github.com/Leung555/db_alpha_interface))

This project is developed based on the ROS 2 (test with humble, jazzy version) with Ubuntu 22, 24

## Installation guide
- First, install the ROS2 desktop version [Example: ros2 humble documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
Note: Installing ROS-Base Install (Bare Bones) might not have all dependencies related for build the packages

- install dynamixel related dependencies, and create ros2 workspace folder.
```
sudo apt-get install ros-[ROS Distribution]-dynamixel-sdk
sudo apt-get install ros-[ROS Distribution]-dynamixel-workbench
sudo apt-get install ros-[ROS Distribution]-dynamixel-workbench-toolbox
sudo apt-get install ros-[ROS Distribution]-dynamixel-workbench-msgs

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/Leung555/db_beta_interface.git
```

## Build
- build the project using colcon build
```
colcon build
```

## Parameters Config

In this version, we need to change the configuration parameters in the code of the /src/position_controller or src/reboot.cpp files such as scan_id, portname, buadrate. Normally the parameters are set as follows:

- scan_id: 73 (The interface will search the motors upto the scan_id)
- portname: /dev/ttyUSB
- buadrate: 4000000

## Running the interface
There are 2 functions :
1. position control : enable all motors as position control mode.
2. reboot
