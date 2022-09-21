# db_beta_interface
The dynamixel motors interface software for Hexapod robot in ROS 2

This project is developed based on the previous project in ROS 1 ([db_alpha_interface](https://github.com/Leung555/db_alpha_interface))

This project is developed based on the ROS 2 humble with Ubuntu 22.04.1.

## Installation guide
- First, install the ROS2 humble desktop version [ros2 humble documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
Note: Installing ROS-Base Install (Bare Bones) might not have all dependencies related for build the packages

- create ros2 workspace folder and download packages into the src/ folder. use -b to specify the branch (ros2 humble) of the packages we want to use.
```
cd /src/
git clone https://github.com/Leung555/db_beta_interface.git
git clone -b ros2 https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone -b ros2 https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
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