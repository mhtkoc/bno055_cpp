

# bno055_cpp

This package is a ROS 2 driver for the Bosch BNO055 IMU sensor, communicating via the I2C interface. It is a C++ and ROS 2 port/fork of the original Python-based [flynneva/bno055](https://github.com/flynneva/bno055) repository.

## Features
- Compatible with ROS 2 (rclcpp)
- Direct sensor access via I2C
- Easy configuration with parameter file
- Publishes IMU, magnetometer, and temperature data

## Requirements
- ROS 2 (Foxy, Humble, or a compatible distribution)
- `libi2c-dev` (system library for I2C interface)

To install dependencies:
```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-base libi2c-dev
```

## Build
```bash
cd ~/ros2_ws/src
git clone <this repository>
cd ~/ros2_ws
colcon build --packages-select bno055_cpp
```

## Usage
```bash
source install/setup.bash
ros2 launch bno055_cpp bno055.launch.py
```

Parameters can be edited in the `config/bno055_params.yaml` file.

## Notes
- This package is a C++ and ROS 2 adaptation of the flynneva/bno055 project.
- You may need to configure hardware permissions and I2C connection settings for your system.
