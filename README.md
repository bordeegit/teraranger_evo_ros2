# teraranger_evo_ros2

## Description

This repository contains a ROS2 package written in Python for reading the distance from a Teraranger Evo lidar sensor connected with a USB.
Creates a node that publishes on different topics:
* /device_name/distance_raw : Unfiltered distance in mm
* /device_name/distance : Distance in meters, `inf` if out of range, `nan` if sensor is unable to measure, `-inf` if below minimum range
* /device_name/range : Distance in a sensor_msg/Range message type, with timestamp


## Installation Steps

1. Clone the repository in the src folder of your workspace
```
cd ~/your_ros2_ws/src
git clone https://github.com/bordeegit/teraranger_evo_ros2
```
2. Set the lidar type in `config/teraranger_evo_params.yaml`
3. Set the device communcation port in `config/teraranger_evo_params.yaml`, should be similar to /dev/ttyACM* on Linux or COM* on Windows
4. Build the package with `colcon`
```
cd ~/your_ros2_ws
colcon build
```

## Usage 

Run the launch file 
```
ros2 launch teraranger_evo teraranger_evo.launch.py
```

