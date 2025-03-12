# teraranger_evo_ros2

## Description

This repository contains a ROS2 package written in Python for reading the distance from a Teraranger Evo lidar sensor connected with a USB.


## Installation Steps

1. Clone the repository in the src folder of your workspace
```
cd ~/your_ros2_ws/src
git clone https://github.com/bordeegit/teraranger_evo_ros2
```
2. Set the device communcation port in the `config/teraranger_evo_params.yaml`, should be similar to /dev/ttyACM*
3. Build the package with `colcon`
```
cd ~/your_ros2_ws
colcon build
```

## Usage 

Run the launch file 
```
ros2 launch teraranger_evo teraranger_evo.launch.py
```

