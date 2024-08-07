
# Polecart Controller

This repository contains an implementation of a polecart controller realised via a PID and ROS2. The polecart in question is the following, created using the gazebo simulator: 

https://github.com/cscribano/gazebo_polecart_ros


## Usage

First follow the instruction in the repository of the simulator:

https://github.com/cscribano/gazebo_polecart_ros

Then, in the src file of colcon copy this repository, build the packages using:
```bash
colcon build
```

To launch the simulator and the controller use:
```bash
ros2 launch polecart_controller polecart.launch.py
```