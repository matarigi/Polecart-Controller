
# Polecart Controller

This repository contains an implementation of a polecart controller realised via a PID and ROS2. The polecart in question is the following, created using the gazebo simulator: 

https://github.com/cscribano/gazebo_polecart_ros


## Usage

First build the package using:
```bash
colcon build
```

Them, the controller package can be started via the following command:
```bash
ros2 run polecart_controller polecart_controller
```

It is important to start the controller first and then the simulator, so that it is controlled from the start or reset the simulation with the following command:
```bash
ros2 service call /reset_simulation std_srvs/srv/Empty
```


