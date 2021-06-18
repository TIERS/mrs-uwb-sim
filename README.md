# UWB Simulation in Multi-Robot System
This is a repo to simulate the Ultra-wideband positioning system (global and relative) in multi-robot system.

## Installation
### PX4 Flight Control

## Simulation Configuration


## Simulation Launch
1. Set PX4 environment
```
    cd PX4_Autopilot
    source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```
2. launch the UAV and UGV simulation scenario
```
roslaunch uwb_multi_robot_sim scenarioDroneMobile_px4.launch
```

## Robot Control

