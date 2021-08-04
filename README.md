
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Multi-robot UGVs+UAVs with UWB | Gazebo Sim


| Status  |
|---------|
| ![ROS Melodic build badge](https://github.com/TIERS/mrs-uwb-sim/actions/workflows/melodic.yml/badge.svg) |


## Description

Gazebo simulations for relative UWB-based localization. Focus on modeling UWB noise cause by ground and aerial robots in addition to their environment.

The simulator uses ClearPath Husky robots as ground units and PX4 iris quadrotors as aerial units with SITL simulation.

## Dependencies

This package depends on `mavros` and `husky_gazebo` mainly. Install dependencies for ROS Melodic:
```
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras \
    ros-melodic-husky-gazebo
```

## Installation

Create a catkin workspace
```
mkdir -p ~/mrs_uwb_sim_ws/src
cd mrs_uwb_sim_ws/src
```

Clone this repository and the `tiers_ros_msgs` dependency:
```
git clone git@github.com:TIERS/mrs-uwb-sim.git
git clone https://github.com/TIERS/tiers-ros-msgs
```

Build it using `caktkin build` (recommended).

## Download PX4

To be able to launch aerial units in the simulator, we use PX4 and its SITL simulation.

In your catkin workspace, download the PX4 Autopilot stack and built the sitl simulation files:
```
cd ~/mrs_uwb_sim_ws
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
make px4_sitl gazebo
```
   
## Launch simulation

Before launching the simulations, you will need to source PX4 files on top of the `devel/setup,bash`.
A script in this repo does both. Move it to your workspace root and source it:
```
mv ~/mrs_uwb_sim_ws/src/mrs-uwb-sim/source_all.bash
cd ~/mrs_uwb_sim_ws
source source_all.bash
```

Alternatively, you will need to run the following:
```
cd PX4-Autopilot
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

Now you can directly:
```
roslaunch mrs_uwb_sim tiers_mrs_px4.launch
```

To launch only the UWB simulator (for example while running a rosbag recorded with Gazebo):
```
roslaunch mrs_uwb_sim uwb_sim.launch
```

## UWB Ranging simulation and configuration

`TODO`

## Simulation Configuration

### Add more mobile robot
1. Open the launch file in ` ./launch/scenarios/scenarioDroneMobile_px4.launch`.
2. Copy the code block shown in the launch file which looks like the picture below. 
   ![Codes blocks needs to be added](./images/add_mobile_robot_conf.png)
3. Change the configuration shown in the picture above

### Add more drones
1. Open the launch file in ` ./launch/scenarios/scenarioDroneMobile_px4.launch`.
2. Copy the code block shown in the launch file which looks like the picture below. 
   ![Codes blocks needs to be added](./images/add_drone_conf.png)
3. Change the configuration shown in the picture above

### Change environment around
`TO DO`

## Simulation Launch
1. Set PX4 environment
```
cd PX4-Autopilot
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

2. launch the UAV and UGV simulation scenario
```
roslaunch uwb_multi_robot_sim scenarioDroneMobile_px4.launch
```

## Robot Control

### PX4 Control Fly Simulate
```
export ROS_NAMESPACE=uav0 # according to the robot you want to control
roslaunch uwb_multi_robot_sim fly_simulate.launch
```

### Keyboard Control
```
export ROS_NAMESPACE=uav0 # according to the robot you want to control
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
``` 
