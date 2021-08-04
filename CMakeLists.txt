cmake_minimum_required(VERSION 3.0.2)
project(mrs_uwb_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_ros)
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  mavros
  mavros_msgs
  nav_msgs
  sensor_msgs
  genmsg
  actionlib_msgs
  tiers_ros_msgs
) 

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    message_runtime
    std_msgs
    actionlib_msgs
    tiers_ros_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS} 
  ${SDFormat_INCLUDE_DIRS}
)

## Declare C++ executables
add_executable(
  ${PROJECT_NAME}_uwbsimulator 
  src/uwbsimulator.cpp
)

## Rename C++ executables without prefix
set_target_properties(
  ${PROJECT_NAME}_uwbsimulator
  PROPERTIES OUTPUT_NAME uwbsimulator PREFIX ""
)

## Link a library or executable target against
target_link_libraries(${PROJECT_NAME}_uwbsimulator
  ${catkin_LIBRARIES}
)

# include_directories(${PROJECT_NAME}/include)
# # add_executable(${PROJECT_NAME}_fly_node src/fly_simulate/fly_simulate_node.cpp)
# # target_link_libraries(${PROJECT_NAME}_fly_node ${catkin_LIBRARIES})

