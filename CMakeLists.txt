cmake_minimum_required(VERSION 3.0.2)
project(sarus_path_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)




catkin_install_python(PROGRAMS
  planner.py

  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sarus_path_planning
)


# projects also need
catkin_package(
  INCLUDE_DIRS cpp_algorithms
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

