cmake_minimum_required(VERSION 2.8.3)

project(stochlite_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS 
  roscpp
  sensor_msgs
  roslaunch
  # message_filters
)

# find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES gazebo_slope_estimator
  CATKIN_DEPENDS 
  roscpp
  sensor_msgs
  roslaunch
  # message_filters
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
  /home/hubble-02/orocos_ws/stoch2.5_orocos/include/
)

add_library(stoch_gazebo_slope_esti_lib
  src/gazebo_slope_estimator.cpp
)

# Path to the stochlib
find_library(STOCH_OROCOS libstoch_2_5-gnulinux.so PATHS /home/hubble-02/orocos_ws/stoch2.5_orocos/install/lib/orocos/gnulinux/stoch_2_5)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(stoch_gazebo_slope_esti_node src/tof.cpp)

target_link_libraries(stoch_gazebo_slope_esti_node 
  ${STOCH_OROCOS} 
  ${catkin_LIBRARIES} 
  # ${Boost_LIBRARIES} 
  stoch_gazebo_slope_esti_lib
)

# 

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)
