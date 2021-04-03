cmake_minimum_required(VERSION 2.8.3)

project(stochlite_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS 
  roscpp
  sensor_msgs
  roslaunch
)



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
)


include_directories(
  include/
  ${catkin_INCLUDE_DIRS}

)

add_library(stoch_gazebo_slope_esti_lib
  src/stochlite_gazebo/gazebo_slope_estimator.cpp
  src/stochlite_gazebo/tools/slope_estimator.cpp  
  src/stochlite_gazebo/tools/matrix_op.cpp
  src/stochlite_gazebo/tools/vector_op.cpp
  src/stochlite_gazebo/tools/rotation.cpp
)


add_executable(stoch_gazebo_slope_esti_node src/stochlite_gazebo_slope_esti_node.cpp)


target_link_libraries(stoch_gazebo_slope_esti_node 
  ${catkin_LIBRARIES} 
  stoch_gazebo_slope_esti_lib
)

