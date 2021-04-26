cmake_minimum_required(VERSION 2.8.3)
project(stochlite_base)

add_compile_options(-std=c++11)

find_package(catkin 
    REQUIRED 
    COMPONENTS 
    roscpp
    rospy
    champ_msgs
    visualization_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    nav_msgs
    tf
    tf2_ros
    urdf
    champ
    message_filters
    stoch_linear
)

catkin_package(
    INCLUDE_DIRS include
#    LIBRARIES stochlite_base
    CATKIN_DEPENDS 
    roscpp
    rospy
    champ_msgs
    visualization_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    nav_msgs
    tf
    tf2_ros
    urdf
    message_filters
    stoch_linear
)


include_directories(
  include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
  ${champ_INCLUDE_DIRS}/champ/champ/
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_library(quadruped_controller_m src/quadruped_controller.cpp)

add_executable(quadruped_controller_node_m src/quadruped_controller_node.cpp)
target_link_libraries(quadruped_controller_node_m quadruped_controller_m trot ${catkin_LIBRARIES})
add_dependencies(quadruped_controller_node_m ${catkin_EXPORTED_TARGETS})
add_dependencies(quadruped_controller_m ${catkin_EXPORTED_TARGETS})

add_executable(message_relay_node_m src/message_relay_node.cpp)
target_link_libraries(message_relay_node_m message_relay ${catkin_LIBRARIES})
add_dependencies(message_relay_node_m ${catkin_EXPORTED_TARGETS})
add_dependencies(message_relay ${catkin_EXPORTED_TARGETS})

install(TARGETS 
    quadruped_controller_m
    quadruped_controller_node_m
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )