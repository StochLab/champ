cmake_minimum_required(VERSION 2.8.3)
project(stochlite_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation message_runtime)

add_message_files(
  DIRECTORY msg
  FILES
  stoch_slope_esti.msg
)

# generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime message_runtime)
