cmake_minimum_required(VERSION 2.8.3)
project(velodyne_msgs)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  VelodynePacket.msg
  VelodyneScan.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
