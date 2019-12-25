cmake_minimum_required(VERSION 2.8.3)
project(gnss_driver)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  message_generation
  xsens_imu_driver
)


add_message_files(
  DIRECTORY msg
  FILES
  gpgga_msg.msg
  gprmc_msg.msg
  lidar_sync_msg.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs serial message_runtime
)
	
## Build
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ ${catkin_INCLUDE_DIRS})
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES)

add_executable(gnss_driver ${PROJECT_SOURCES})
target_link_libraries(gnss_driver ${catkin_LIBRARIES})
add_dependencies(gnss_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} 
${xsens_imu_driver_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS})

