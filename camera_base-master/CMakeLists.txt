cmake_minimum_required(VERSION 2.8.3)
project(camera_base)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    camera_info_manager
    diagnostic_updater
    dynamic_reconfigure
    image_transport
    sensor_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS camera_info_manager diagnostic_updater dynamic_reconfigure image_transport roscpp sensor_msgs
    DEPENDS
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
