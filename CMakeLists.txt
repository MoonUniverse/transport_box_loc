cmake_minimum_required(VERSION 2.8.3)
project(transport_box_loc)

find_package(Boost REQUIRED COMPONENTS 
thread
system
filesystem
program_options
date_time
chrono)
find_package(PCL REQUIRED)
# find_package(libpointmatcher REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)


find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    sensor_msgs
    std_msgs
    visualization_msgs
    geometry_msgs
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    roslib
    # libpointmatcher
    # libpointmatcher_ros
)

add_definitions("-std=c++14")


catkin_package(
  INCLUDE_DIRS
    include
    include/common
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    PCL
)

include_directories(
    include
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(transport_box_localizer
  src/transport_box_localizer.cpp
)

target_link_libraries(transport_box_localizer
  # ${libpointmatcher_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${CERES_INCLUDE_DIRS}
)

add_executable(localizer_node
  src/localizer_node.cpp
)
target_link_libraries(localizer_node  transport_box_localizer)


