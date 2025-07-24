cmake_minimum_required(VERSION 3.4)
project(franka_ds)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  franka_hw
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  visualization_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

# Configure catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_ds_core unified_ds_controller
  CATKIN_DEPENDS
    controller_interface
    franka_hw
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
    rospy
    visualization_msgs
    std_msgs
  DEPENDS Franka
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

# ========== Unified DS Controller ==========

# DS core components library
add_library(franka_ds_core
  src/ds_robot_state.cpp
  src/ds_primitives.cpp
  src/ds_utils.cpp
  src/energy_tank_manager.cpp
)

target_link_libraries(franka_ds_core
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_ds_core SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Unified DS controller
add_library(unified_ds_controller
  src/unified_ds_controller.cpp
)

target_link_libraries(unified_ds_controller
  franka_ds_core
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(unified_ds_controller SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Installation
install(TARGETS franka_ds_core unified_ds_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES unified_ds_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 