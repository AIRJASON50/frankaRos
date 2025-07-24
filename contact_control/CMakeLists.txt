cmake_minimum_required(VERSION 3.4)
project(contact_control)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
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
  serial
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

# 配置catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES contact_control
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
    serial
  DEPENDS Franka
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

# 编译控制器库
add_library(contact_control
  src/contact_controller.cpp
  src/soft_contact_model.cpp
  src/log_generator.cpp
  src/energy_tank_monitor.cpp
)

# 添加依赖
add_dependencies(contact_control
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 链接库
target_link_libraries(contact_control PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

# 编译力传感器读取节点
add_executable(contact_force_sensor_read
  src/force_sensor_read.cpp
)

target_link_libraries(contact_force_sensor_read PUBLIC
  ${catkin_LIBRARIES}
)

add_dependencies(contact_force_sensor_read
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 安装
install(TARGETS contact_control contact_force_sensor_read
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

install(FILES contact_control_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 