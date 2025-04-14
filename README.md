# FR3机器人带独立探头仿真

![Franka FR3 Robot](https://franka.de/assets/images/content/franka-emika-fr3.jpg)

## 项目概述
本项目是基于ROS和Gazebo的Franka FR3机器人仿真系统，特别针对带有力探头的应用场景进行了优化。系统能够模拟机器人与环境的互动，获取力反馈数据，并通过多种控制器实现不同的运动模式。

## 功能特点
- 独立设计的力探头末端执行器，可测量接触力
- 多种控制模式：圆周运动、笛卡尔阻抗控制等
- 模拟软硬表面的接触场景
- 优化的机器人初始姿态，避免碰撞
- 完整的力反馈数据获取接口

## 环境要求
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- 依赖包: xacro, gazebo_ros_control, joint_state_publisher, effort_controllers

## 快速开始
1. 安装依赖：
   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-xacro ros-noetic-gazebo-ros-control ros-noetic-joint-state-publisher ros-noetic-effort-controllers
   ```

2. 克隆仓库到catkin工作空间：
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/AIRJASON50/frankaRos.git
   ```

3. 编译：
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. 启动仿真：
   ```bash
   roslaunch franka_gazebo fr3_with_independent_probe.launch
   ```

## 主要修改
1. 优化启动文件，改进机器人位置和初始姿态
2. 调整探头质量从1.96kg到0.5kg，提高末端姿态稳定性
3. 增强控制器参数，改善末端轨迹跟踪

## 项目结构
详细的项目结构和文件说明请参考 `franka_ros/docs/guidence.txt`。

## 联系方式
GitHub: [AIRJASON50](https://github.com/AIRJASON50)

## 许可证
Apache License 2.0 