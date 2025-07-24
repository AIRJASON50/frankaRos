# Franka DS (Dynamic Systems) 模块

## 概述

`franka_ds` 是一个独立的 ROS 包，提供基于力传感器的 Franka 机器人接触控制功能。这个模块从 `franka_example_controllers` 包中提取了 `contact_controller` 相关的核心功能，作为独立模块便于维护和扩展。

## 功能特性

### 核心控制器：ContactController
- **多阶段状态机控制**：校准→接近→接触→深度控制→轨迹运动→圆周运动
- **力传感器集成**：实时数据读取、滤波、调零
- **力反馈控制**：基于PID控制的z轴位置调节
- **安全监控**：能量罐监控、异常检测
- **数据记录**：实时日志和可视化

### 支持组件
- **力传感器读取节点**：`force_sensor_read`
- **数据可视化**：`force_sensor_visualizer.py`
- **软接触模型**：物理交互建模
- **能量监控**：安全保护机制

## 快速开始

### 1. 编译
```bash
cd ~/ws/catkin_ws
catkin_make --only-pkg-with-deps franka_ds
source devel/setup.bash
```

### 2. 启动完整系统
```bash
roslaunch franka_ds contact_controller.launch
```

### 3. 主要参数配置

#### 机器人连接
- `robot_ip`: 机器人IP地址 (默认: 172.16.0.2)
- `robot`: 机器人类型 (panda/fr3)

#### 力传感器
- `force_sensor_baudrate`: 波特率 (默认: 921600)
- `force_sensor_frequency`: 采样频率 (默认: 1000Hz)
- `enable_force_visualizer`: 是否启用可视化 (默认: true)

#### 运动控制
- `circle_radius`: 圆周半径 (默认: 0.075m)
- `motion_frequency`: 运动频率 (默认: 0.2Hz)
- `probe_length`: 探头长度 (默认: 0.1m)

#### 力反馈控制
- `kp`: PID比例增益 (默认: 0.001)
- `kd`: PID微分增益 (默认: 0.0005)
- `z_offset_max/min`: z轴最大调节幅度 (±0.005m)

## 文件结构

```
franka_ds/
├── src/                      # 源文件
│   ├── contact_controller.cpp    # 主控制器
│   ├── force_sensor_read.cpp     # 力传感器节点
│   ├── soft_contact_model.cpp    # 软接触模型
│   ├── log_generator.cpp         # 日志生成器
│   └── energy_tank_monitor.cpp   # 能量监控
├── include/franka_ds/        # 头文件
├── config/                   # 配置文件
│   └── contact_controller.yaml
├── launch/                   # 启动文件
│   ├── contact_controller.launch
│   └── robot.rviz
├── scripts/                  # Python脚本
│   └── force_sensor_visualizer.py
└── franka_ds_plugin.xml      # 控制器插件定义
```

## 与原版差异

1. **独立性**：作为独立包，不依赖 `franka_example_controllers`
2. **简化性**：只包含 contact_controller 相关的必要组件
3. **命名空间**：使用 `franka_ds` 命名空间
4. **配置分离**：独立的配置文件便于参数调优

## 依赖关系

### 必要依赖
- `franka_hw`, `franka_control`, `franka_description`
- `controller_interface`, `hardware_interface`
- `serial` (力传感器通信)
- `visualization_msgs` (RViz可视化)

### 运行时依赖
- `franka_visualization` (启动画面)
- `rviz` (可视化界面)
- `controller_manager` (控制器管理)

## 使用说明

1. **确保机器人连接正常**
2. **检查力传感器串口连接**
3. **启动系统**：`roslaunch franka_ds contact_controller.launch`
4. **监控状态**：观察终端输出和RViz可视化
5. **力传感器可视化**：实时力数据图表窗口

## 故障排除

### 编译问题
- 确保所有Franka相关包已安装
- 检查 `libfranka` 版本兼容性

### 运行时问题
- 检查机器人IP连接
- 验证力传感器串口权限
- 确认控制器插件加载正常

## 维护说明

- 主要维护点：`contact_controller.cpp` 中的状态机逻辑
- 参数调优：`config/contact_controller.yaml` 和 launch 文件
- 扩展功能：可在 `franka_ds` 命名空间下添加新组件

## 版本信息

- 版本：1.0.0
- 基于：franka_example_controllers (franka_ros 0.10.1)
- 兼容：ROS Noetic, Franka 0.8.0+ 