# Energy Tank Monitor for FR3 Robot

## 概述

本项目为FR3机械臂集成了基于文献的能量罐安全监控系统。该系统实时监控机械臂末端能量，并在必要时限制控制信号以确保手术安全。

## 设计原理

### 能量罐理论
基于以下三篇文献的能量罐理论：
- "An Energy Tank-Based Interactive Control Architecture for Robotic Surgery"
- "Unified Passivity-Based Cartesian Force/Impedance Control for Rigid and Flexible Joint Robots via Task-Energy Tanks" 
- "Passive Hierarchical Impedance Control Via Energy Tanks"

### 核心功能
1. **末端能量计算**: 监控机械臂末端的动能和功率
2. **安全级别评估**: 基于能量、速度和功率的多层安全评估
3. **自适应缩放**: 根据安全级别自动调整控制信号强度
4. **实时监控**: 实时发布能量状态和安全级别

## 实现架构

### 文件结构
```
franka_example_controllers/
├── include/franka_example_controllers/
│   └── energy_tank_monitor.h          # 能量罐监控器头文件
├── src/
│   ├── energy_tank_monitor.cpp        # 能量罐监控器实现
│   └── circle_controller.cpp          # 集成了能量罐的控制器
└── config/
    └── simulation_parameters_config.yaml  # 包含能量罐参数配置
```

### 安全级别
- **NORMAL (0)**: 正常运行，100%控制强度
- **WARNING (1)**: 警告状态，80%控制强度  
- **CRITICAL (2)**: 临界状态，50%控制强度
- **EMERGENCY (3)**: 紧急状态，10%控制强度

### 保守参数设置 (基于FR3规格)
- 最大线速度限制: 1.5 m/s (FR3最大值3.0 m/s的50%)
- 最大角速度限制: 1.25 rad/s (FR3最大值2.5 rad/s的50%)
- 能量罐容量: 2.0-15.0 J
- 功率限制: 50.0 W

## 使用方法

### 1. 编译
```bash
cd /home/jason/ws/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 启动仿真
```bash
roslaunch franka_gazebo fr3_with_force_estimation.launch
```

### 3. 监控能量状态
```bash
# 查看能量级别
rostopic echo /circle_controller/energy_tank/level

# 查看安全状态
rostopic echo /circle_controller/energy_tank/safety_status
```

### 4. 参数调整
修改 `config/simulation_parameters_config.yaml` 中的 `energy_tank` 部分:
```yaml
energy_tank:
  capacity_max: 15.0              # 调整最大能量容量
  threshold_warning: 12.0         # 调整警告阈值
  vel_linear_warning: 1.2         # 调整速度阈值
  # ... 其他参数
```

## 数据流程

```
机器人状态 → 能量罐监控器 → 安全评估 → 缩放因子 → 调整后的控制信号 → 机器人
     ↑                                    ↓
关节力矩/速度                           ROS话题发布
```

## 监控话题

| 话题名称 | 类型 | 描述 |
|---------|------|------|
| `/circle_controller/energy_tank/level` | `std_msgs/Float64` | 当前能量级别 [J] |
| `/circle_controller/energy_tank/safety_status` | `std_msgs/String` | 安全状态 (NORMAL/WARNING/CRITICAL/EMERGENCY) |

## 安全特性

1. **保守设计**: 所有阈值都设置为机械臂最大能力的60%以下
2. **渐进式限制**: 四级安全缩放，避免突然停止
3. **实时响应**: 每个控制周期都进行安全评估
4. **参数可调**: 所有安全参数都可通过配置文件调整

## 注意事项

1. 本系统仅在仿真环境中测试过，实际机器人使用前需要充分验证
2. 能量罐参数需要根据具体应用场景调整
3. 建议在使用前进行充分的安全测试
4. 系统设计为保守安全，可能会影响机械臂性能

## 故障排除

### 编译错误
- 确保所有依赖包已正确安装
- 检查CMakeLists.txt中是否正确添加了源文件

### 运行时错误
- 检查参数配置是否正确
- 查看ROS日志: `rosrun rqt_console rqt_console`

### 性能问题
- 调整能量罐阈值参数
- 检查控制频率设置

## 扩展功能

未来可以考虑的扩展:
1. 添加更复杂的能量平衡模型
2. 集成机器学习的安全预测
3. 支持多机械臂协调的能量管理
4. 添加用户界面进行实时参数调整 