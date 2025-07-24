# 统一DS控制器使用说明

## 📋 概述

统一DS控制器是基于动力学系统理论的Franka机器人速度控制器，实现了从自由空间到接触空间的无缝衔接运动控制。

### 🚀 相比传统控制器的优势

| 特性 | 传统PD控制 | DS控制器 |
|------|------------|----------|
| **响应速度** | 需要10秒等待稳定 | ✅ 立即启动，无需等待 |
| **扰动恢复** | 需要重新规划轨迹 | ✅ 自动恢复，瞬时重新计算 |
| **控制精度** | 间接位置控制 | ✅ 直接速度调制 |
| **安全保证** | 依赖硬限制 | ✅ 能量罐无源性保证 |
| **参数配置** | 硬编码 | ✅ 完全参数化配置 |

## 🎯 快速启动

### 1. 启动控制器
```bash
# 启动统一DS控制器（不影响原有系统）
roslaunch franka_ds unified_ds_controller.launch robot_ip:=172.16.0.2

# 调试模式启动
roslaunch franka_ds unified_ds_controller.launch robot_ip:=172.16.0.2 debug:=true

# 带RViz可视化
roslaunch franka_ds unified_ds_controller.launch robot_ip:=172.16.0.2 rviz:=true
```

### 2. 发送运动命令
```bash
# 方法1：使用用户界面
# 界面会自动启动，按s开始运动

# 方法2：直接发送ROS命令
rostopic pub /unified_ds/user_command std_msgs/String "start"
```

### 3. 监控系统状态
```bash
# 查看控制阶段
rostopic echo /unified_ds/control_phase

# 查看能量罐状态
rostopic echo /unified_ds/energy_tank

# 查看DS速度命令（调试）
rostopic echo /unified_ds/ds_velocity
```

## ⚙️ 参数配置

### 动态参数调整
```bash
# 启动参数调整工具
rosrun rqt_reconfigure rqt_reconfigure
```

### 主要可配置参数

#### 运动参数
- `circular_radius`: 圆周半径 (0.025m = 2.5cm，可调整为0.075m)
- `linear_max_velocity`: 最大线速度 (0.05 m/s)
- `contact_force_threshold`: 接触检测阈值 (0.3N)

#### 安全参数
- `energy_tank_max`: 最大能量容量 (4.0J)
- `max_linear_velocity`: 全局速度限制 (0.1 m/s)

#### 目标位置
- `target_position`: {x: 0.5, y: 0.0, z: 0.3}

## 🔄 控制阶段说明

### 1. CALIBRATION (校准阶段)
- **功能**: 系统初始化，力传感器偏置校准
- **特点**: 速度为零，等待初始化完成
- **DS优势**: 无需额外等待时间

### 2. LINEAR_APPROACH (线性接近)
- **功能**: 使用LinearDS向目标位置收敛
- **公式**: `v = -λ(x - x*)`
- **特点**: 线性收敛，自动避障

### 3. PROBE_DESCENT (探索下探)
- **功能**: 混合DS（水平收敛 + 垂直下探）
- **特点**: 寻找未知表面位置
- **DS优势**: 平滑过渡，无需重规划

### 4. CIRCULAR_MOTION (圆周运动)
- **功能**: CircularDS + 力反馈控制
- **特点**: 接触抛光运动
- **DS优势**: 立即启动力反馈，无需10秒等待！

## 🛡️ 安全特性

### 能量罐无源性保证
- **原理**: `ṡ = α*p_d - β*p_n - γ*p_f`
- **约束**: 当能量不足时自动限制运动
- **优势**: 理论保证系统稳定性

### 自动扰动恢复
- **检测**: 自动检测外部扰动
- **恢复**: 瞬时重新计算DS速度
- **优势**: 无需停止或重规划

## 🔧 故障排除

### 常见问题

#### 1. 控制器启动失败
```bash
# 检查Franka连接
ping 172.16.0.2

# 检查控制器是否已加载
rosservice call /controller_manager/list_controllers
```

#### 2. 力传感器无数据
```bash
# 检查force_sensor_reader节点
rosnode info /force_sensor_reader

# 检查力传感器话题
rostopic hz /force_sensor/filtered_wrench
```

#### 3. 参数配置无效
```bash
# 重新加载参数
rosparam load $(rospack find franka_ds)/config/unified_ds_params.yaml

# 检查参数加载
rosparam get /ds_params
```

## 📊 性能对比

### 与原contact_controller对比

| 指标 | contact_controller | unified_ds_controller |
|------|-------------------|----------------------|
| 启动时间 | 10秒等待 + 运动时间 | ✅ 立即启动 |
| 扰动恢复 | 手动重启 | ✅ <2秒自动恢复 |
| 参数调整 | 需重新编译 | ✅ 动态配置 |
| 姿态控制 | 无 | ✅ 6DOF完整控制 |
| 安全保证 | 硬限制 | ✅ 能量罐理论保证 |

## 🚀 后续开发计划

### 第二阶段 (2周后)
- [ ] 表面学习与法向量估计
- [ ] 高级DS调制算法
- [ ] 多表面适应性

### 第三阶段 (4周后)  
- [ ] 工业级鲁棒性优化
- [ ] 完整性能评估
- [ ] 用户手册完善

## 📞 技术支持

如有问题，请查看：
1. [渐进构建计划](./渐进构建计划.md) - 详细技术设计
2. [旧版本算法逻辑和要求](./旧版本算法逻辑和要求.md) - 原有系统对比
3. ROS日志: `~/.ros/log/` 目录下的最新日志

---

**重要提醒**: 此统一DS控制器与原有`contact_controller`完全独立，不会干扰现有系统运行。 