# 力引导控制器 (Force Guided Controller)

## ✅ **实现的功能**

根据您的需求，我已经成功实现了一个完整的力引导控制器系统：

### **1. 末端姿态控制**
- **功能**：自动保持末端执行器Z轴垂直向下
- **实现**：基于旋转向量的姿态误差控制
- **参考**：采用`unified_ds_controller.cpp`中的姿态控制代码
- **参数**：可调节的姿态刚度和阻尼

### **2. 力跟随移动**
- **功能**：通过施加外力引导末端位置移动
- **力阈值**：2.0 N（可配置）
- **响应特性**：力越大移动越快
- **最大速度**：0.1 m/s（可配置）
- **参考**：基于`franka_ros`中的阻抗控制实现

### **3. 撤力保持位姿**
- **功能**：撤销外力后保持当前位姿不变
- **实现**：低通滤波的期望位置更新
- **稳定性**：临界阻尼控制确保无振荡

### **4. 动态吸引子设置**
- **功能**：通过脚本获取当前位置并设置为吸引子
- **脚本**：`set_attractor_position.py`
- **数据源**：通过`rostopic`从`/franka_state_controller/franka_states`获取
- **更新方式**：自动更新配置文件和控制器参数

## 📁 **文件结构**

```
franka_ds/
├── include/franka_ds/
│   └── force_guided_controller.h          # 控制器头文件
├── src/
│   └── force_guided_controller.cpp        # 控制器实现
├── config/
│   └── force_guided_controller.yaml       # 配置参数
├── launch/
│   └── force_guided_controller.launch     # 启动文件
├── scripts/
│   ├── set_attractor_position.py          # 吸引子设置脚本
│   ├── force_guided_info_display.py       # 信息显示脚本
│   └── force_guided_usage.md              # 使用说明
├── force_guided_plugin.xml                # 插件配置
└── README_FORCE_GUIDED.md                 # 本文档
```

## 🚀 **快速开始**

### **1. 编译系统**
```bash
cd /home/jason/ws/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps franka_ds
```

### **2. 启动控制器**
```bash
# 启动力引导控制器
roslaunch franka_ds force_guided_controller.launch
```

### **3. 力引导操作**
- 直接用手推拉机械臂末端执行器
- 需要超过2N的外力才会响应
- 撤销外力后机械臂保持当前位置

### **4. 设置吸引子位置**
```bash
# 手动移动机械臂到期望位置，然后运行：
python3 src/franka_ds/scripts/set_attractor_position.py
```

## 🔧 **技术实现详情**

### **核心控制算法**
- **阻抗控制**：基于`cartesian_impedance_example_controller`
- **姿态控制**：参考`unified_ds_controller`的实现
- **力引导**：力到速度的映射控制
- **安全保护**：力矩限制和移动范围限制

### **ROS接口**
- **输入**：外力数据、控制命令、吸引子位置
- **输出**：当前位姿、控制状态
- **配置**：YAML参数文件，支持动态调整

### **安全特性**
- 最大关节力矩限制：12 Nm
- 最大移动速度限制：0.1 m/s
- 最大单次偏移限制：0.5 m
- 力矩变化率限制：1.0 Nm

## ⚙️ **参数配置**

### **关键参数**
```yaml
# 力引导灵敏度
force_threshold: 2.0          # 响应阈值 [N]
force_gain: 0.01              # 力到速度增益 [m/(N*s)]
max_force_velocity: 0.1       # 最大引导速度 [m/s]

# 阻抗参数
translational_stiffness: 150.0  # 位置刚度 [N/m]
rotational_stiffness: 10.0      # 姿态刚度 [Nm/rad]

# 安全限制
max_joint_torque: 12.0        # 最大关节力矩 [Nm]
```

### **调优建议**
- **增加灵敏度**：降低`force_threshold`，增加`force_gain`
- **增加稳定性**：增加`translational_stiffness`，减小`filter_params`
- **调整响应速度**：修改`max_force_velocity`

## 🔍 **监控和调试**

### **实时监控**
```bash
# 查看当前位姿
rostopic echo /force_guided/current_pose

# 查看外力数据
rostopic echo /force_sensor/wrench

# 查看控制器状态
rostopic echo /force_guided/status
```

### **控制命令**
```bash
# 启用力引导模式
rostopic pub /force_guided/command std_msgs/String "data: 'enable_force_guide'"

# 禁用力引导模式
rostopic pub /force_guided/command std_msgs/String "data: 'disable_force_guide'"
```

## 🛠️ **故障排除**

### **常见问题**
1. **机械臂不响应外力**
   - 检查力传感器数据
   - 确认力超过阈值（2N）
   - 检查力引导模式是否启用

2. **姿态控制失效**
   - 检查姿态刚度和阻尼参数
   - 查看控制器日志输出
   - 确认机械臂未到达关节限位

3. **位置漂移**
   - 调整位置刚度和阻尼
   - 检查零空间控制参数
   - 确认无持续外力干扰

## 📊 **与原需求对比**

| 需求 | 实现状态 | 说明 |
|------|----------|------|
| 末端姿态保持竖直 | ✅ 完成 | 基于旋转向量的姿态控制 |
| 力跟随移动 | ✅ 完成 | 2N阈值，可调增益 |
| 撤力保持位姿 | ✅ 完成 | 低通滤波稳定控制 |
| rostopic获取位置 | ✅ 完成 | 脚本自动解析O_T_EE |
| 设置吸引子和接触面 | ✅ 完成 | 自动更新配置文件 |
| 参考franka_ros | ✅ 完成 | 基于cartesian_impedance |
| 参考unified_ds | ✅ 完成 | 姿态控制代码复用 |

## 🎯 **下一步建议**

1. **测试验证**：在实际机器人上测试控制器性能
2. **参数调优**：根据具体需求微调控制参数
3. **功能扩展**：可考虑添加轨迹记录和回放功能
4. **UI集成**：可开发图形界面进行参数调整

---

**作者**：AI Assistant  
**日期**：2024年12月  
**版本**：1.0.0 