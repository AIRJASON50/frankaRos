# ADMM-MPC控制器移植计划（修订版）

## 1. 项目概述

### 1.1 目标
将论文《Real-Time Deformable-Contact-Aware Model Predictive Control for Force-Modulated Manipulation》中的ADMM-MPC框架移植到Franka Emika机械臂，实现在保持力控制的同时精确跟踪自定义轨迹。

### 1.2 核心功能
- 基于ADMM的实时MPC控制（1kHz控制频率）
- Hertz接触模型实现
- 力-位混合控制与轨迹优化
- 支持自定义轨迹输入
- 动态力大小控制

## 2. 项目架构

### 2.1 实际包结构
```
franka_ros/
├── franka_example_controllers/             # 主控制器包
│   ├── include/franka_example_controllers/
│   │   ├── circle_controller.h             # 轨迹控制器（含软接触交互）
│   │   ├── trajectory_generator.h          # 【待添加】轨迹生成器
│   │   └── soft_contact_model.h            # 软接触模型
│   ├── src/
│   │   ├── circle_controller.cpp           # 轨迹控制器实现
│   │   ├── trajectory_generator.cpp        # 【待添加】轨迹生成器实现
│   │   └── soft_contact_model.cpp          # 软接触模型实现
│   ├── config/
│   │   ├── trajectory_controller.yaml      # 【待添加】通用轨迹控制器配置
│   │   └── soft_contact_params.yaml        # 软接触模型参数
│   ├── launch/
│   │   ├── fr3_custom_trajectory.launch    # 【待添加】自定义轨迹启动
│   │   └── fr3_with_soft_contact.launch    # 软接触测试启动
├── franka_gazebo/                          # 仿真环境包
│   ├── logs/                               # 日志文件夹
│   │   └── force_data_*.csv                # 力数据记录
```

## 3. 实现阶段

### 阶段1：已完成部分

#### 已实现功能
- ✅ 基础控制器框架（三阶段控制：接近、接触、圆周）
- ✅ Hertz接触模型
- ✅ 恒定力控制
- ✅ 圆周轨迹跟踪
- ✅ 数据记录系统

### 阶段2：轨迹系统扩展（下一步重点）

#### 任务2.1：通用轨迹生成器
- [ ] 实现轨迹生成器接口
  - [ ] 添加直线轨迹生成
  - [ ] 添加正方形轨迹生成
  - [ ] 添加自定义路径跟踪（从文件读取）
- [ ] 修改控制器支持动态轨迹切换
- [ ] 添加轨迹参数通过ROS参数服务器配置

#### 任务2.2：动态力控制
- [ ] 实现力轨迹生成器
  - [ ] 支持恒定力
  - [ ] 支持正弦变化力
  - [ ] 支持阶跃力变化
- [ ] 实现力变化与位置轨迹的同步
- [ ] 添加力轨迹参数配置

### 阶段3：ADMM-MPC框架实现

#### 任务3.1：ADMM优化器
- [ ] 实现ADMM核心算法
  - [ ] 动力学约束处理
  - [ ] 接触约束处理
  - [ ] 轨迹跟踪约束处理
- [ ] 实现子问题求解器

#### 任务3.2：MPC控制器
- [ ] 实现有限时域预测模型
- [ ] 实现滚动时域优化
- [ ] 集成ADMM求解器和接触模型

### 阶段4：性能优化

- [ ] 优化计算效率（确保1kHz控制频率）
- [ ] 改进鲁棒性（应对外部干扰）
- [ ] 参数自动调整

## 4. 技术关键点

### 4.1 轨迹生成改进
```cpp
// 当前圆周轨迹生成
Eigen::Vector3d circular_position = circle_center_ + 
             circle_radius_ * cos(angle) * circle_x_axis_ + 
             circle_radius_ * sin(angle) * circle_y_axis_;

// 需要改进为通用轨迹生成接口
Eigen::Vector3d desired_position = trajectory_generator_->getPosition(elapsed_time_);
```

### 4.2 动态力控制
```cpp
// 当前恒力控制
double force_error = target_force_ - current_contact_state.contact_force;

// 改进为动态力控制
double target_force_current = force_trajectory_->getForce(elapsed_time_);
double force_error = target_force_current - current_contact_state.contact_force;
```

## 5. 工作计划时间表

### 短期目标（2周内）
1. 实现通用轨迹生成器（5天）
   - 设计接口和基类（1天）
   - 实现3种基本轨迹类型（2天）
   - 实现文件导入轨迹（2天）
2. 实现动态力控制（3天）
   - 设计力轨迹接口（1天）
   - 实现3种力变化模式（2天）
3. 集成测试与调优（6天）
   - 基本轨迹跟踪测试（2天）
   - 力控制测试（2天）
   - 综合测试与参数调优（2天）

### 中期目标（1-2个月）
1. ADMM优化器基础实现（3周）
2. MPC控制器框架实现（2周）
3. 集成与性能优化（3周）

## 6. 注意事项

- 保持实时性要求（1kHz控制频率）
- 确保轨迹平滑过渡，避免突变
- 力控制与轨迹跟踪需要平衡权重

## 7. 后续扩展方向

- 增加复杂路径绘制功能
- 实现多点接触控制
- 添加触觉反馈和材料识别
- 开发图形用户界面，便于轨迹设计和参数调整 