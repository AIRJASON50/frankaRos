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
│   │   ├── trajectory_generator.h          # 轨迹生成器
│   │   └── soft_contact_model.h            # 软接触模型
│   ├── src/
│   │   ├── circle_controller.cpp           # 轨迹控制器实现
│   │   ├── trajectory_generator.cpp        # 轨迹生成器实现
│   │   └── soft_contact_model.cpp          # 软接触模型实现
│   ├── config/
│   │   ├── trajectory_controller.yaml      # 通用轨迹控制器配置
│   │   └── soft_contact_params.yaml        # 软接触模型参数
│   ├── launch/
│   │   ├── fr3_custom_trajectory.launch    # 自定义轨迹启动
│   │   └── fr3_with_soft_contact.launch    # 软接触测试启动
├── franka_gazebo/                          # 仿真环境包
│   ├── logs/                               # 日志文件夹
│   │   └── force_data_*.csv                # 力数据记录
```

## 3. 仿真启动方法与参数配置

### 3.1 基础启动命令
```bash
# 基础仿真命令
roslaunch franka_gazebo fr3_with_soft_contact.launch

# 指定轨迹类型
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=circular
```

### 3.2 支持的轨迹类型及参数

#### 轨迹类型
| 轨迹类型 | 参数名称 | 说明 |
|---------|---------|------|
| circular | trajectory_type:=circular | 圆形轨迹（默认） |
| rectangular | trajectory_type:=rectangular | 矩形轨迹 |
| figure_eight | trajectory_type:=figure_eight | 八字形轨迹 |
| linear | trajectory_type:=linear | 直线往返轨迹 |
| custom | trajectory_type:=custom | 自定义轨迹 |

#### 轨迹参数设置
| 参数名称 | 默认值 | 说明 |
|---------|-------|------|
| circle_radius | 0.1 | 圆形轨迹半径（米） |
| rect_width | 0.1 | 矩形宽度（米） |
| rect_height | 0.1 | 矩形高度（米） |
| eight_width | 0.15 | 八字形宽度（米） |
| eight_height | 0.1 | 八字形高度（米） |
| line_length | 0.2 | 直线长度（米） |
| custom_trajectory_file | custom_trajectory_example.csv | 自定义轨迹文件路径 |
| speed_factor | 1.0 | 轨迹运动速度系数 |

#### 力控制参数
| 参数名称 | 默认值 | 说明 |
|---------|-------|------|
| target_force | 10.0 | 目标接触力（牛顿） |
| force_p_gain | 0.05 | 力控P增益 |
| force_i_gain | 0.0 | 力控I增益 |
| force_d_gain | 0.01 | 力控D增益 |

#### 软接触参数
| 参数名称 | 默认值 | 说明 |
|---------|-------|------|
| youngs_modulus | 1000.0 | 杨氏模量（帕斯卡） |
| poisson_ratio | 0.45 | 泊松比 |
| contact_stiffness | 800.0 | 接触刚度（N/m） |
| contact_damping | 10.0 | 接触阻尼（Ns/m） |

### 3.3 使用示例

#### 基本圆形轨迹（默认）
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch
```

#### 矩形轨迹示例
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=rectangular rect_width:=0.15 rect_height:=0.1
```

#### 八字形轨迹示例
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=figure_eight eight_width:=0.2 eight_height:=0.1 speed_factor:=0.8
```

#### 自定义轨迹示例
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=custom custom_trajectory_file:=my_custom_path.csv
```

#### 调整力大小和材料属性
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch target_force:=8.0 youngs_modulus:=2000.0 poisson_ratio:=0.4
```

### 3.4 日志数据与分析

运行仿真后，力数据将被自动记录到CSV文件中：
```
franka_ros/franka_gazebo/logs/force_data_YYYYMMDD_HHMMSS.csv
```

CSV文件格式：
```
# 实验开始时间: [时间] 秒
# 本地时间: YYYYMMDD_HHMMSS
# 控制阶段说明: 0=APPROACH, 1=CONTACT, 2=TRAJECTORY
# 软块物理属性: 杨氏模量=[值]Pa, 泊松比=[值]
# 目标力: [值]N
# ---------------------------------------
time,phase,pos_x,pos_y,pos_z,force_x,force_y,force_z,force_magnitude,depth,target_force
...数据内容...
```

### 3.5 常见问题与解决方法

1. **力控不稳定**：调整力控PID参数，特别是增大阻尼系数
2. **轨迹跟踪不准确**：检查速度因子是否过大，必要时降低速度
3. **接触不稳定**：尝试调整软接触参数，增大杨氏模量或降低目标力
4. **启动失败**：检查ROS环境和依赖包是否正确安装

## 4. 实现阶段

### 阶段1：已完成部分

#### 已实现功能
- ✅ 基础控制器框架（三阶段控制：接近、接触、圆周）
- ✅ Hertz接触模型
- ✅ 恒定力控制
- ✅ 圆周轨迹跟踪
- ✅ 数据记录系统
- ✅ 完整轨迹路径可视化系统
- ✅ 轨迹预测与延迟补偿（50ms预测 + 10ms前馈）
- ✅ 探头颜色优化（从红色改为白色，提高可视性）

### 阶段2：轨迹系统扩展（已全部完成）

#### 任务2.1：通用轨迹生成器
- ✅ 实现轨迹生成器接口
  - ✅ 添加直线轨迹生成
  - ✅ 添加矩形轨迹生成
  - ✅ 添加八字形轨迹生成
  - ✅ 添加自定义路径跟踪（从文件读取）
- ✅ 修改控制器支持动态轨迹切换
- ✅ 添加轨迹参数通过ROS参数服务器和launch文件配置

#### 任务2.2：动态力控制
- ✅ 实现力轨迹生成器
  - ✅ 支持恒定力
  - ✅ 支持正弦变化力
  - ✅ 支持阶跃力变化
  - ✅ 支持三维空间力噪音扰动（可选，独立于力变化类型）
- ✅ 实现力变化与位置轨迹的同步
- ✅ 添加力轨迹参数和噪音参数配置，支持launch灵活组合

### 阶段3：MPC实现思路（结合论文与当前系统）

#### 1. MPC目标与接口
- 目标：在已知软接触模型和轨迹/力参考的前提下，实时优化机械臂的关节输入，使末端轨迹和接触力同时高精度跟踪参考。
- 输入：当前状态（关节位置/速度、末端力）、参考轨迹（位置+力）、软接触模型参数。
- 输出：下一步最优关节输入（如力矩/加速度/期望位置）。

#### 2. 有限时域预测模型设计
- 状态变量：$\mathbf{x} = [\mathbf{q}, \dot{\mathbf{q}}, \mathbf{F}_e]$（关节位置、速度、末端力）
- 控制变量：$\mathbf{u} = \tau_u$（关节力矩）
- 动力学：$\mathbf{x}[i+1] = \mathcal{F}(\mathbf{x}[i], \mathbf{u}[i])$，含软接触模型（Hertz理论）
- 约束：
  - 关节/末端物理约束（位置、速度、力矩）
  - 软接触/滑动/路径约束（如论文式(10)）
  - 轨迹跟踪误差、力误差
- 目标函数：
  - 末端轨迹误差二次型
  - 力跟踪误差二次型
  - 控制输入正则项

#### 3. MPC主循环（滚动优化）
- 每个控制周期：
  1. 采集当前状态
  2. 生成未来N步的参考轨迹/力
  3. 构建优化问题（目标+约束）
  4. 调用优化器（初期可用QP/暴力法，后续替换为ADMM）
  5. 应用第一个时刻的最优输入
  6. 状态前滚，进入下周期

#### 4. ADMM优化器接口预留
- 优化器接口设计为可插拔，初期可用简单QP/梯度法，后续替换为ADMM分布式求解（论文三块结构：动力学子问题、IK子问题、约束投影子问题）。
- ADMM实现时，需保证各子问题变量一致性与收敛。

#### 5. 与低层力控的集成
- MPC输出为期望轨迹/力，底层由现有力控/阻抗控制器实现高频跟踪。
- 保持高层MPC与低层力控解耦，便于调试与切换。

#### 6. 代码结构建议
- mpc_controller.cpp/h：主循环与接口
- mpc_optimizer_admm.cpp/h：ADMM优化器实现
- mpc_model.cpp/h：动力学与接触模型封装
- mpc_utils.cpp/h：轨迹、约束、目标函数等工具

#### 7. 近期开发建议
- 先实现MPC主流程+简单优化器，跑通仿真闭环
- 再逐步替换/集成ADMM优化器，提升约束处理能力与效率
- 每步均可与现有轨迹/力控模块无缝对接，便于调试

### 阶段4：性能优化

- [ ] 优化计算效率（确保1kHz控制频率）
- [ ] 改进鲁棒性（应对外部干扰）
- [ ] 参数自动调整

## 5. 技术关键点

### 5.1 轨迹生成改进
```cpp
// 当前圆周轨迹生成
Eigen::Vector3d circular_position = circle_center_ + 
             circle_radius_ * cos(angle) * circle_x_axis_ + 
             circle_radius_ * sin(angle) * circle_y_axis_;

// 需要改进为通用轨迹生成接口
Eigen::Vector3d desired_position = trajectory_generator_->getPosition(elapsed_time_);
```

### 5.2 动态力控制
```cpp
// 当前恒力控制
double force_error = target_force_ - current_contact_state.contact_force;

// 改进为动态力控制
double target_force_current = force_trajectory_->getForce(elapsed_time_);
double force_error = target_force_current - current_contact_state.contact_force;
```

## 6. 工作计划时间表

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

## 7. 注意事项

- 保持实时性要求（1kHz控制频率）
- 确保轨迹平滑过渡，避免突变
- 力控制与轨迹跟踪需要平衡权重

## 8. 后续扩展方向

- 增加复杂路径绘制功能
- 实现多点接触控制
- 添加触觉反馈和材料识别
- 开发图形用户界面，便于轨迹设计和参数调整 

## 9. 更新日志

### 2024-05-XX
- 实现了自定义轨迹生成器接口
- 添加了多种轨迹类型支持（直线、矩形、八字形、自定义）
- 添加了从文件读取自定义路径功能
- 优化了轨迹可视化，实现历史轨迹点(100个)和未来预测点(200个)显示
- 添加了50ms轨迹预测减少跟踪延迟
- 改进了力控制参数，提高了响应速度
- 将探头颜色从红色改为白色，提高可视性
- 实现了恒定/正弦/阶跃/三维扰动力变化，参数可通过launch灵活配置
- 支持力噪音扰动与力变化类型独立叠加，便于鲁棒性实验
- launch和README文档完善，支持任意轨迹+任意力变化+扰动组合实验
- 已知问题：高幅值扰动下实际力跟踪有物理极限，建议合理设置参数
- 所有代码已上传至GitHub仓库
