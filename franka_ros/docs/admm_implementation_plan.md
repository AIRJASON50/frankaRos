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
│   │   ├── force_generator.h               # 力轨迹生成器
│   │   └── soft_contact_model.h            # 软接触模型
│   ├── src/
│   │   ├── circle_controller.cpp           # 轨迹控制器实现
│   │   ├── trajectory_generator.cpp        # 轨迹生成器实现
│   │   ├── force_generator.cpp             # 力轨迹生成器实现
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

### 阶段3：ADMM-MPC框架实现

#### 任务3.1：软接触模型移植与扩展（预计1周）
- [ ] 创建独立的软接触模型类
  - [ ] 从ADMM项目移植Hertz接触理论核心公式
  - [ ] 实现基本接触力计算（法向力、摩擦力）
  - [ ] 添加接触状态估计功能
- [ ] 设计接触参数结构体
  - [ ] 支持从YAML配置文件读取参数
  - [ ] 与现有SDF模型参数保持一致

#### 任务3.2：基础MPC控制器框架（预计2周）
- [ ] 设计MPC控制器类结构
  - [ ] 实现状态预测方法
  - [ ] 提供参考轨迹生成接口
  - [ ] 集成软接触模型预测
- [ ] 实现有限时域滚动优化
  - [ ] 设计预测步长和优化窗口
  - [ ] 实现基本目标函数（轨迹误差、力跟踪误差）
  - [ ] 添加约束处理功能（状态约束、控制约束）
- [ ] 与现有控制器框架集成
  - [ ] 保持与三阶段控制流程兼容
  - [ ] 实现平滑过渡机制
- [ ] 简单优化求解器实现
  - [ ] 实现基于QP或梯度下降的简单求解器
  - [ ] 为后续ADMM优化器预留接口

#### 任务3.3：ADMM优化器开发（预计3周）
- [ ] 创建ADMM优化器类
  - [ ] 实现ADMM基本框架
  - [ ] 设计子问题分解结构
- [ ] 实现三个子问题求解
  - [ ] 动力学子问题（机器人+接触模型）
  - [ ] 逆运动学子问题（末端跟踪）
  - [ ] 约束投影子问题
- [ ] 迭代更新与收敛逻辑
  - [ ] 实现变量更新和残差计算
  - [ ] 添加收敛判断条件
  - [ ] 最大迭代次数与提前退出机制
- [ ] 优化性能与实时性
  - [ ] 矩阵运算优化
  - [ ] 并行计算机会（如可能）
  - [ ] 确保满足1kHz控制频率要求

#### 任务3.4：集成测试（预计1周）
- [ ] 创建MPC专用测试启动文件
  - [ ] 配置必要参数
  - [ ] 设置默认轨迹和力变化模式
- [ ] 基础功能测试
  - [ ] 静态环境中轨迹跟踪测试
  - [ ] 动态力变化测试
  - [ ] 外部扰动响应测试
- [ ] 比较性能评估
  - [ ] 与现有控制器对比
  - [ ] 生成分析报告

### 阶段4：性能优化与扩展功能

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

## 6. 工作计划时间表（修订版）

### 短期目标（已完成）
- ✅ 实现通用轨迹生成器
- ✅ 实现动态力控制
- ✅ 集成测试与调优

### 中期目标（6-8周）
1. 软接触模型类开发（1周）
   - 设计并实现独立的软接触模型类（3天）
   - 与现有控制器集成测试（2天）
2. 基础MPC框架实现（2周）
   - 设计控制器结构（3天）
   - 实现状态预测与优化函数（4天）
   - 与现有控制器集成（3天）
   - 初步测试（4天）
3. ADMM优化器开发（3-4周）
   - 实现基础ADMM框架（1周）
   - 开发三个子问题求解逻辑（1-2周）
   - 优化与性能调优（1周）
4. 集成测试与文档（1周）
   - 综合测试（3天）
   - 文档与教程完善（2天）

### 长期目标（3个月以上）
1. 多点接触扩展
2. 复杂任务支持（如轮廓跟踪、形变控制）
3. 实际硬件测试与部署

## 7. 实施建议

### 代码结构
```
franka_example_controllers/
├── include/franka_example_controllers/
│   ├── models/
│   │   └── soft_contact_model.h       # 软接触模型类
│   ├── optimization/
│   │   ├── admm_optimizer.h           # ADMM优化器
│   │   ├── dynamics_subproblem.h      # 动力学子问题
│   │   ├── ik_subproblem.h            # 逆运动学子问题
│   │   └── constraint_subproblem.h    # 约束子问题
│   ├── mpc_controller.h               # MPC控制器主类
│   └── [现有文件...]
├── src/
│   ├── models/
│   │   └── soft_contact_model.cpp     # 软接触模型实现
│   ├── optimization/
│   │   ├── admm_optimizer.cpp         # ADMM优化器实现
│   │   ├── dynamics_subproblem.cpp    # 动力学子问题实现
│   │   ├── ik_subproblem.cpp          # 逆运动学子问题实现
│   │   └── constraint_subproblem.cpp  # 约束子问题实现
│   ├── mpc_controller.cpp             # MPC控制器实现
│   └── [现有文件...]
```

### 优先级建议
1. 首先完成软接触模型类，使其与现有控制器兼容
2. 实现简化版MPC框架，不包含ADMM优化（使用简单求解器）
3. 在MPC框架稳定后添加ADMM优化器
4. 最后再进行性能优化和扩展功能

## 8. 注意事项

- 保持实时性要求（1kHz控制频率）
- 确保轨迹平滑过渡，避免突变
- 力控制与轨迹跟踪需要平衡权重

## 9. 后续扩展方向

- 增加复杂路径绘制功能
- 实现多点接触控制
- 添加触觉反馈和材料识别
- 开发图形用户界面，便于轨迹设计和参数调整 

## 10. 更新日志

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

### 2024-05-15
- 实现了软接触力估计器节点，用于理论模型力与Gazebo仿真力对比
- 优化了力估计的计算与日志记录系统，支持多种数据分析
- 解决了实现中遇到的几个关键问题：
  - 修复了机器人位置信息获取问题：从FrankaState消息中提取变换矩阵中的位置数据
  - 调整力检测阈值：将力变化检测阈值和绝对力阈值分别调整为5N和8N
  - 优化接触检测逻辑：首次接触需同时满足力变化和力绝对值条件，后续状态维持仅依赖力绝对值
  - 移除基于深度的接触检测，只使用力变化和绝对力值判断
  - 增加调试信息输出，记录关键状态变化
- 问题汇总与后续优化方向：
  - 深度计算需进一步优化：考虑使用滤波和更精确的参考点
  - 力变化检测逻辑需调整：当前窗口大小和变化阈值可能需要根据不同材料特性进行适配
  - 运动安排逻辑需优化：考虑添加自适应运动控制，根据接触状态动态调整
  - 日志系统增强：添加更多调试信息，便于问题分析和算法改进

### 2024-05-20
- 重构了力轨迹生成相关代码，提高模块化：
  - 创建了独立的ForceGenerator类，从CircleController中提取力轨迹生成逻辑
  - 实现了ForceGenerator的头文件和源文件
  - 添加了初始化方法，支持从ROS参数服务器读取配置
  - 分离了力轨迹生成和力噪声生成功能
  - 在CircleController中集成ForceGenerator类
- 解决了编译问题：
  - 在CircleController中保留force_noise_enable_成员变量
  - 修复了CMakeLists.txt中的依赖关系
- 优化了代码结构和注释
- 确保修改前后功能一致性，验证了力轨迹生成在不同场景下的正确行为
- 分析了日志文件，确认力控制数据记录正常
