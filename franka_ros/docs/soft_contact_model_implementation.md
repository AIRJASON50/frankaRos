# 软接触模型实现总结

## 1. 概述

本文档总结了我们从ADMM项目中移植软接触模型到Franka FR3机器人的过程。我们成功实现了Hertz接触模型，并将其集成到现有的圆周控制器中，实现了三个阶段的运动控制：接近、接触和圆周运动。

## 2. 实现组件

### 2.1 软接触模型

我们实现了基于Hertz接触理论的软接触模型，主要包括以下组件：

- `SoftContactModel` 类：实现软接触力学模型
- `ContactParams` 结构体：定义接触参数（杨氏模量、泊松比等）
- `ContactState` 结构体：保存当前接触状态
- `ContactVisualizer` 类：用于可视化接触点和力

主要文件：
- `franka_example_controllers/include/franka_example_controllers/soft_contact_model.h`
- `franka_example_controllers/src/soft_contact_model.cpp`

### 2.2 控制器集成

我们修改了现有的 `CircleController` 类，添加了以下功能：

- 三段式控制流程（接近、接触、圆周运动）
- 基于软接触模型的力控制
- 接触状态监测和可视化
- 平滑过渡机制

主要文件：
- `franka_example_controllers/include/franka_example_controllers/circle_controller.h`
- `franka_example_controllers/src/circle_controller.cpp`

### 2.3 配置和启动文件

我们创建了以下配置和启动文件：

- `franka_example_controllers/config/circle_controller_with_contact.yaml`
- `franka_gazebo/launch/fr3_with_soft_contact.launch`
- `franka_gazebo/launch/rviz/fr3_contact_visualization.rviz`

## 3. 软接触模型原理

### 3.1 Hertz接触理论

Hertz接触理论描述了两个弹性体接触时的变形和力的关系。我们实现的模型基于以下假设：

- 接触区域远小于物体尺寸
- 材料为线性弹性
- 接触表面光滑

接触力计算公式：
```
F = (4/3) * E_eff * sqrt(R) * δ^(3/2)
```
其中：
- `F`：接触力
- `E_eff`：有效弹性模量 `E/(1-ν²)`
- `R`：接触工具半径
- `δ`：接触深度

### 3.2 摩擦力模型

我们实现了库仑摩擦模型，摩擦力与法向力成正比：
```
F_f = μ * F_n * v_t/|v_t|
```
其中：
- `F_f`：摩擦力
- `μ`：摩擦系数
- `F_n`：法向力
- `v_t`：切向速度

### 3.3 动态模型

我们还实现了基于接触状态的力学微分方程，用于预测接触力的变化：
```
dF/dt = K * v + 10 * a
```
其中：
- `dF/dt`：力的时间导数
- `K`：接触刚度（由接触深度动态计算）
- `v`：接触点速度
- `a`：接触点加速度

## 4. 控制器实现

### 4.1 接触检测

我们使用以下条件检测接触：
- 末端执行器位置低于软体块表面
- 接触深度大于阈值
- 接触力大于阈值

### 4.2 力控制

我们使用PID控制器实现恒力控制，调整期望位置以实现目标接触力：
```
Δp = -(K_p * e_f + K_i * ∫e_f dt + K_d * de_f/dt)
```
其中：
- `Δp`：位置调整量
- `e_f`：力误差（目标力 - 当前力）
- `K_p`, `K_i`, `K_d`：PID增益

### 4.3 轨迹生成

圆周轨迹生成使用参数化方程：
```
p(t) = p_center + r * cos(ω*t) * x_axis + r * sin(ω*t) * y_axis
```
其中：
- `p(t)`：时间t时的位置
- `p_center`：圆心（接触点）
- `r`：圆半径
- `ω`：角频率
- `x_axis`, `y_axis`：圆平面上的基向量

### 4.4 阶段转换

控制器包含三个阶段，转换条件如下：
1. 接近 → 接触：检测到接触
2. 接触 → 圆周运动：接触稳定后（固定时间）

## 5. 可视化和监控

我们实现了以下可视化功能：

- 接触点标记（红色球体）
- 接触力向量（绿色箭头）
- 接触深度文本
- 实时力数据发布
- 控制阶段状态发布

## 6. 使用方法

### 6.1 启动仿真

```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch
```

### 6.2 监控数据

以下话题可用于监控：
- `/circle_controller/contact_force`：接触力
- `/circle_controller/control_phase`：控制阶段
- `/circle_controller/equilibrium_pose`：期望位姿
- `/circle_controller/contact_markers`：接触可视化标记

### 6.3 参数调整

可在配置文件中调整的参数：
- 软接触模型参数（杨氏模量、摩擦系数等）
- 力控制参数（目标力、PID增益等）
- 圆周运动参数（半径、频率、平面等）

## 7. 结论与下一步工作

我们成功实现了软接触模型并将其集成到现有控制器中。系统能够实现稳定的接触和圆周运动。下一步工作包括：

1. 实现ADMM优化器，进一步提高接触控制性能
2. 扩展到多接触点场景
3. 实现更复杂的接触任务，如轮廓跟踪和力控绘图
4. 将系统从仿真环境迁移到实际硬件 