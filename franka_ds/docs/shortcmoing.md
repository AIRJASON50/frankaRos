## 5. 速度场混合策略缺陷 (unified_ds_controller.cpp)

### 5.1 接近场与圆周场的平滑混合缺失
**缺陷描述**：
目前 `unified_ds_controller.cpp::computeBaseVelocityField` (L:295) 中，接近场 (`approach_velocity`) 和圆周场 (`circular_velocity`) 的混合使用了硬编码的离散权重 (`0.9/0.1`, `0.7/0.3`, `0.3/0.7`)。这种非连续的权重切换导致速度场在不同区域之间发生突变，从而使机器人运动不平滑，在圆周运动阶段表现为不匀速甚至停滞。这与论文 `接触任务中的运动和力生成：一种动力学系统方法.md` (II. 提出的方法, 附录A,B) 中强调的平滑过渡（例如使用 `tanh` 函数或旋转矩阵进行混合）原则不符。

**参考代码位置**：
- `franka_ds/src/unified_ds_controller.cpp:321-334` (离散权重切换)
- `franka_ds/src/unified_ds_controller.cpp:341-343` (Z轴强化控制可能导致冲突)

### 5.2 期望接触力生成不平滑
**缺陷描述**：
`unified_ds_controller.cpp::computeDesiredContactForce` (L:530) 中，期望接触力 `Fd(x)` 的生成是离散的阈值切换 (0N, 5N, `desired_normal_force_`)。这种不连续的力值输出会导致 `computeForceModulation` (L:510) 产生的力调制项 (`fn(x)`) 也出现突变，进而影响整体速度场的平滑性。这与论文中提及的平滑力生成（例如通过连续函数）不符。

**参考代码位置**：
- `franka_ds/src/unified_ds_controller.cpp:539-552` (期望力离散切换)

### 5.3 圆周运动的径向收缩与Z轴控制耦合问题
**缺陷描述**：
在 `unified_ds_controller.cpp::computeCircularVelocity` (L:370) 中，径向收缩 `radial_velocity` 在 `R_xy` (水平半径) 接近0时可能导致其值过大，从而在机器人接近圆心时产生不必要的急剧减速或停滞。同时，圆周运动中对Z轴的线性收敛 (`circular_velocity(2) = -2.0 * relative_position(2)`) 与 `computeBaseVelocityField` 中的Z轴强化控制可能存在冗余或冲突，影响XY平面运动的均匀性。

**参考代码位置**：
- `franka_ds/src/unified_ds_controller.cpp:383-384` (径向速度计算)
- `franka_ds/src/unified_ds_controller.cpp:398` (圆周运动Z轴控制)
