# Franka机械臂能量罐实现详解

## 1. 能量罐的核心原理

### 1.1 不是直接"Relax"
能量罐**不会**在超出上限时让机械臂直接"relax"（完全失去控制），而是通过以下机制实现柔顺性：

- **被动柔顺性**：通过调制控制律实现平滑的柔顺性
- **速度调制**：使用调制增益λ控制速度命令的幅度
- **保持稳定性**：确保整个系统在任何情况下都保持被动性

### 1.2 DS的恢复性机制

**关键理解**：机械臂"被拉到远处，放手后回到工作状态"的能力来自于：

1. **DS本身的吸引子特性**：
   - DS始终产生指向吸引子的速度场
   - 无论机械臂被拖到哪里，DS都会"想要"回到吸引子
   - 这是DS的基本数学特性，不依赖于能量罐

2. **能量罐的调制作用**：
   - 当外力大时：λ减小 → 速度降低 → 机械臂表现柔顺，容易被拖拽
   - 当外力消失时：λ恢复到1.0 → 速度恢复 → 机械臂重新执行DS轨迹

### 1.3 能量流动机制
```
能量罐动态：
s˙ = α(s)·pd - β(s)·pn - γ(s)·pf

其中：
- s: 能量罐当前能量级别
- pd: 耗散功率（阻尼产生，总是≥0）
- pn: 标称DS功率（可能<0或>0）
- pf: 接触力功率（可能<0或>0）
```

## 2. 发送给Franka机械臂的命令

### 2.1 命令类型
发送给Franka机械臂的是**调制后的笛卡尔速度命令**：

```cpp
// Franka接口类型
franka::CartesianVelocities velocity_command;

// 命令内容（单位：m/s 和 rad/s）
velocity_command.O_dP_EE[0] = modulated_velocity.x();  // X轴线速度
velocity_command.O_dP_EE[1] = modulated_velocity.y();  // Y轴线速度
velocity_command.O_dP_EE[2] = modulated_velocity.z();  // Z轴线速度
velocity_command.O_dP_EE[3] = omega_x;                 // X轴角速度
velocity_command.O_dP_EE[4] = omega_y;                 // Y轴角速度
velocity_command.O_dP_EE[5] = omega_z;                 // Z轴角速度
```

### 2.2 调制公式

**标称DS计算**（参考force_based_ds_modulation）：
```cpp
// 1. 计算到达速度（指向吸引子）
Eigen::Vector3d reaching_velocity = target_position - current_pos;
reaching_velocity = (max_velocity * reaching_velocity) / reaching_velocity.norm();

// 2. 计算圆周运动速度
Eigen::Vector3d circular_velocity = computeCircularMotionVelocity(current_pos, target_pos);

// 3. 使用tanh函数平滑混合
double blend_factor = std::tanh(distance_to_target / threshold);
nominal_velocity = blend_factor * reaching_velocity + (1.0 - blend_factor) * circular_velocity;
```

**调制增益λ计算**：
```cpp
// force_based_ds_modulation的调制公式
double delta = pow(2.0 * e1.dot(fx) * gammap * Fd / d1, 2.0) + 4.0 * pow(fx.norm(), 4.0);
double lambda = (-2.0 * e1.dot(fx) * gammap * Fd / d1 + sqrt(delta)) / (2.0 * pow(fx.norm(), 2.0));
```

**最终调制速度**：
```cpp
modulated_velocity = lambda * nominal_velocity + gammap * Fd * e1 / d1;
```

## 3. 柔顺性的具体表现

### 3.1 正常状态（无外力）
- 能量罐充足：s ≈ smax
- 调制增益：λ ≈ 1.0
- 机械臂行为：正常执行DS轨迹，刚性较强

### 3.2 受到外力时
- 外力消耗能量：s 下降
- 调制增益：λ 减小（0.1 - 1.0之间）
- 机械臂行为：速度降低，容易被推拽，表现柔顺

### 3.3 外力消失后
- 能量罐恢复：s 逐渐增加
- 调制增益：λ 恢复到1.0
- 机械臂行为：**DS重新发挥作用，自动回到吸引子轨迹**

## 4. 关键改进点

### 4.1 DS设计改进
1. **确保DS始终指向吸引子**：
   - 线性阶段：直接指向目标位置
   - 圆周阶段：混合到达运动和圆周运动

2. **参考force_based_ds_modulation的参数**：
   - 圆周半径：0.05m
   - 角速度：π rad/s
   - 使用tanh函数平滑过渡

### 4.2 能量罐调制改进
1. **正确的λ计算公式**：使用force_based_ds_modulation的数学公式
2. **合理的能量流动**：α, β, γ参数的正确设置
3. **特殊情况处理**：当s<0且pn<0时，λ=1.0

## 5. 预期效果

通过这些改进，机械臂应该能够：

1. **正常工作时**：平滑地从起始位置到达目标，然后进行圆周运动
2. **受到外力时**：变得柔顺，容易被推拽到其他位置
3. **外力消失后**：自动恢复到原来的轨迹，继续执行任务

这正是论文和视频中展示的"随时打断运动都能恢复运动状态"的能力。 