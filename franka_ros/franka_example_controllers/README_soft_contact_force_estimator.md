# 软接触力估计器

本模块使用赫兹接触理论实现了一个软接触力估计器，用于估计机械臂末端与软体块接触时的接触力，并与Gazebo仿真的力进行对比，以验证模型的准确性。

## 功能特点

1. 基于赫兹接触理论的软接触力估计
2. 实时对比Gazebo仿真力与估计力
3. 记录力数据、位置、压入深度，便于后续分析
4. 多种轨迹类型的力对比支持（圆形、矩形、八字形、直线）
5. 支持可视化接触力对比曲线

## 使用方法

### 启动仿真与力估计

```bash
# 使用默认参数启动
roslaunch franka_gazebo fr3_with_force_estimation.launch

# 使用自定义参数
roslaunch franka_gazebo fr3_with_force_estimation.launch trajectory_type:=circular circle_radius:=0.08 young_modulus:=1500
```

### 可用参数

#### 轨迹相关参数

| 参数名 | 描述 | 默认值 |
| ------ | ---- | ------ |
| trajectory_type | 轨迹类型 (circular/rectangular/figure_eight/line/custom) | circular |
| circle_frequency | 轨迹运动频率 [Hz] | 0.3 |
| center_x | 轨迹中心点X坐标 [m] | 0.4 |
| center_y | 轨迹中心点Y坐标 [m] | 0.0 |
| center_z | 轨迹中心点Z坐标 [m] | 0.505 |
| speed_factor | 轨迹运动速度因子 | 0.2 |
| circle_radius | 圆周运动半径 [m] | 0.1 |
| circle_plane | 圆周运动平面 (xy/xz/yz) | xy |
| rect_length | 矩形长度 [m] | 0.2 |
| rect_width | 矩形宽度 [m] | 0.1 |
| eight_radius_x | 八字形X轴半径 [m] | 0.1 |
| eight_radius_y | 八字形Y轴半径 [m] | 0.05 |
| line_length | 直线长度 [m] | 0.2 |

#### 软接触模型相关参数

| 参数名 | 描述 | 默认值 |
| ------ | ---- | ------ |
| young_modulus | 软体杨氏模量 [Pa] | 1000.0 |
| poisson_ratio | 软体泊松比 | 0.45 |
| friction_coef | 摩擦系数 | 0.3 |
| contact_radius | 接触探头半径 [m] | 0.01 |

### 查看力对比结果

1. **RViz中查看接触可视化**：启动后会自动运行RViz，展示接触点、接触力方向等信息。

2. **rqt_plot查看力对比**：
   - 力大小对比：启动后会自动打开两个力对比图表窗口
   - 第一个窗口: Gazebo力(红线)与估计力(绿线)对比
   - 第二个窗口: 力与压入深度的关系曲线

3. **使用rosbag回放**：
   ```bash
   # 回放力估计的记录
   rosbag play $(find franka_example_controllers)/data/force_estimation.bag
   ```

## 日志数据

力估计器会在`franka_gazebo/logs/`目录下生成CSV格式的日志文件，记录以下数据：

- 时间戳
- 末端位置 (x, y, z)
- 压入深度
- Gazebo力 (x, y, z分量及大小)
- 估计力 (x, y, z分量及大小)
- 误差 (x, y, z分量及大小)

日志文件名格式为`force_estimation_YYYYMMDD_HHMMSS.csv`。

## 理论基础

本估计器基于赫兹接触理论计算软接触力，核心公式：

```
F = (4/3) * E_effective * sqrt(R) * depth^(3/2)
```

其中：
- `F`是接触法向力
- `E_effective`是有效弹性模量，计算公式: `E / (1-v^2)`，其中`E`是杨氏模量，`v`是泊松比
- `R`是接触半径
- `depth`是接触深度

## 参考文献

- K.L. Johnson, "Contact Mechanics", Cambridge University Press, 1985
- Real-Time Deformable-Contact-Aware MPC 