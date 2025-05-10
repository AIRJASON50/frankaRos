# ROS integration for Franka Robotics research robots

[![CI](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml)


See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs

# 多轨迹类型的力控制仿真

本项目实现了在保持恒定接触力的情况下，沿着各种轨迹进行运动的控制器。

## 支持的轨迹类型

控制器支持以下轨迹类型：

1. **圆形轨迹（circular）**：默认轨迹类型，可设置半径和频率
2. **矩形轨迹（rectangular）**：可设置长度和宽度
3. **八字形轨迹（figure_eight）**：可设置X轴和Y轴半径
4. **直线往返轨迹（line）**：可设置直线长度
5. **自定义轨迹（custom）**：可通过CSV文件指定自定义轨迹点

## 使用方法

### 启动仿真

使用以下命令启动仿真，可以通过参数指定轨迹类型和参数：

```bash
# 圆形轨迹
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=circular circle_radius:=0.1

# 矩形轨迹
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=rectangular rect_length:=0.2 rect_width:=0.1

# 八字形轨迹
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=figure_eight eight_radius_x:=0.1 eight_radius_y:=0.05

# 直线往返轨迹
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=line line_length:=0.2

# 自定义轨迹
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=custom custom_trajectory_file:=$(rospack find franka_example_controllers)/config/custom_trajectory_example.csv
```

### 自定义轨迹文件格式

自定义轨迹文件是CSV格式，包含以下列：
- `time`：时间点（秒）
- `x`, `y`, `z`：相对于中心点的坐标（米）

示例文件位于 `franka_example_controllers/config/custom_trajectory_example.csv`

## 控制参数

### 通用参数
- `circle_frequency`：轨迹运动频率（Hz），默认0.3
- `center_x`, `center_y`, `center_z`：轨迹中心点坐标（米）

### 轨迹特定参数
- 圆形轨迹：`circle_radius`（半径）, `circle_plane`（平面：xy/xz/yz）
- 矩形轨迹：`rect_length`（长度）, `rect_width`（宽度）
- 八字形轨迹：`eight_radius_x`（X轴半径）, `eight_radius_y`（Y轴半径）
- 直线轨迹：`line_length`（长度）
- 自定义轨迹：`custom_trajectory_file`（CSV文件路径）

## 力控制类型与参数配置

本项目支持多种力变化类型，用户可通过launch参数灵活选择和配置：

### 支持的力变化类型
- `constant`：恒定力（默认）
- `sine`：正弦变化力
- `step`：阶跃变化力

### 主要参数说明
| 参数名 | 说明 | 适用类型 | 默认值 |
|--------|------|----------|--------|
| force_profile_type | 力变化类型(constant/sine/step) | 所有 | constant |
| force_sine_amplitude | 正弦力幅值（N） | sine | 0.0 |
| force_sine_frequency | 正弦力频率（Hz） | sine | 1.0 |
| force_step_time | 阶跃发生时刻（秒） | step | 5.0 |
| force_step_value | 阶跃后力值（N） | step | 0.0 |
| force_noise_enable | 是否启用力噪音 (true/false) | 所有 | false |
| force_noise_min | 力噪音最小值（N） | force_noise_enable=true | -2.0 |
| force_noise_max | 力噪音最大值（N） | force_noise_enable=true | 2.0 |

### 力噪音说明
- 若`force_noise_enable`为`true`，则每个控制周期在目标力上叠加**三维空间（x/y/z方向）-2~2N的均匀分布噪音**，扰动作用于末端期望位置，最大扰动幅度约±2mm。
- 可通过`force_noise_min`和`force_noise_max`调整噪音幅度区间。

### 使用示例

#### 1. 恒定力（默认）
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch force_profile_type:=constant
```

#### 2. 正弦变化力
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch force_profile_type:=sine force_sine_amplitude:=2.0 force_sine_frequency:=0.5
```

#### 3. 阶跃变化力
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch force_profile_type:=step force_step_time:=5.0 force_step_value:=15.0
```

#### 4. 启用力噪音
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch force_profile_type:=constant force_noise_enable:=true force_noise_min:=-2.0 force_noise_max:=2.0
```

#### 5. 关闭力噪音（默认）
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch force_profile_type:=constant force_noise_enable:=false
```

你可以将上述参数与轨迹参数组合使用，实现任意轨迹+任意力变化的实验。
