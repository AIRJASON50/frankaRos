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
