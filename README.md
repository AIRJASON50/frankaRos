# FR3机器人带独立探头仿真

![Franka FR3 Robot](https://franka.de/assets/images/content/franka-emika-fr3.jpg)

## 项目概述
本项目是基于ROS和Gazebo的Franka FR3机器人仿真系统，特别针对带有力探头的应用场景进行了优化。系统能够模拟机器人与环境的互动，获取力反馈数据，并通过多种控制器实现不同的运动模式。

## 功能特点
- 独立设计的力探头末端执行器，可测量接触力
- 多种控制模式：圆周运动、矩形运动、八字形、直线往返和自定义轨迹等
- 多种力轨迹模式：恒定力、正弦变化力、阶跃力
- 支持力噪声干扰，便于测试鲁棒性
- 模拟软硬表面的接触场景
- 优化的机器人初始姿态，避免碰撞
- 完整的力反馈数据获取接口

## 环境要求
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- 依赖包: xacro, gazebo_ros_control, joint_state_publisher, effort_controllers

## 快速开始
1. 安装依赖：
   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-xacro ros-noetic-gazebo-ros-control ros-noetic-joint-state-publisher ros-noetic-effort-controllers
   ```

2. 克隆仓库到catkin工作空间：
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/AIRJASON50/frankaRos.git
   ```

3. 编译：
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. 启动仿真：
   ```bash
   roslaunch franka_gazebo fr3_with_independent_probe.launch
   ```

## 详细启动方法与参数

### 基础启动命令

基础仿真环境（带软接触力控制）：
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch
```

### 轨迹类型选择

支持多种轨迹类型，通过`trajectory_type`参数指定：
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=circular
```

可用轨迹类型：
- `circular`: 圆形轨迹（默认）
- `rectangular`: 矩形轨迹
- `figure_eight`: 八字形轨迹
- `linear`: 直线往返轨迹
- `custom`: 自定义轨迹（从CSV文件读取）

### 力轨迹类型选择

支持不同的力变化模式，通过`force_profile_type`参数指定：
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch force_profile_type:=sine
```

可用力轨迹类型：
- `constant`: 恒定力（默认）
- `sine`: 正弦变化力
- `step`: 阶跃力

### 力轨迹参数设置

1. **正弦力轨迹参数**：
   ```bash
   # 设置正弦力振幅(牛顿)和频率(赫兹)
   roslaunch franka_gazebo fr3_with_soft_contact.launch force_profile_type:=sine force_sine_amplitude:=5.0 force_sine_frequency:=0.5
   ```

2. **阶跃力轨迹参数**：
   ```bash
   # 设置阶跃时间(秒)和阶跃后力值(牛顿)
   roslaunch franka_gazebo fr3_with_soft_contact.launch force_profile_type:=step force_step_time:=5.0 force_step_value:=15.0
   ```

3. **力噪声参数**：
   ```bash
   # 启用力噪声并设置噪声范围
   roslaunch franka_gazebo fr3_with_soft_contact.launch force_noise_enable:=true force_noise_min:=-2.0 force_noise_max:=2.0
   ```

### 控制参数设置

1. **力控参数**：
   ```bash
   roslaunch franka_gazebo fr3_with_soft_contact.launch target_force:=10.0
   ```

2. **轨迹参数**：
   ```bash
   # 圆形轨迹半径（米）
   roslaunch franka_gazebo fr3_with_soft_contact.launch circle_radius:=0.1
   
   # 矩形尺寸（米）
   roslaunch franka_gazebo fr3_with_soft_contact.launch rect_width:=0.1 rect_height:=0.2
   
   # 八字形尺寸（米）
   roslaunch franka_gazebo fr3_with_soft_contact.launch eight_width:=0.15 eight_height:=0.1
   
   # 直线长度（米）
   roslaunch franka_gazebo fr3_with_soft_contact.launch line_length:=0.2
   ```

3. **自定义轨迹**：
   ```bash
   roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=custom custom_trajectory_file:=custom_path.csv
   ```

4. **运动速度**：
   ```bash
   roslaunch franka_gazebo fr3_with_soft_contact.launch speed_factor:=1.0
   ```

### 软接触材料参数

调整接触表面的软硬度：
```bash
roslaunch franka_gazebo fr3_with_soft_contact.launch youngs_modulus:=1000.0 poisson_ratio:=0.45
```

### 组合参数示例

```bash
# 在矩形轨迹上以8N的力运行，速度为默认的1.5倍
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=rectangular target_force:=8.0 speed_factor:=1.5 rect_width:=0.15 rect_height:=0.1

# 圆形轨迹 + 正弦力变化 + 2倍速度
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=circular force_profile_type:=sine force_sine_amplitude:=3.0 force_sine_frequency:=0.5 speed_factor:=2.0

# 八字形轨迹 + 阶跃力变化 + 力噪声
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=figure_eight force_profile_type:=step force_step_time:=5.0 force_step_value:=15.0 force_noise_enable:=true

# 自定义轨迹 + 恒定力 + 硬接触表面
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=custom custom_trajectory_file:=heart_path.csv youngs_modulus:=5000.0
```

### 力估计对比实验

要运行带力估计对比的仿真：
```bash
roslaunch franka_gazebo fr3_with_force_estimation.launch
```

此启动文件将显示Gazebo模拟力和理论估计力的对比图，并记录详细的力数据日志用于分析。

### 日志与数据记录

仿真运行时会自动记录力数据到CSV文件：
```
/home/jason/ws/catkin_ws/src/franka_ros/franka_gazebo/logs/force_data_YYYYMMDD_HHMMSS.csv
```

查看日志数据：
```bash
cd ~/ws/catkin_ws/src/franka_ros/franka_gazebo/logs
cat force_data_YYYYMMDD_HHMMSS.csv
   ```

## 主要修改
1. 优化启动文件，改进机器人位置和初始姿态
2. 调整探头质量从1.96kg到0.5kg，提高末端姿态稳定性
3. 增强控制器参数，改善末端轨迹跟踪
4. 添加多种轨迹类型支持，包括矩形、八字形、直线往返和自定义轨迹
5. 实现模块化力轨迹生成器，支持恒定力、正弦变化力和阶跃力
6. 添加力噪声功能，便于测试控制器稳健性
7. 增加力估计对比工具，支持理论模型与实际测量力比较

## 项目结构
```
franka_ros/
├── franka_example_controllers/             # 主控制器包
│   ├── include/franka_example_controllers/
│   │   ├── circle_controller.h             # 轨迹控制器
│   │   ├── trajectory_generator.h          # 轨迹生成器
│   │   ├── force_generator.h               # 力轨迹生成器
│   │   └── soft_contact_model.h            # 软接触模型
│   ├── src/
│   │   ├── circle_controller.cpp           # 控制器实现
│   │   ├── trajectory_generator.cpp        # 轨迹生成实现
│   │   ├── force_generator.cpp             # 力轨迹生成实现
│   │   └── soft_contact_model.cpp          # 接触模型实现
├── franka_gazebo/                          # 仿真环境包
│   ├── logs/                               # 日志文件夹
│   │   └── force_data_*.csv                # 力数据记录
│   ├── launch/
│   │   ├── fr3_with_soft_contact.launch    # 基础仿真启动
│   │   └── fr3_with_force_estimation.launch # 力估计对比启动
```

## 自定义轨迹创建与使用

系统支持通过CSV文件定义自定义轨迹，使机器人末端能够按照任意路径运动，同时保持恒定力控制。

### 自定义轨迹文件格式

自定义轨迹CSV文件需要遵循以下格式：

```csv
# 这是注释行，会被忽略
# 格式: x,y,z,time
# 坐标单位: 米, 时间单位: 秒
0.0000,0.0000,0.0000,0.0  # 起始点
0.0100,0.0050,0.0000,0.1  # 下一个位置点和到达该点的时间
0.0200,0.0100,0.0000,0.2
...
```

要点说明：
1. 每行包含4个值：x坐标、y坐标、z坐标、时间
2. 坐标是相对于初始接触点的相对位置（单位：米）
3. 时间是相对于轨迹开始的时间（单位：秒）
4. 以#开头的行为注释，会被忽略
5. 第一个点通常应为(0,0,0,0)，表示从当前位置开始

### 创建自定义轨迹

可以通过以下方法创建自定义轨迹：

#### 方法1：手动创建

1. 使用文本编辑器创建一个CSV文件
2. 按照上述格式定义轨迹点
3. 保存为.csv文件

#### 方法2：使用脚本生成

可以使用Python等编程语言生成复杂轨迹：

```python
import numpy as np
import pandas as pd

# 生成心形轨迹示例
t = np.linspace(0, 2*np.pi, 100)
x = 0.05 * (16 * np.sin(t)**3)
y = 0.05 * (13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t))
z = np.zeros_like(t)
time = np.linspace(0, 10, 100)  # 10秒完成整个轨迹

# 创建DataFrame并保存为CSV
df = pd.DataFrame({
    'x': x,
    'y': y,
    'z': z,
    'time': time
})
df.to_csv('heart_trajectory.csv', index=False, header=False)

# 添加注释头
with open('heart_trajectory.csv', 'r') as f:
    content = f.read()
with open('heart_trajectory.csv', 'w') as f:
    f.write('# 心形轨迹\n# 格式: x,y,z,time\n# 坐标单位: 米, 时间单位: 秒\n')
    f.write(content)
```

#### 方法3：从现有数据转换

如果有其他格式的轨迹数据，可以编写转换脚本将其转换为所需的CSV格式。

### 使用自定义轨迹

1. 将创建好的CSV文件放入以下目录：
   ```
   ~/ws/catkin_ws/src/franka_ros/franka_example_controllers/config/
   ```

2. 启动仿真并指定自定义轨迹：
   ```bash
   roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=custom custom_trajectory_file:=your_trajectory.csv
   ```

   如果轨迹文件不在默认目录，可以提供完整路径：
   ```bash
   roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=custom custom_trajectory_file:=/完整/路径/your_trajectory.csv
   ```

### 轨迹示例

系统默认提供了一个示例轨迹文件：`custom_trajectory_example.csv`，可以参考其格式：

```bash
# 查看示例轨迹文件
cat ~/ws/catkin_ws/src/franka_ros/franka_example_controllers/config/custom_trajectory_example.csv
```

### 调整轨迹执行速度

通过`speed_factor`参数可以调整轨迹执行速度：

```bash
# 以0.5倍速执行自定义轨迹
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=custom custom_trajectory_file:=your_trajectory.csv speed_factor:=0.5

# 以2倍速执行自定义轨迹
roslaunch franka_gazebo fr3_with_soft_contact.launch trajectory_type:=custom custom_trajectory_file:=your_trajectory.csv speed_factor:=2.0
```

### 注意事项

1. 确保轨迹点之间的距离合理，避免机器人需要做大幅度快速运动
2. 轨迹应平滑过渡，避免突变导致控制不稳定
3. 考虑机器人的工作空间限制，避免轨迹超出范围
4. 在实际运行前，建议先以低速测试自定义轨迹
5. Z轴方向的运动会影响接触力，使用时需谨慎设计

## 联系方式
GitHub: [AIRJASON50](https://github.com/AIRJASON50)

## 许可证
Apache License 2.0 