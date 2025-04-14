# Franka Emika 机器人球体末端执行器设置指南

本文档详细说明了如何在Franka Emika机器人（FR3或Panda）上添加和配置球体末端执行器。

## 1. 文件修改

### 1.1 fr3.urdf.xacro

在`franka_ros/franka_description/robots/fr3/fr3.urdf.xacro`文件中添加以下内容：

```xml
<!-- 球体末端执行器参数 -->
<xacro:arg name="sphere_ee" default="false" />
<xacro:arg name="sphere_tcp_xyz" default="0 0 0.03" />
<xacro:arg name="sphere_tcp_rpy" default="0 0 0" />

<!-- 添加球体末端执行器 -->
<xacro:if value="$(arg sphere_ee)">
  <xacro:sphere_end_effector
      arm_id="$(arg arm_id)"
      connected_to="$(arg arm_id)_link8"
      tcp_xyz="$(arg sphere_tcp_xyz)"
      tcp_rpy="$(arg sphere_tcp_rpy)"
      gazebo="$(arg gazebo)" />
</xacro:if>
```

### 1.2 sphere_end_effector.xacro

在`franka_ros/franka_description/robots/common/sphere_end_effector.xacro`文件中：

```xml
<?xml version='1.0' encoding='utf-8'?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="sphere_end_effector" params="arm_id connected_to tcp_xyz:='0 0 0.05' tcp_rpy:='0 0 0' gazebo:=false">
    <!-- 球体末端执行器参数 -->
    <xacro:property name="sphere_radius" value="0.03" />
    <xacro:property name="sphere_mass" value="0.1" />
    
    <!-- 球体末端执行器连接 -->
    <joint name="${arm_id}_sphere_joint" type="fixed">
      <parent link="${connected_to}" />
      <child link="${arm_id}_sphere_link" />
      <origin xyz="0 0 0" rpy="0 0 0" />
    </joint>
    
    <!-- 球体末端执行器链接 -->
    <link name="${arm_id}_sphere_link">
      <visual>
        <geometry>
          <sphere radius="${sphere_radius}" />
        </geometry>
        <material name="blue">
          <color rgba="0 0 1 1" />
        </material>
      </visual>
      <collision>
        <geometry>
          <sphere radius="${sphere_radius}" />
        </geometry>
      </collision>
      <inertial>
        <mass value="${sphere_mass}" />
        <inertia ixx="${2/5 * sphere_mass * sphere_radius * sphere_radius}" 
                 ixy="0" 
                 ixz="0" 
                 iyy="${2/5 * sphere_mass * sphere_radius * sphere_radius}" 
                 iyz="0" 
                 izz="${2/5 * sphere_mass * sphere_radius * sphere_radius}" />
      </inertial>
    </link>
    
    <!-- 末端执行器TCP -->
    <joint name="${arm_id}_sphere_tcp_joint" type="fixed">
      <parent link="${arm_id}_sphere_link" />
      <child link="${arm_id}_sphere_tcp" />
      <origin xyz="${tcp_xyz}" rpy="${tcp_rpy}" />
    </joint>
    
    <link name="${arm_id}_sphere_tcp" />
    
    <!-- Gazebo特定标签 -->
    <xacro:if value="${gazebo}">
      <gazebo reference="${arm_id}_sphere_link">
        <material>Gazebo/Blue</material>
        <mu1>0.4</mu1>
        <mu2>0.4</mu2>
        <kp>1000000</kp>
        <kd>100</kd>
      </gazebo>
    </xacro:if>
  </xacro:macro>
</robot>
```

### 1.3 robot.launch

在`franka_ros/franka_gazebo/launch/robot.launch`文件中添加：

```xml
<!-- 球体末端执行器参数 -->
<arg name="sphere_ee" default="false" />
<arg name="sphere_tcp_xyz" default="0 0 0.03" />
<arg name="sphere_tcp_rpy" default="0 0 0" />

<!-- 在URDF生成命令中添加参数 -->
<param name="robot_description"
       command="xacro $(find franka_description)/robots/$(arg robot)/$(arg robot).urdf.xacro
                gazebo:=true
                hand:=$(arg use_gripper)
                sphere_ee:=$(arg sphere_ee)
                sphere_tcp_xyz:='$(arg sphere_tcp_xyz)'
                sphere_tcp_rpy:='$(arg sphere_tcp_rpy)'
                arm_id:=$(arg arm_id)
                xyz:='$(arg x) $(arg y) $(arg z)'
                rpy:='$(arg roll) $(arg pitch) $(arg yaw)'
                $(arg xacro_args)">
</param>

<!-- 添加球体末端执行器的质量参数 -->
<param name="m_ee" value="0.1" if="$(arg sphere_ee)" />
```

### 1.4 fr3.launch

在`franka_ros/franka_gazebo/launch/fr3.launch`文件中添加：

```xml
<!-- 球体末端执行器参数 -->
<arg name="sphere_ee" default="false" />
<arg name="sphere_tcp_xyz" default="0 0 0.03" />
<arg name="sphere_tcp_rpy" default="0 0 0" />

<!-- 在include部分传递参数 -->
<include file="$(find franka_gazebo)/launch/robot.launch">
  <!-- ... 其他参数 ... -->
  <arg name="sphere_ee" value="$(arg sphere_ee)" />
  <arg name="sphere_tcp_xyz" value="$(arg sphere_tcp_xyz)" />
  <arg name="sphere_tcp_rpy" value="$(arg sphere_tcp_rpy)" />
  <!-- ... 其他参数 ... -->
</include>
```

## 2. 参数说明

### 2.1 球体末端执行器参数

- `sphere_ee`: 是否启用球体末端执行器（默认：false）
- `sphere_tcp_xyz`: 球体末端执行器TCP的位置偏移（默认：0 0 0.03米）
- `sphere_tcp_rpy`: 球体末端执行器TCP的旋转偏移（默认：0 0 0弧度）

### 2.2 球体物理参数

- `sphere_radius`: 球体半径（默认：0.03米）
- `sphere_mass`: 球体质量（默认：0.1千克）
- `mu1`, `mu2`: 摩擦系数（默认：0.4）
- `kp`, `kd`: 接触参数（默认：1000000, 100）

## 3. 使用方法

### 3.1 启动命令

```bash
roslaunch franka_gazebo fr3.launch \
  x:=-0.5 \
  world:=$(rospack find franka_gazebo)/world/empty_with_soft_block.world \
  controller:=circle_controller \
  use_gripper:=false \
  sphere_ee:=true \
  rviz:=true
```

### 3.2 参数调整

可以通过以下方式调整球体末端执行器的位置和方向：

```bash
roslaunch franka_gazebo fr3.launch \
  sphere_ee:=true \
  sphere_tcp_xyz:="0 0 0.05" \
  sphere_tcp_rpy:="0 0 1.57"
```

## 4. 注意事项

1. 确保在使用球体末端执行器时禁用夹爪（`use_gripper:=false`）
2. 球体末端执行器的TCP位置会影响控制器的参考点
3. 调整`sphere_tcp_xyz`和`sphere_tcp_rpy`时注意单位（米和弧度）
4. 如果使用不同的控制器，确保控制器支持球体末端执行器的配置

## 5. 故障排除

1. 如果出现"unknown macro name: xacro:gazebo"错误，检查`sphere_end_effector.xacro`文件中的Gazebo标签
2. 如果球体位置不正确，检查`sphere_tcp_xyz`和`sphere_tcp_rpy`参数
3. 如果控制器无法正常工作，确保`m_ee`参数设置正确 