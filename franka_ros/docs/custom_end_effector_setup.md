# Franka Emika 机器人自定义末端执行器设置指南

本文档详细说明了如何在Franka Emika机器人（FR3或Panda）上添加自定义3D模型作为末端执行器，以及如何设置物理属性和工作平台。

## 1. 准备工作

### 1.1 3D模型要求
- 文件格式：支持`.dae`（Collada）或`.stl`格式
- 模型要求：
  - 尺寸单位：米（m）
  - 坐标系：右手坐标系
  - 模型应该已经完成简化优化
  - 建议将模型分解为视觉模型（.dae）和碰撞模型（.stl）

### 1.2 文件结构
```
franka_description/
├── meshes/
│   ├── visual/          # 视觉模型（.dae文件）
│   └── collision/       # 碰撞模型（.stl文件）
└── robots/
    └── common/
        ├── custom_end_effector.xacro  # 自定义末端执行器定义文件
        └── custom_platform.xacro      # 自定义工作平台定义文件

franka_gazebo/
└── world/
    └── custom_platform.world         # 工作平台世界文件
```

## 2. 工作流程

### 2.1 准备3D模型
1. 使用3D建模软件（如Blender、SolidWorks等）创建或修改模型
2. 导出两个版本的模型：
   - 视觉模型：高精度，包含材质和纹理（.dae）
   - 碰撞模型：简化版本，仅包含基本几何形状（.stl）
3. 确保模型尺寸和方向正确
4. 将模型文件放入对应目录：
   ```bash
   cp your_model.dae franka_description/meshes/visual/
   cp your_model.stl franka_description/meshes/collision/
   cp platform_model.dae franka_description/meshes/visual/
   cp platform_model.stl franka_description/meshes/collision/
   ```

### 2.2 创建末端执行器定义文件
1. 在`franka_description/robots/common/`目录下创建`custom_end_effector.xacro`
2. 定义末端执行器的基本参数和物理属性：
   ```xml
   <?xml version='1.0' encoding='utf-8'?>
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
     <xacro:macro name="custom_end_effector" params="arm_id connected_to tcp_xyz:='0 0 0' tcp_rpy:='0 0 0' gazebo:=false">
       <!-- 末端执行器参数 -->
       <xacro:property name="ee_mass" value="0.1" />
       
       <!-- 物理属性参数 -->
       <xacro:property name="ee_stiffness" value="1000000" />  <!-- 刚度 (N/m) -->
       <xacro:property name="ee_damping" value="100" />        <!-- 阻尼 (N·s/m) -->
       <xacro:property name="ee_friction" value="0.4" />       <!-- 摩擦系数 -->
       <xacro:property name="ee_contact_max_vel" value="0" />  <!-- 最大接触速度 (m/s) -->
       <xacro:property name="ee_contact_min_depth" value="0.001" /> <!-- 最小接触深度 (m) -->
       
       <!-- 末端执行器连接 -->
       <joint name="${arm_id}_custom_ee_joint" type="fixed">
         <parent link="${connected_to}" />
         <child link="${arm_id}_custom_ee_link" />
         <origin xyz="0 0 0" rpy="0 0 0" />
       </joint>
       
       <!-- 末端执行器链接 -->
       <link name="${arm_id}_custom_ee_link">
         <visual>
           <geometry>
             <mesh filename="package://franka_description/meshes/visual/your_model.dae" />
           </geometry>
         </visual>
         <collision>
           <geometry>
             <mesh filename="package://franka_description/meshes/collision/your_model.stl" />
           </geometry>
         </collision>
         <inertial>
           <!-- 需要根据实际模型计算惯性参数 -->
           <mass value="${ee_mass}" />
           <inertia ixx="0.001" ixy="0" ixz="0" 
                    iyy="0.001" iyz="0" 
                    izz="0.001" />
         </inertial>
       </link>
       
       <!-- TCP定义 -->
       <joint name="${arm_id}_custom_ee_tcp_joint" type="fixed">
         <parent link="${arm_id}_custom_ee_link" />
         <child link="${arm_id}_custom_ee_tcp" />
         <origin xyz="${tcp_xyz}" rpy="${tcp_rpy}" />
       </joint>
       
       <link name="${arm_id}_custom_ee_tcp" />
       
       <!-- Gazebo特定标签 -->
       <xacro:if value="${gazebo}">
         <gazebo reference="${arm_id}_custom_ee_link">
           <material>Gazebo/Blue</material>
           <mu1>${ee_friction}</mu1>
           <mu2>${ee_friction}</mu2>
           <kp>${ee_stiffness}</kp>
           <kd>${ee_damping}</kd>
           <maxVel>${ee_contact_max_vel}</maxVel>
           <minDepth>${ee_contact_min_depth}</minDepth>
         </gazebo>
       </xacro:if>
     </xacro:macro>
   </robot>
   ```

### 2.3 创建工作平台定义文件
1. 在`franka_description/robots/common/`目录下创建`custom_platform.xacro`：
   ```xml
   <?xml version='1.0' encoding='utf-8'?>
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
     <xacro:macro name="custom_platform" params="name xyz:='0 0 0' rpy:='0 0 0' gazebo:=false">
       <!-- 工作平台参数 -->
       <xacro:property name="platform_mass" value="1.0" />
       
       <!-- 物理属性参数 -->
       <xacro:property name="platform_stiffness" value="1000000" />  <!-- 刚度 (N/m) -->
       <xacro:property name="platform_damping" value="100" />        <!-- 阻尼 (N·s/m) -->
       <xacro:property name="platform_friction" value="0.4" />       <!-- 摩擦系数 -->
       <xacro:property name="platform_contact_max_vel" value="0" />  <!-- 最大接触速度 (m/s) -->
       <xacro:property name="platform_contact_min_depth" value="0.001" /> <!-- 最小接触深度 (m) -->
       
       <!-- 工作平台链接 -->
       <link name="${name}_link">
         <visual>
           <geometry>
             <mesh filename="package://franka_description/meshes/visual/platform_model.dae" />
           </geometry>
         </visual>
         <collision>
           <geometry>
             <mesh filename="package://franka_description/meshes/collision/platform_model.stl" />
           </geometry>
         </collision>
         <inertial>
           <mass value="${platform_mass}" />
           <inertia ixx="0.1" ixy="0" ixz="0" 
                    iyy="0.1" iyz="0" 
                    izz="0.1" />
         </inertial>
       </link>
       
       <!-- 固定到世界坐标系 -->
       <joint name="${name}_joint" type="fixed">
         <parent link="world" />
         <child link="${name}_link" />
         <origin xyz="${xyz}" rpy="${rpy}" />
       </joint>
       
       <!-- Gazebo特定标签 -->
       <xacro:if value="${gazebo}">
         <gazebo reference="${name}_link">
           <material>Gazebo/Grey</material>
           <mu1>${platform_friction}</mu1>
           <mu2>${platform_friction}</mu2>
           <kp>${platform_stiffness}</kp>
           <kd>${platform_damping}</kd>
           <maxVel>${platform_contact_max_vel}</maxVel>
           <minDepth>${platform_contact_min_depth}</minDepth>
         </gazebo>
       </xacro:if>
     </xacro:macro>
   </robot>
   ```

### 2.4 创建工作平台世界文件
1. 在`franka_gazebo/world/`目录下创建`custom_platform.world`：
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.5">
     <world name="custom_platform_world">
       <include>
         <uri>model://ground_plane</uri>
         <pose>0 0 0 0 0 0</pose>
       </include>
       <include>
         <uri>model://sun</uri>
         <pose>0 0 10 0 0 0</pose>
       </include>
     </world>
   </sdf>
   ```

### 2.5 修改机器人URDF文件
1. 在`fr3.urdf.xacro`或`panda.urdf.xacro`中添加：
   ```xml
   <xacro:include filename="$(find franka_description)/robots/common/custom_end_effector.xacro"/>
   <xacro:include filename="$(find franka_description)/robots/common/custom_platform.xacro"/>
   
   <!-- 自定义末端执行器参数 -->
   <xacro:arg name="custom_ee" default="false" />
   <xacro:arg name="custom_ee_tcp_xyz" default="0 0 0" />
   <xacro:arg name="custom_ee_tcp_rpy" default="0 0 0" />
   
   <!-- 工作平台参数 -->
   <xacro:arg name="platform_name" default="custom_platform" />
   <xacro:arg name="platform_xyz" default="0 0 0" />
   <xacro:arg name="platform_rpy" default="0 0 0" />
   
   <!-- 添加自定义末端执行器 -->
   <xacro:if value="$(arg custom_ee)">
     <xacro:custom_end_effector
         arm_id="$(arg arm_id)"
         connected_to="$(arg arm_id)_link8"
         tcp_xyz="$(arg custom_ee_tcp_xyz)"
         tcp_rpy="$(arg custom_ee_tcp_rpy)"
         gazebo="$(arg gazebo)" />
   </xacro:if>
   
   <!-- 添加工作平台 -->
   <xacro:custom_platform
       name="$(arg platform_name)"
       xyz="$(arg platform_xyz)"
       rpy="$(arg platform_rpy)"
       gazebo="$(arg gazebo)" />
   ```

### 2.6 修改launch文件
1. 在`robot.launch`中添加参数：
   ```xml
   <!-- 自定义末端执行器参数 -->
   <arg name="custom_ee" default="false" />
   <arg name="custom_ee_tcp_xyz" default="0 0 0" />
   <arg name="custom_ee_tcp_rpy" default="0 0 0" />
   
   <!-- 工作平台参数 -->
   <arg name="platform_name" default="custom_platform" />
   <arg name="platform_xyz" default="0 0 0" />
   <arg name="platform_rpy" default="0 0 0" />
   ```

2. 在URDF生成命令中添加参数：
   ```xml
   custom_ee:=$(arg custom_ee)
   custom_ee_tcp_xyz:='$(arg custom_ee_tcp_xyz)'
   custom_ee_tcp_rpy:='$(arg custom_ee_tcp_rpy)'
   platform_name:=$(arg platform_name)
   platform_xyz:='$(arg platform_xyz)'
   platform_rpy:='$(arg platform_rpy)'
   ```

3. 修改世界文件参数：
   ```xml
   <arg name="world" default="$(find franka_gazebo)/world/custom_platform.world" />
   ```

## 3. 重要参数说明

### 3.1 末端执行器参数
- `custom_ee`: 是否启用自定义末端执行器
- `custom_ee_tcp_xyz`: TCP位置偏移（米）
- `custom_ee_tcp_rpy`: TCP旋转偏移（弧度）
- `ee_mass`: 末端执行器质量（千克）

### 3.2 末端执行器物理参数
- `ee_stiffness`: 刚度（N/m），控制接触时的弹性
- `ee_damping`: 阻尼（N·s/m），控制接触时的能量耗散
- `ee_friction`: 摩擦系数，控制滑动阻力
- `ee_contact_max_vel`: 最大接触速度（m/s），控制接触稳定性
- `ee_contact_min_depth`: 最小接触深度（m），控制穿透深度

### 3.3 工作平台参数
- `platform_name`: 工作平台名称
- `platform_xyz`: 工作平台位置（米）
- `platform_rpy`: 工作平台方向（弧度）
- `platform_mass`: 工作平台质量（千克）

### 3.4 工作平台物理参数
- `platform_stiffness`: 刚度（N/m）
- `platform_damping`: 阻尼（N·s/m）
- `platform_friction`: 摩擦系数
- `platform_contact_max_vel`: 最大接触速度（m/s）
- `platform_contact_min_depth`: 最小接触深度（m）

## 4. 使用说明

### 4.1 启动命令
```bash
roslaunch franka_gazebo fr3.launch \
  custom_ee:=true \
  custom_ee_tcp_xyz:="0 0 0.05" \
  custom_ee_tcp_rpy:="0 0 1.57" \
  platform_name:="work_platform" \
  platform_xyz:="0.5 0 0" \
  platform_rpy:="0 0 0" \
  use_gripper:=false
```

### 4.2 物理参数调整
1. 末端执行器物理参数调整：
   ```xml
   <!-- 在custom_end_effector.xacro中修改 -->
   <xacro:property name="ee_stiffness" value="2000000" />  <!-- 增加刚度 -->
   <xacro:property name="ee_damping" value="200" />        <!-- 增加阻尼 -->
   <xacro:property name="ee_friction" value="0.6" />       <!-- 增加摩擦系数 -->
   ```

2. 工作平台物理参数调整：
   ```xml
   <!-- 在custom_platform.xacro中修改 -->
   <xacro:property name="platform_stiffness" value="500000" />  <!-- 降低刚度 -->
   <xacro:property name="platform_damping" value="50" />        <!-- 降低阻尼 -->
   <xacro:property name="platform_friction" value="0.2" />      <!-- 降低摩擦系数 -->
   ```

### 4.3 物理参数调优建议
1. 接触稳定性：
   - 增加`stiffness`和`damping`可以提高接触稳定性
   - 但过高的值可能导致数值不稳定
   - 建议范围：`stiffness`=100000-1000000, `damping`=10-1000

2. 摩擦特性：
   - 增加`friction`可以提高抓取稳定性
   - 但过高的值可能导致滑动不稳定
   - 建议范围：0.1-1.0

3. 接触参数：
   - 降低`maxVel`和增加`minDepth`可以减少穿透
   - 但可能影响接触响应速度
   - 建议：`maxVel`=0, `minDepth`=0.001-0.01

## 5. 注意事项

1. 模型准备：
   - 确保模型尺寸正确（单位：米）
   - 简化碰撞模型以提高性能
   - 检查模型方向是否正确

2. 物理参数：
   - 准确设置质量参数
   - 正确计算惯性参数
   - 根据实际需求调整接触参数
   - 末端执行器和工作平台的物理参数需要匹配

3. TCP设置：
   - 确保TCP位置便于控制
   - 考虑控制器的工作空间

4. 性能优化：
   - 使用简化的碰撞模型
   - 优化视觉模型的复杂度
   - 适当设置物理参数
   - 避免过多的接触点

5. 工作平台设置：
   - 确保工作平台位置便于操作
   - 考虑工作平台与机器人的相对位置
   - 工作平台的物理属性要与任务匹配

## 6. 故障排除

1. 模型显示问题：
   - 检查文件路径是否正确
   - 确认文件格式是否支持
   - 验证模型尺寸和方向
   - 确保模型文件已正确复制到`meshes`目录
   - 检查模型文件的权限是否正确

2. 物理仿真问题：
   - 检查惯性参数
   - 调整接触参数
   - 验证质量设置
   - 如果出现穿透，增加`stiffness`和`damping`
   - 如果出现不稳定，降低`stiffness`和`damping`
   - 如果出现抖动，检查`maxVel`和`minDepth`参数
   - 确保物理参数在合理范围内

3. 控制器问题：
   - 确认TCP设置正确
   - 检查工作空间限制
   - 验证控制器参数
   - 如果控制器不稳定，检查物理参数是否合适
   - 确保TCP位置在机器人工作空间内
   - 检查控制器配置文件中的参数是否正确

4. 工作平台问题：
   - 确保工作平台正确加载
   - 检查工作平台位置是否正确
   - 验证工作平台物理参数是否合适
   - 确保工作平台不会与机器人发生碰撞
   - 检查工作平台的坐标系设置

5. 参数传递问题：
   - 确保所有参数在launch文件中正确定义
   - 检查参数名称是否一致
   - 验证参数值格式是否正确
   - 确保xacro文件中的参数引用正确
   - 检查参数默认值是否合理

6. Gazebo插件问题：
   - 确保Gazebo插件正确加载
   - 检查插件参数是否正确
   - 验证插件版本兼容性
   - 确保所有必要的依赖都已安装
   - 检查Gazebo日志中的错误信息

7. URDF/Xacro问题：
   - 确保所有xacro文件都被正确包含
   - 检查xacro宏的参数定义
   - 验证xacro文件的语法正确
   - 确保所有必要的属性都已定义
   - 检查xacro文件的引用路径

8. 常见错误及解决方案：
   - `[ERROR] [gazebo_ros_control]: No transmission elements found in URDF`：
     - 检查URDF文件中是否包含transmission标签
     - 确保控制器配置文件正确加载
   
   - `[ERROR] [gazebo]: Failed to load model`：
     - 检查模型文件路径是否正确
     - 验证模型文件格式是否支持
     - 确保模型文件存在且可访问
   
   - `[ERROR] [gazebo]: Invalid parameter`：
     - 检查参数名称是否正确
     - 验证参数值格式是否正确
     - 确保参数在有效范围内
   
   - `[ERROR] [gazebo]: Failed to create joint`：
     - 检查关节定义是否正确
     - 验证父链接和子链接是否存在
     - 确保关节类型正确
   
   - `[ERROR] [gazebo]: Failed to load mesh`：
     - 检查网格文件路径是否正确
     - 验证网格文件格式是否支持
     - 确保网格文件存在且可访问

9. 调试技巧：
   - 使用`roslaunch --screen`查看详细日志
   - 使用`gz topic -l`查看Gazebo话题
   - 使用`gz topic -e`查看话题内容
   - 使用`gz model -l`查看模型列表
   - 使用`gz model -m`查看模型信息
   - 使用`gz joint -l`查看关节列表
   - 使用`gz joint -m`查看关节信息

10. 性能优化建议：
    - 简化碰撞模型以提高性能
    - 减少不必要的视觉细节
    - 优化物理参数以减少计算量
    - 使用适当的更新频率
    - 避免过多的接触点
    - 合理设置仿真步长