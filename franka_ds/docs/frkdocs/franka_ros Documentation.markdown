# franka_ros

franka_ros is not supported on Windows.

Before continuing with this chapter, please install or compile franka_ros.

## franka_description

This package contains the description of Franka Robotics robots and end effectors in terms of kinematics, joint limits, visual surfaces, and collision space. The collision space is a simplified version of the visual description used to improve performance of collision checks. The descriptions are based on the URDF format according to the [URDF XML documentation](http://wiki.ros.org/urdf/XML).

To simulate the FR3 robots, you can pass a `gazebo` argument to the XACRO file. `franka_description` contains the files for all Franka Robotics robot models. For example:

```bash
xacro $(rospack find franka_description)/robots/fr3/fr3.urdf.xacro gazebo:=true
```

The same works for FER (Panda):

```bash
xacro $(rospack find franka_description)/robots/panda/panda.urdf.xacro gazebo:=true
```

### Collision Volumes

The URDF defines two types of collision models:

- **Fine**: These collision volumes are made from convex meshes, which are approximated and drastically simplified versions of the visual meshes (.dae) of each link. Use fine volumes for simulating robot collisions in Gazebo.
- **Coarse**: These collision volumes are capsules (a cylinder with two semispherical end caps) attached to each link and inflated by a certain safety distance. These are more efficient to compute and are used internally in the robot for self-collision avoidance. Use these geometries when planning motions, e.g., with MoveIt.

To distinguish between the two types of collision models, artificial links are inserted in the URDF with an `_sc` suffix (for self-collision):

- panda_link0, panda_link0_sc
- panda_link1, panda_link1_sc
- panda_link2, panda_link2_sc
- panda_link3, panda_link3_sc
- panda_link4, panda_link4_sc
- panda_link5, panda_link5_sc
- panda_link6, panda_link6_sc
- panda_link7, panda_link7_sc
- panda_link8, panda_link8_sc
- panda_hand, panda_leftfinger, panda_rightfinger, panda_hand_sc, panda_hand_tcp

You can control which collision model is loaded into your URDF via the `gazebo` XACRO argument:

- `xacro ... panda.urdf.xacro gazebo:=false`: Uses both fine and coarse collision models. This is the default if the argument is omitted. Use this for MoveIt.
- `xacro ... panda.urdf.xacro gazebo:=true`: Uses only the fine collision model. Use this for a simulatable URDF in Gazebo. The coarse collision model will cause constant collisions with the capsules of the next link.

## franka_gripper

This package implements the `franka_gripper_node` for interfacing a gripper from ROS. The node publishes the state of the gripper and offers the following action servers:

- `franka_gripper::MoveAction(width, speed)`: Moves to a target width with the defined speed.
- `franka_gripper::GraspAction(width, epsilon_inner, epsilon_outer, speed, force)`: Tries to grasp at the desired width with a desired force while closing with the given speed. The operation is successful if the distance $d$ between the gripper fingers satisfies: $width - \epsilon_{\text{inner}} < d < width + \epsilon_{\text{outer}}$.
- `franka_gripper::HomingAction()`: Homes the gripper and updates the maximum width given the mounted fingers.
- `franka_gripper::StopAction()`: Aborts a running action. This can be used to stop applying forces after grasping.
- `control_msgs::GripperCommandAction(width, max_effort)`: A standard gripper action recognized by MoveIt. If `max_effort > 0`, the gripper will try to grasp an object at the desired width. If `max_effort < 10^{-4}`, the gripper will move to the desired width without grasping.

> **Note**: Use the `max_effort` argument only when grasping an object; otherwise, the gripper will close, ignoring the `width` argument.

You can launch the `franka_gripper_node` with:

```bash
roslaunch franka_gripper franka_gripper.launch robot_ip:=<fci-ip>
```

> **Hint**: Starting with franka_ros 0.6.0, specifying `load_gripper:=true` for `roslaunch franka_control franka_control.launch` will also start a `franka_gripper_node`.

## franka_hw

This package contains the hardware abstraction of the robot for the ROS control framework based on the libfranka API. The hardware class `franka_hw::FrankaHw` offers the following interfaces to controllers:

| Interface | Function |
|-----------|----------|
| `hardware_interface::JointStateInterface` | Reads joint states. |
| `hardware_interface::VelocityJointInterface` | Commands joint velocities and reads joint states. |
| `hardware_interface::PositionJointInterface` | Commands joint positions and reads joint states. |
| `hardware_interface::EffortJointInterface` | Commands joint-level torques and reads joint states. |
| `franka_hw::FrankaStateInterface` | Reads the full robot state. |
| `franka_hw::FrankaPoseCartesianInterface` | Commands Cartesian poses and reads the full robot state. |
| `franka_hw::FrankaVelocityCartesianInterface` | Commands Cartesian velocities and reads the full robot state. |
| `franka_hw::FrankaModelInterface` | Reads the dynamic and kinematic model of the robot. |

To use ROS control interfaces, retrieve resource handles by name:

| Interface | Resource handle name |
|-----------|----------------------|
| `hardware_interface::JointStateInterface` | `<arm_id>_joint1` to `<arm_id>_joint7` |
| `hardware_interface::VelocityJointInterface` | `<arm_id>_joint1` to `<arm_id>_joint7` |
| `hardware_interface::PositionJointInterface` | `<arm_id>_joint1` to `<arm_id>_joint7` |
| `hardware_interface::EffortJointInterface` | `<arm_id>_joint1` to `<arm_id>_joint7` |
| `franka_hw::FrankaStateInterface` | `<arm_id>_robot` |
| `franka_hw::FrankaPoseCartesianInterface` | `<arm_id>_robot` |
| `franka_hw::FrankaVelocityCartesianInterface` | `<arm_id>_robot` |
| `franka_hw::FrankaModelInterface` | `<arm_id>_robot` |

> **Hint**: By default, `<arm_id>` is set to `panda`.

The `franka_hw::FrankaHw` class also implements the starting, stopping, and switching of controllers. It serves as the base class for `FrankaCombinableHw`, a hardware class that can be combined with others to control multiple robots from a single controller. The `FrankaCombinedHw` class, based on `FrankaCombinableHw`, supports an arbitrary number of Panda robots (configured by parameters) for the ROS control framework.

> **Important**: The `FrankaCombinableHw` class is available from version 0.7.0 and supports torque/effort control only.

The interfaces offered by `FrankaCombinableHw` and `FrankaCombinedHw` are:

| Interface | Function |
|-----------|----------|
| `hardware_interface::EffortJointInterface` | Commands joint-level torques and reads joint states. |
| `hardware_interface::JointStateInterface` | Reads joint states. |
| `franka_hw::FrankaStateInterface` | Reads the full robot state. |
| `franka_hw::FrankaModelInterface` | Reads the dynamic and kinematic model of the robot. |

The only admissible command interface claim is `EffortJointInterface`, which can be combined with any set of read-only interfaces (`FrankaModelInterface`, `JointStateInterface`, `FrankaStateInterface`). Resource handles follow the same naming conventions as `FrankaHw`.

> **Note**: The `FrankaCombinedHw` class offers an additional action server in the control node namespace to recover all robots. If a reflex or error occurs on any robot, the control loop of all robots stops until they are recovered.

> **Important**: `FrankaHw` uses the ROS `joint_limits_interface` to enforce position, velocity, and effort safety limits, including:
> - `joint_limits_interface::PositionJointSoftLimitsInterface`
> - `joint_limits_interface::VelocityJointSoftLimitsInterface`
> - `joint_limits_interface::EffortJointSoftLimitsInterface`
> Approaching these limits will result in unannounced modification of commands.

## franka_control

The ROS nodes `franka_control_node` and `franka_combined_control_node` are hardware nodes for ROS control that use the corresponding hardware classes from `franka_hw`. They provide a variety of ROS services to expose the full libfranka API in the ROS ecosystem, including:

- `franka_msgs::SetJointImpedance`: Specifies joint stiffness for the internal controller (damping is automatically derived).
- `franka_msgs::SetCartesianImpedance`: Specifies Cartesian stiffness for the internal controller (damping is automatically derived).
- `franka_msgs::SetEEFrame`: Specifies the transformation from `<arm_id>_EE` (end effector) to `<arm_id>_NE` (nominal end effector) frame. The transformation from flange to end effector is split into `<arm_id>_EE` to `<arm_id>_NE` and `<arm_id>_NE` to `<arm_id>_link8`. The latter can only be set through the administrator's interface.
- `franka_msgs::SetKFrame`: Specifies the transformation from `<arm_id>_K` to `<arm_id>_EE` frame.
- `franka_msgs::SetForceTorqueCollisionBehavior`: Sets thresholds for external Cartesian wrenches to configure the collision reflex.
- `franka_msgs::SetFullCollisionBehavior`: Sets thresholds for external forces on Cartesian and joint levels to configure the collision reflex.
- `franka_msgs::SetLoad`: Sets an external load to compensate (e.g., for a grasped object).
- `std_srvs::Trigger`: Allows connecting and disconnecting the hardware node (available from 0.8.0). When no active controller is running, you can disconnect the hardware node, freeing the robots for non-FCI applications (e.g., Desk-based operations). Call `connect` to resume FCI operations.

> **Important**: The `<arm_id>_EE` frame denotes the configurable end effector frame adjustable during runtime via franka_ros. The `<arm_id>_K` frame marks the center of the internal Cartesian impedance and serves as a reference for external wrenches. Neither `<arm_id>_EE` nor `<arm_id>_K` is contained in the URDF as they can be changed at runtime. By default, `<arm_id>` is set to `panda`.

To recover from errors and reflexes in reflex mode, use the `franka_msgs::ErrorRecoveryAction` via an action client or by publishing to the action goal topic:

```bash
rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"
```

After recovery, `franka_control_node` restarts the running controllers. The node does not terminate when reflexes or errors occur. Launch it with:

```bash
roslaunch franka_control franka_control.launch robot_ip:=<fci-ip> load_gripper:=<true|false> robot:=<panda|fr3>
```

This launch file also starts a `franka_control::FrankaStateController` for reading and publishing robot states, including external wrenches, configurable transforms, and joint states for visualization with RViz. A `robot_state_publisher` is started for visualization.

The package also implements `franka_combined_control_node`, a hardware node for ROS control based on `franka_hw::FrankaCombinedHw`. The set of robots is configured via the ROS parameter server in the hardware node's namespace (see `franka_combined_control_node.yaml`). An example configuration for two robots:

```yaml
robot_hardware:
  - panda_1
  - panda_2
panda_1:
  type: franka_hw/FrankaCombinableHw
  arm_id: panda_1
  joint_names:
    - panda_1_joint1
    - panda_1_joint2
    - panda_1_joint3
    - panda_1_joint4
    - panda_1_joint5
    - panda_1_joint6
    - panda_1_joint7
  joint_limit_enforcement: 0.1 # [rad]
  rate_limiting: true
  cutoff_frequency: 1000
  internal_controller: joint_impedance
  collision_config:
    lower_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0] # [Nm]
    upper_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0] # [Nm]
    lower_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0] # [Nm]
    upper_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0] # [Nm]
    lower_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0] # [N, N, N, Nm, Nm, Nm]
    upper_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0] # [N, N, N, Nm, Nm, Nm]
    lower_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0] # [N, N, N, Nm, Nm, Nm]
    upper_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0] # [N, N, N, Nm, Nm, Nm]
```

> **Note**: Ensure unique and consistent `arm_id` parameters that match the prefixes in the joint names and the robot description loaded to the control node's namespace.

A second parameter file (see `default_combined_controllers.yaml`) configures default controllers. An example for two robots:

```yaml
panda_1_state_controller:
  type: franka_control/FrankaStateController
  arm_id: panda_1
  joint_names:
    - panda_1_joint1
    - panda_1_joint2
    - panda_1_joint3
    - panda_1_joint4
    - panda_1_joint5
    - panda_1_joint6
    - panda_1_joint7
  publish_rate: 30 # [Hz]
panda_2_state_controller:
  type: franka_control/FrankaStateController
  arm_id: panda_2
  joint_names:
    - panda_2_joint1
    - panda_2_joint2
    - panda_2_joint3
    - panda_2_joint4
    - panda_2_joint5
    - panda_2_joint6
    - panda_2_joint7
  publish_rate: 30 # [Hz]
```

Launch `franka_combined_control_node` with:

```bash
roslaunch franka_control franka_combined_control.launch \
  robot_ips:="{<arm_id_1>/robot_ip: <my_ip_1>, <arm_id_2>/robot_ip: <my_ip_2>}" \
  robot:=<path_to_your_robot_description> \
  args:=<xacro_args_passed_to_the_robot_description> \
  robot_id:=<name_of_your_multi_robot_setup> \
  hw_config_file:=<path_to_your_hw_config_file> \
  controllers_file:=<path_to_your_default_controller_parameterizations> \
  controllers_to_start:=<list_of_default_controllers_to_start> \
  joint_states_source_list:=<list_of_infrastructure_to_reuse_a_complete_joint_states_topic>
```

> **Important**: Pass the correct robot IPs as a map, e.g., `{panda_1/robot_ip: <ip_1>, panda_2/robot_ip: <ip_2>}`.

## franka_visualization

This package contains publishers that connect to a robot and publish robot and gripper joint states for visualization in RViz. Launch it with:

```bash
roslaunch franka_visualization franka_visualization.launch robot_ip:=<fci-ip> load_gripper:=<true|false> robot:=<panda|fr3>
```

This is purely for visualization; no commands are sent to the robot. It can be used to check the connection.

> **Important**: Only one instance of `franka::Robot` can connect to the robot. Thus, the `franka_joint_state_publisher` cannot run in parallel with `franka_control_node`, and visualization cannot run alongside a separate controller program.

## franka_example_controllers

This package implements example controllers for controlling the robot via ROS, demonstrating the interfaces offered by `franka_hw::FrankaHw`. Each example has a stand-alone launch file that starts the controller and visualizes it.

To launch the joint impedance example:

```bash
roslaunch franka_example_controllers joint_impedance_example_controller.launch robot_ip:=<fci-ip> load_gripper:=<true|false> robot:=<panda|fr3>
```

The `dual_arm_cartesian_impedance_example_controller` showcases control of two Panda robots using `FrankaCombinedHw` with a single realtime controller for Cartesian tasks with impedance-based control. Launch it with:

```bash
roslaunch franka_example_controllers dual_arm_cartesian_impedance_example_controller.launch \
  robot_id:=<name_of_the_2_arm_setup> \
  robot_ips:="{<arm_id_1>/robot_ip: <ip_1>, <arm_id_2>/robot_ip: <ip_2>}" \
  rviz:=<true|false> \
  rqt:=<true|false>
```

This assumes a robot configuration as in `dual_panda_example.urdf.xacro`, with two Pandas mounted 1 meter apart on a box. The `rviz` option enables visualization, and `rqt` launches an RQT GUI for online adjustment of end-effector impedances via dynamic reconfigure.

## franka_gazebo

This package, available from version 0.8.0, allows simulation of the robot in Gazebo, integrated with the ROS control framework via the `gazebo_ros` package.

### Pick & Place Example

To simulate transporting a stone from A to B, launch Gazebo with a Panda and an example world:

```bash
roslaunch franka_gazebo panda.launch x:=-0.5 \
  world:=$(rospack find franka_gazebo)/world/stone.sdf \
  controller:=cartesian_impedance_example_controller \
  rviz:=true
```

This opens the Gazebo GUI with the environment and stone, and RViz for controlling the end-effector pose.

To open the gripper to a width of 8 cm at 10 cm/s:

```bash
rostopic pub --once /franka_gripper/move/goal franka_gripper/MoveActionGoal "goal: {width: 0.08, speed: 0.1}"
```

Using the Cartesian impedance controller, move the end-effector with the interactive marker in RViz to position the stone between the gripper fingers.

> **Note**: If the robot moves strangely with the elbow, adjust the nullspace stiffness via Dynamic Reconfigure: `panda/cartesian_impedance_example_controller/nullspace_stiffness`.

To grasp the 3 cm wide, 50 g stone with 5 N force:

```bash
rostopic pub --once /franka_gripper/grasp/goal franka_gripper/GraspActionGoal "goal: {width: 0.03, epsilon: {inner: 0.005, outer: 0.005}, speed: 0.1, force: 5.0}"
```

> **Note**: In Gazebo, go to View > Contacts to visualize contact points and forces.

If the grasp succeeds, the fingers hold the stone. If it fails, the goal tolerances (`epsilon_inner`, `epsilon_outer`) may be too small. Move the stone to the red drop-off area and stop the grasp:

```bash
rostopic pub --once /franka_gripper/stop/goal franka_gripper/StopActionGoal "{}"
```

Contact forces disappear as no force is applied. Alternatively, use the `move` action.

### Customization

The `franka_gazebo` launch file supports customization. To spawn two Pandas:

```xml
<?xml version="1.0"?>
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="true"/>
    <arg name="use_sim_time" value="true"/>
  </include>
  <group ns="panda_1">
    <include file="$(find franka_gazebo)/launch/panda.launch">
      <arg name="arm_id" value="panda_1"/>
      <arg name="y" value="-0.5"/>
      <arg name="controller" value="cartesian_impedance_example_controller"/>
      <arg name="rviz" value="false"/>
      <arg name="gazebo" value="false"/>
      <arg name="paused" value="true"/>
    </include>
  </group>
  <group ns="panda_2">
    <include file="$(find franka_gazebo)/launch/panda.launch">
      <arg name="arm_id" value="panda_2"/>
      <arg name="y" value="0.5"/>
      <arg name="controller" value="force_example_controller"/>
      <arg name="rviz" value="false"/>
      <arg name="gazebo" value="false"/>
      <arg name="paused" value="false"/>
    </include>
  </group>
</launch>
```

> **Note**: Check supported arguments with: `roslaunch franka_gazebo panda.launch --rosargs`.

### FrankaHWSim

The `FrankaHWSim` plugin enables Gazebo to support Franka-specific interfaces. Declare it in the URDF:

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>$(arm_id)</robotNamespace>
    <controlPeriod>0.001</controlPeriod>
    <robotSimType>franka_gazebo/FrankaHWSim</robotSimType>
  </plugin>
  <self_collide>true</self_collide>
</gazebo>
```

Supported interfaces:

| Status | Interface | Function |
|--------|-----------|----------|
| ✓ | `hardware_interface::JointStateInterface` | Reads joint states. |
| ✓ | `hardware_interface::EffortJointInterface` | Commands joint-level torques and reads joint states. |
| ✓ | `hardware_interface::VelocityJointInterface` | Commands joint velocities and reads joint states. |
| ✓ | `hardware_interface::PositionJointInterface` | Commands joint positions and reads joint states. |
| ✓ | `franka_hw::FrankaStateInterface` | Reads the full robot state. |
| ✓ | `franka_hw::FrankaModelInterface` | Reads the dynamic and kinematic model of the robot. |
| ✗ | `franka_hw::FrankaPoseCartesianInterface` | Commands Cartesian poses and reads the full robot state. |
| ✗ | `franka_hw::FrankaVelocityCartesianInterface` | Commands Cartesian velocities and reads the full robot state. |

> **Important**: Only controllers claiming supported interfaces can be simulated. For example, the Cartesian Impedance Example Controller is supported, but the Joint Impedance Example Controller is not due to its dependency on `FrankaPoseCartesianInterface`.

Supported non-realtime commands:

| Status | Service / Type | Explanation |
|--------|----------------|-------------|
| ✗ | `set_joint_impedance / SetJointImpedance` | Gazebo does not simulate an internal impedance controller; commanded torques are set directly. |
| ✗ | `set_cartesian_impedance / SetCartesianImpedance` | Gazebo does not simulate an internal impedance controller; commanded torques are set directly. |
| ✓ | `set_EE_frame / SetEEFrame` | Sets ${NE}_T_{EE}$, the transformation from nominal end-effector to end-effector. Can be initialized via ROS parameter `/<arm_id>/NE_T_EE`. |
| ✓ | `set_K_frame / SetKFrame` | Sets ${EE}_T_K$, the transformation from end-effector to stiffness frame. |
| ✓ | `set_force_torque_collision_behavior / SetForceTorqueCollisionBehavior` | Sets thresholds for external wrenches treated as contacts and collisions. |
| ✗ | `set_full_collision_behavior / SetFullCollisionBehavior` | Not yet implemented. |
| ✓ | `set_load / SetLoad` | Sets an external load for gravity compensation. Can be initialized via ROS parameters `/<arm_id>/(m_load, I_load, F_x_load)`. |
| ✓ | `set_user_stop / std_srvs::SetBool` (since 0.9.1) | Simulates the user stop, disconnecting command signals. Reconnect via the `error_recovery` action. |

### FrankaGripperSim

The `FrankaGripperSim` plugin simulates the `franka_gripper_node` in Gazebo as a ROS controller for the two finger joints with a position and force controller. It offers the same actions as the real gripper node:

- `/<arm_id>/franka_gripper/homing`
- `/<arm_id>/franka_gripper/stop`
- `/<arm_id>/franka_gripper/move`
- `/<arm_id>/franka_gripper/grasp`
- `/<arm_id>/franka_gripper/gripper_action`

> **Important**: The `grasp` action has a bug preventing success or abortion if the target width opens the fingers, due to missing joint limits causing oscillation. Use `grasp` only to close the fingers.

Required parameters:

- `type`: `franka_gazebo/FrankaGripperSim`
- `arm_id`: Infers finger joint names.
- `finger1/gains/p`, `finger2/gains/p`: Proportional gains (required).
- `finger1/gains/i`, `finger1/gains/d`, `finger2/gains/i`, `finger2/gains/d`: Integral and differential gains (default: 0).
- `move/width_tolerance`: Success threshold for move action (default: 5 mm).
- `grasp/resting_threshold`: Speed threshold for grasp evaluation (default: 1 mm/s).
- `grasp/consecutive_samples`: Number of consecutive samples below threshold (default: 3).
- `gripper_action/width_tolerance`: Success threshold for gripper action (default: 5 mm).
- `gripper_action/speed`: Speed for gripper action (default: 10 cm/s).

### JointStateInterface

Declare a joint in a transmission tag in the URDF to access the `JointStateInterface`:

```xml
<transmission name="${joint}_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="${joint}">
    <hardwareInterface>hardware_interface/JointStateInterface</hardwareInterface>
  </joint>
</transmission>
```

> **Note**: For joints named `<arm_id>_jointN`, `FrankaHWSim` automatically compensates gravity to mimic libfranka.

### EffortJointInterface

To send effort commands, declare a transmission tag:

```xml
<transmission name="${joint}_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="${joint}">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="${joint}_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </actuator>
</transmission>
<gazebo reference="${joint}">
  <provideFeedback>true</provideFeedback>
</gazebo>
```

> **Note**: Set `<provideFeedback>true</provideFeedback>` to read external forces or torques from collisions.

### FrankaStateInterface

Declare a transmission tag with all seven joints:

```xml
<transmission name="${arm_id}_franka_state">
  <type>franka_hw/FrankaStateInterface</type>
  <joint name="${arm_id}_joint1"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></joint>
  <joint name="${arm_id}_joint2"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></joint>
  <joint name="${arm_id}_joint3"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></joint>
  <joint name="${arm_id}_joint4"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></joint>
  <joint name="${arm_id}_joint5"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></joint>
  <joint name="${arm_id}_joint6"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></joint>
  <joint name="${arm_id}_joint7"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></joint>
  <actuator name="${arm_id}_motor1"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></actuator>
  <actuator name="${arm_id}_motor2"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></actuator>
  <actuator name="${arm_id}_motor3"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></actuator>
  <actuator name="${arm_id}_motor4"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></actuator>
  <actuator name="${arm_id}_motor5"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></actuator>
  <actuator name="${arm_id}_motor6"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></actuator>
  <actuator name="${arm_id}_motor7"><hardwareInterface>franka_hw/FrankaStateInterface</hardwareInterface></actuator>
</transmission>
```

Simulated `RobotState` fields:

| Status | Field | Comment |
|--------|-------|---------|
| ✓ | `O_T_EE` | |
| ✗ | `O_T_EE_d` | Motion generation not supported; contains zeros. |
| ✓ | `F_T_EE` | Configurable via `F_T_NE`, `NE_T_EE`, or `set_EE_frame`. |
| ✓ | `NE_T_EE` | Configurable via `NE_T_EE` or `set_EE_frame`. |
| ✓ | `EE_T_K` | Configurable via `EE_T_K` or `set_K_frame`. |
| ✓ | `m_ee` | Set from URDF inertial tag if hand is found; otherwise zero. Overwritable by `m_ee`. |
| ✓ | `I_ee` | Set from URDF inertial tag if hand is found; otherwise zero. Overwritable by `I_ee`. |
| ✓ | `F_x_Cee` | Set from URDF inertial tag origin if hand is found; otherwise zero. Overwritable by `F_x_Cee`. |
| ✓ | `m_load` | Configurable via `m_load` or `set_load`. |
| ✓ | `I_load` | Configurable via `I_load` or `set_load`. |
| ✓ | `F_x_Cload` | Configurable via `F_x_Cload` or `set_load`. |
| ✓ | `m_total` | |
| ✓ | `I_total` | |
| ✓ | `F_x_ctotal` | |
| ✗ | `elbow` | |
| ✗ | `elbow_d` | |
| ✗ | `elbow_c` | |
| ✗ | `delbow_d` | |
| ✗ | `delbow_c` | |
| ✓ | `tau_J` | From Gazebo. |
| ✓ | `tau_J_d` | Values sent by effort controller; zero otherwise. |
| ✓ | `dtau_d` | Numerical derivative of `tau_J`. |
| ✓ | `q` | From Gazebo. |
| ✓ | `q_d` | Last commanded joint position (position interface); same as `q` (velocity interface); not updated (effort interface). |
| ✓ | `dq` | From Gazebo. |
| ✓ | `dq_d` | Last commanded joint velocity (velocity interface); same as `q` (position interface); zero (effort interface). |
| ✓ | `ddq_d` | Current acceleration (position/velocity interface); zero (effort interface). |
| ✓ | `joint_contact` | $|\hat{\tau}_{\text{ext}}| > \text{thresh}_{\text{lower}}$, set by `set_force_torque_collision_behavior`. |
| ✓ | `joint_collision` | $|\hat{\tau}_{\text{ext}}| > \text{thresh}_{\text{upper}}$, set by `set_force_torque_collision_behavior`. |
| ✓ | `cartesian_contact` | $|K \hat{F}_{K,\text{ext}}| > \text{thresh}_{\text{lower}}$, set by `set_force_torque_collision_behavior`. |
| ✓ | `cartesian_collision` | $|K \hat{F}_{K,\text{ext}}| > \text{thresh}_{\text{upper}}$, set by `set_force_torque_collision_behavior`. |
| ✓ | `tau_ext_hat_filtered` | $\hat{\tau}_{\text{ext}} = \tau_J - \tau_{J_d} - \tau_{\text{gravity}}$, filtered with EMA (configurable via ROS parameter). |
| ✓ | `O_F_ext_hat_K` | $O \hat{F}_{K,\text{ext}} = J_O^{\top+} \hat{\tau}_{\text{ext}}$. |
| ✓ | `K_F_ext_hat_K` | $K \hat{F}_{K,\text{ext}} = J_K^{\top+} \hat{\tau}_{\text{ext}}$. |
| ✗ | `O_dP_EE_d` | |
| ✓ | `O_ddP_D` | Same as `gravity_vector` ROS parameter; default: `{0, 0, -9.8}`. |
| ✗ | `O_T_EE_c` | |
| ✗ | `O_dP_EE_c` | |
| ✗ | `O_ddP_EE_c` | |
| ✓ | `theta` | Same as `q` (no soft joints in Gazebo). |
| ✓ | `dtheta` | Same as `dq` (no soft joints in Gazebo). |
| ✗ | `current_errors` | Always false; reflex system not implemented. |
| ✗ | `last_motion_errors` | Always false; reflex system not implemented. |
| ✓ | `control_command_success_rate` | Always 1.0. |
| ✗ | `robot_mode` | Robot mode switches and reflex system not implemented. |
| ✓ | `time` | Current ROS time in simulation, from Gazebo. |

### FrankaModelInterface

Declare a transmission tag with the root and tip of the kinematic chain:

```xml
<transmission name="${arm_id}_franka_model">
  <type>franka_hw/FrankaModelInterface</type>
  <joint name="${root}">
    <role>root</role>
    <hardwareInterface>franka_hw/FrankaModelInterface</hardwareInterface>
  </joint>
  <joint name="${tip}">
    <role>tip</role>
    <hardwareInterface>franka_hw/FrankaModelInterface</hardwareInterface>
  </joint>
  <actuator name="${root}_motor_root">
    <hardwareInterface>franka_hw/FrankaModelInterface</hardwareInterface>
  </actuator>
  <actuator name="${tip}_motor_tip">
    <hardwareInterface>franka_hw/FrankaModelInterface</hardwareInterface>
  </actuator>
</transmission>
```

Model functions are implemented with KDL, using the kinematic structure and inertial properties from the URDF to calculate properties like the Jacobian or mass matrix.

### Friction

For proper friction (e.g., between fingers and objects), tune Gazebo parameters in the URDF for `panda_finger/joint1` and `panda_finger/joint2`:

```xml
<gazebo reference="${link}">
  <collision>
    <max_contacts>10</max_contacts>
    <surface>
      <contact>
        <ode>
          <max_vel>0</max_vel>
          <min_depth>0.003</min_depth>
        </ode>
      </contact>
      <friction>
        <ode>
          <mu>1.16</mu>
          <mu2>1.16</mu2>
        </ode>
      </friction>
      <bounce/>
    </surface>
  </collision>
</gazebo>
```

> **Note**: Refer to the [Gazebo Friction Documentation](http://gazebosim.org/tutorials?tut=friction).

## franka_msgs

This package contains message, service, and action types used by `franka_hw` and `franka_control` to publish robot states or expose the libfranka API in the ROS ecosystem. See `franka_control` for details on services and actions.

## panda_moveit_config

> **Note**: This package has been moved to the [ros_planning repositories](https://github.com/ros-planning). For details, documentation, and tutorials, refer to the [MoveIt tutorials website](https://moveit.ros.org/documentation/).

## Writing Your Own Controller

Example controllers in `franka_example_controllers` derive from `controller_interface::MultiInterfaceController`, allowing up to four interfaces. The class declaration is:

```cpp
class NameOfYourControllerClass : public controller_interface::MultiInterfaceController<
    my_mandatory_first_interface,
    my_possible_second_interface,
    my_possible_third_interface,
    my_possible_fourth_interface> {
public:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& nh) override; // mandatory
  void update(const ros::Time& time, const ros::Duration& period) override; // mandatory
  void starting(const ros::Time& time) override; // optional
  void stopping(const ros::Time& time) override; // optional
  // ...
};
```

Available interfaces are described in the `franka_hw` section. Possible command interface claims for `franka_hw::FrankaHw` and `franka_combinable_hw::FrankaCombinableHw` are:

- Single interface claims.
- `EffortJointInterface + PositionJointInterface`
- `EffortJointInterface + VelocityJointInterface`
- `EffortJointInterface + FrankaCartesianPoseInterface`
- `EffortJointInterface + FrankaCartesianVelocityInterface`

Read-only interfaces (`JointStateInterface`, `FrankaStateInterface`, `FrankaModelInterface`) can always be claimed. Combining `EffortJointInterface` with a motion generator interface exposes internal motion generators, allowing use of the robot's inverse kinematics. For example, claim `EffortJointInterface + FrankaCartesianPoseInterface` to stream a Cartesian trajectory and compute joint-level torques based on the desired joint pose (`q_d`) from the robot state.

Implement at least the `init` and `update` functions. Use `starting` for initialization (e.g., start poses), as it is called when restarting the controller, while `init` is called only once. Use `stopping` for shutdown-related functionality.

> **Important**: Command a gentle slowdown before shutting down. For velocity interfaces, do not command zero velocity in `stopping`, as this could cause a velocity jump, leading to high joint torques. Maintain the current velocity and let the robot handle the slowdown.

Export the controller with pluginlib:

```cpp
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    name_of_your_controller_package::NameOfYourControllerClass,
    controller_interface::ControllerBase)
```

Define a `plugin.xml` file:

```xml
<library path="lib/libname_of_your_controller_library">
  <class name="name_of_your_controller_package/NameOfYourControllerClass"
         type="name_of_your_controller_package::NameOfYourControllerClass"
         base_class_type="controller_interface::ControllerBase">
    <description>
      Some text to describe what your controller is doing
    </description>
  </class>
</library>
```

Export it in `package.xml`:

```xml
<export>
  <controller_interface plugin="${prefix}/plugin.xml"/>
</export>
```

Load the controller with a configuration file:

```yaml
your_custom_controller_name:
  type: name_of_your_controller_package/NameOfYourControllerClass
  additional_example_parameter: 0.0
```

Start the controller using `controller_spawner` or service calls from `controller_manager`, ensuring `controller_spawner` and `franka_control_node` run in the same namespace. Refer to `franka_example_controllers` or [ROS control tutorials](http://wiki.ros.org/ros_control) for details.