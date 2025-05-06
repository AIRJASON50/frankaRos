# libfranka

Before continuing with this chapter, please install or compile libfranka for Linux.

API documentation for the latest version of libfranka is available at [https://frankaemika.github.io/libfranka](https://frankaemika.github.io/libfranka).

The libfranka changelog is available at [CHANGELOG.md](CHANGELOG.md).

## Schematic Overview

Libfranka is the C++ implementation of the client side of the Franka Control Interface (FCI). It handles network communication with the Control and provides interfaces to easily:

- Execute non-realtime commands to control the Hand and configure Arm parameters.
- Execute realtime commands to run your own 1 kHz control loops.
- Read the robot state to get sensor data at 1 kHz.
- Access the model library to compute desired kinematic and dynamic parameters.

During operation, you might encounter several errors, which are detailed at the end of this section.

## Non-Realtime Commands

Non-realtime commands are blocking, TCP/IP-based, and always executed outside of any realtime control loop. They encompass all Hand commands and some configuration-related commands for the Arm.

### Non-Realtime Commands for Arm and Hand

The most relevant commands for the Hand are:

- `homing`: Calibrates the maximum grasping width of the Hand.
- `move`, `grasp`, and `stop`: To move or grasp with the Hand.
- `readOnce`: Reads the Hand state.

For the Arm, some useful non-realtime commands are:

- `setCollisionBehavior`: Sets the contact and collision detection thresholds.
- `setCartesianImpedance` and `setJointImpedance`: Set the impedance parameters for the internal Cartesian impedance and joint impedance controllers.
- `setEE`: Sets the transformation $NE_T_EE$ from nominal end effector to end effector frame. The transformation from flange to end effector frame $F_T_EE$ is split into two transformations: $F_T_NE$ and $NE_T_EE$. The transformation from flange to nominal end effector frame $F_T_NE$ can only be set through the administrator's interface.
- `setK`: Sets the transformation $EE_T_K$ from end effector frame to stiffness frame.
- `setLoad`: Sets the dynamic parameters of a payload.
- `automaticErrorRecovery`: Clears any command or control exception that previously occurred in the robot.

For a complete list, check the API documentation for the [Robot](https://frankaemika.github.io/libfranka) or [Gripper](https://frankaemika.github.io/libfranka).

All operations (non-realtime or realtime) on the Arm or Hand are performed through the `franka::Robot` and `franka::Gripper` objects, respectively. A connection to the Arm/Hand is established when the object is created:

```cpp
#include <franka/robot.h>
#include <franka/gripper.h>
...
franka::Gripper gripper("<fci-ip>");
franka::Robot robot("<fci-ip>");
```

The address can be passed either as a hostname or an IP address. In case of errors (e.g., networking issues or conflicting library versions), an exception of type `franka::Exception` will be thrown. To use multiple robots simultaneously, create multiple objects with appropriate IP addresses.

To run a specific command, call the corresponding method, e.g.:

```cpp
gripper.homing();
robot.automaticErrorRecovery();
```

## Realtime Commands

Realtime commands are UDP-based and require a 1 kHz connection to the Control. There are two types of realtime interfaces:

- **Motion generators**: Define robot motion in joint or Cartesian space.
- **Controllers**: Define the torques sent to the robot joints.

There are four types of external motion generators and three types of controllers (one external and two internal).

### Realtime Interfaces: Motion Generators and Controllers

You can command:

- Only a motion generator, using one of the two internal controllers to follow the commanded motion.
- Only an external controller, ignoring motion generator signals (i.e., torque control only).
- A motion generator and an external controller to use the inverse kinematics of Control in your external controller.

All realtime loops (motion generator or controller) are defined by a callback function that receives the robot state and the duration of the last cycle (1 ms unless packet losses occur) and returns the specific interface type. The `control` method of the `franka::Robot` class runs the control loop by executing the callback at 1 kHz, as shown below:

```cpp
std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
    my_external_controller = ...; // Define your external controller
robot.control(my_external_controller);
```

The following example uses the joint velocity motion generator interface:

```cpp
robot.control(
    [=, &time](const franka::RobotState&, franka::Duration period) -> franka::JointVelocities {
        time += period.toSec();
        double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
        double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
        franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};
        if (time >= 2 * time_max) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(velocities);
        }
        return velocities;
    });
```

This commands joint velocities to the last four joints, moving them by approximately ±12 degrees. After `2 * time_max` seconds, it returns a `MotionFinished` flag to stop the control loop.

If only a motion generator is used, the default controller is the internal joint impedance controller. You can switch to the internal Cartesian impedance controller by setting the optional argument:

```cpp
// Set joint impedance (optional)
robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});
// Runs with default joint impedance controller
robot.control(my_external_motion_generator_callback);
// Explicitly specify joint impedance
robot.control(my_external_motion_generator_callback, franka::ControllerMode::kJointImpedance);
// Set Cartesian impedance (optional)
robot.setCartesianImpedance({{2000, 2000, 2000, 100, 100, 100}});
// Runs with Cartesian impedance controller
robot.control(my_external_motion_generator_callback, franka::ControllerMode::kCartesianImpedance);
```

For an external controller, the following example commands zero torque for each joint (gravity is compensated by the robot):

```cpp
robot.control([&](const franka::RobotState&, franka::Duration) -> franka::Torques {
    return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
});
```

Examples for all interfaces and combinations are available in the libfranka examples. Before running, ensure the robot has enough free space to avoid collisions. For example, to run the joint velocity motion example:

```bash
./examples/generate_joint_velocity_motion <fci-ip>
```

**Warning**: For custom motion generators or controllers, ensure smooth signals to avoid discontinuity errors or instability. Check interface specifications before starting.

## Signal Processing

To handle non-ideal network connections, libfranka includes signal processing functions to ensure commanded values conform to interface limits:

- **Low-pass filter**: A first-order filter smooths the user-commanded signal. Since version 0.5.0, it runs by default with a 100 Hz cutoff frequency.
- **Rate limiter**: Limits the time derivatives of commanded values (e.g., acceleration, jerk for motion generators; torque rate for controllers). Since version 0.4.0, it runs by default to increase control loop robustness.

**Caution**: Rate limiting ensures no limits are violated except for joint limits after inverse kinematics, which may trigger errors like `cartesian_motion_generator_joint_*`.

**Hint**: Rate limiter limits are defined in `franka/rate_limiting.h`. If behavior is jerky, lower the limits, activate the low-pass filter, or reduce its cutoff frequency.

To control signal processing, `robot.control()` accepts two optional parameters: a flag to enable/disable the rate limiter and the low-pass filter cutoff frequency (≥1000 Hz deactivates the filter):

```cpp
// With rate limiting and 100 Hz low-pass filter
robot.control(my_external_motion_generator_callback, franka::ControllerMode::kCartesianImpedance, true, 100.0);
// Without rate limiting and low-pass filter
robot.control(my_external_motion_generator_callback, franka::ControllerMode::kCartesianImpedance, false, 1000.0);
// External controller without rate limiting but with 100 Hz low-pass filter
robot.control(my_external_controller, false);
// External controller without rate limiting and low-pass filter
robot.control(my_external_controller, false, 1000.0);
```

**Danger**: The low-pass filter and rate limiter are robustness features for packet losses. For initial tests, deactivate them to avoid instabilities from filtering nonsmooth signals. Monitor `control_command_success_rate` in the robot state to check communication quality.

## Under the Hood

The behavior of the full control loop, including the Control side, is detailed below.

### Motion Generators

Motion generator commands have the subscript $c$ (commanded). The Robot Kinematics completion block computes forward/inverse kinematics, yielding desired signals (subscript $d$). Internal controllers generate torques $\tau_d$ to track these signals:

- Joint impedance controller: Tracks joint signals $q_d, \dot{q}_d$.
- Cartesian impedance controller: Tracks Cartesian signals ${}^O T_{EE,d}, {}^O \dot{P}_{EE,d}$.

All Control-side variables (last received $c$ values, computed $d$ values, and derivatives) are sent back in the robot state, enabling use of inverse kinematics in external controllers and full transparency.

**Hint**: For joint motion generators, commanded and desired joint values are equivalent ($q_d = q_c$). For Cartesian motion generators, desired signals may differ from commanded signals to avoid singularities. Both $c$ and $d$ signals are available in the robot state.

### External Controller

External controller torques $\tau_d$ are fed directly to the robot joints, with compensation for gravity and motor friction:

$$
\tau_c = \tau_d + \tau_f + \tau_g
$$

Where:
- $\tau_d$: Desired torque from the user.
- $\tau_c$: Torque commanded to the joint.
- $\tau_f$: Torque to compensate motor friction.
- $\tau_g$: Torque to compensate gravity of the kinematic chain.

**Packet Losses**: If the control loop takes too long (<300 μs, depending on network/PC configuration) or the connection is poor, packet losses may occur. Control extrapolates signals using a constant acceleration or torque model. If ≥20 packets are lost, the loop stops with a `communication_constraints_violation` exception.

**Hint**: Compare commanded values with robot state values to check for filtering or extrapolation. After an exception, check the `franka::ControlException::log` member.

## Robot State

The robot state provides sensor readings and estimated values at 1 kHz, including:

- **Joint-level signals**: Motor and estimated joint angles, derivatives, joint torque, external torque, collision/contacts.
- **Cartesian-level signals**: Pose, end effector/load parameters, external wrench, collision.
- **Interface signals**: Last commanded and desired values and derivatives.

For a complete list, see the [franka::RobotState API](https://frankaemika.github.io/libfranka). The robot state is an input to all control loop callback functions. To read without controlling, use `read` or `readOnce`:

```cpp
franka::RobotState state = robot.readOnce();
```

To continuously read the robot state:

```cpp
size_t count = 0;
robot.read([&count](const franka::RobotState& robot_state) {
    std::cout << robot_state << std::endl;
    return count++ < 100;
});
```

## Model Library

The robot model library provides:

- Forward kinematics of all robot joints.
- Body and zero Jacobian matrices of all robot joints.
- Dynamic parameters: Inertia matrix, Coriolis/centrifugal vector, gravity vector.

The model library can compute parameters for arbitrary robot states, not just the current one, and can be used in non-realtime contexts (e.g., optimization loops). Examples for computing joint poses or Jacobians are included in the libfranka examples.

## Errors

Errors may occur due to noncompliant commands, communication issues, or robot behavior. The most relevant ones are detailed below. For a complete list, see the [Errors API documentation](https://frankaemika.github.io/libfranka).

**Hint**: After an error, use `franka::Robot::automaticErrorRecovery()` to clear it programmatically. Check the exception string to ensure the error is not critical. Some errors can be cleared manually via the external activation device or Desk's error recovery button.

### Errors Due to Noncompliant Commanded Values

- **Initial value errors**:
  - `joint_motion_generator_start_pose_invalid`
  - `cartesian_position_motion_generator_start_pose_invalid`
  - `cartesian_motion_generator_start_elbow_invalid`
  - `cartesian_motion_generator_elbow_sign_inconsistent`

  These indicate a discrepancy between current robot values and initial commanded values. To fix, start with the last commanded value from the robot state:

  ```cpp
  double time{0.0};
  robot.control(
      [=, &time](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
          time += period.toSec();
          if (time == 0) {
              return franka::JointPositions(robot_state.q_c);
          } else {
              // Rest of control loop
          }
      });
  ```

- **Position limit violations**:
  - `joint_motion_generator_position_limits_violation`: Ensure commanded values are within joint limits.
  - `cartesian_motion_generator_joint_position_limits_violation`: Triggered if inverse kinematics yields out-of-limits joint configurations.

- **Velocity and discontinuity errors**:
  - Joint motion generator:
    - `joint_motion_generator_velocity_limits_violation`
    - `joint_motion_generator_velocity_discontinuity` (acceleration limit violation)
    - `joint_motion_generator_acceleration_discontinuity` (jerk limit violation)
  - Cartesian motion generator:
    - `cartesian_motion_generator_velocity_limits_violation`
    - `cartesian_motion_generator_velocity_discontinuity`
    - `cartesian_motion_generator_acceleration_discontinuity`
    - `cartesian_motion_generator_joint_velocity_limits_violation`
    - `cartesian_motion_generator_joint_velocity_discontinuity`
    - `cartesian_motion_generator_joint_acceleration_discontinuity`

  Control computes derivatives using backwards Euler:

  - Velocity: $\dot{q}_{c,k} = \frac{q_{c,k} - q_{c,k-1}}{\Delta t}$
  - Acceleration: $\ddot{q}_{c,k} = \frac{\dot{q}_{c,k} - \dot{q}_{c,k-1}}{\Delta t}$
  - Jerk: $\dddot{q}_{c,k} = \frac{\ddot{q}_{c,k} - \ddot{q}_{c,k-1}}{\Delta t}$

  where $\Delta t = 0.001$. Previous values ($q_d, \dot{q}_d, \ddot{q}_d$) are available in the robot state for advance computation.

- **Torque rate limit violation**:
  - `controller_torque_discontinuity`: Triggered if the torque rate $\dot{\tau}_{d,k} = \frac{\tau_{d,k} - \tau_{d,k-1}}{\Delta t}$ exceeds limits. Previous torque $\tau_d$ is available in the robot state.

**Hint**: Rate limiters (since version 0.4.0) ensure compliance with all limits except joint limits after inverse kinematics. Check `franka/rate_limiting.h` and `src/rate_limiting.cpp` for examples. Ensure signals are smooth before enabling rate limiting to avoid instability.

### Errors Due to Communication Problems

If Control does not receive packets for 20 cycles (20 ms), a `communication_constraints_violation` error occurs. Intermittent packet drops may trigger discontinuity errors. Check the troubleshooting section and consider enabling signal processing functions.

### Behavioral Errors

**Warning**: These monitoring features are not safety-compliant and are intended for research purposes only.

- **Reflex errors**: If estimated external torques $\hat{\tau}_{\text{ext}}$ or forces ${}^O \hat{F}_{\text{ext}}$ exceed thresholds, `cartesian_reflex` or `joint_reflex` errors are triggered. Configure thresholds with `franka::Robot::setCollisionBehavior`.

  **Hint**: For environmental contact, increase collision thresholds to avoid reflex triggers. Fast motions may trigger reflexes if thresholds are low, as external torques/forces are estimated and may be inaccurate during high acceleration. Monitor $\hat{\tau}_{\text{ext}}$ and ${}^O \hat{F}_{\text{ext}}$ in the robot state.

- **Self-collision avoidance**: A `self_collision_avoidance_violation` error occurs if the robot nears a self-collision configuration.

  **Warning**: This does not guarantee prevention of self-collisions, especially at high speeds with the torque interface.

- **Torque sensor limit**: A `tau_j_range_violation` error occurs if the torque sensor limit is reached, aiming to prevent sensor damage.

- **Power limit**: A `power_limit_violation` error stops the robot if the maximum allowed power is reached.

- **Velocity limits**: `joint_velocity_violation` or `cartesian_velocity_violation` errors occur if joint or Cartesian velocity limits are exceeded.