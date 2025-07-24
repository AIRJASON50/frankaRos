# Robot and Interface Specifications

## Realtime Control Commands
Realtime control commands sent to the robot should fulfill recommended and necessary conditions. Recommended conditions should be fulfilled to ensure optimal operation of the robot. If necessary conditions are not met then the motion will be aborted.

The final robot trajectory is the result of processing the user-specified trajectory ensuring that recommended conditions are fulfilled. As long as necessary conditions are met, the robot will try to follow the user-provided trajectory but it will only match the final trajectory if it also fulfills recommended conditions. If the necessary conditions are violated, an error will abort the motion: if, for instance, the first point of the user defined joint trajectory is very different from robot start position (\( q(t=0) \neq q_c(t=0) \)) a start pose invalid error will abort the motion.

Values for the constants used in the equations below are shown in the [Limits for Panda](#limits-for-panda) and [Limits for Franka Research 3](#limits-for-franka-research-3) section.

## Joint Trajectory Requirements
### Necessary Conditions
- \( q_{\min} < q_c < q_{\max} \)
- \( -\dot{q}_{\max} < \dot{q}_c < \dot{q}_{\max} \)
- \( -\ddot{q}_{\max} < \ddot{q}_c < \ddot{q}_{\max} \)
- \( -\dddot{q}_{\max} < \dddot{q}_c < \dddot{q}_{\max} \)

### Recommended Conditions
- \( -\tau_{j_{\max}} < \tau_{j_d} < \tau_{j_{\max}} \)
- \( -\dot{\tau}_{j_{\max}} < \dot{\tau}_{j_d} < \dot{\tau}_{j_{\max}} \)

At the beginning of the trajectory, the following conditions should be fulfilled:
- \( q = q_c \)
- \( \dot{q}_c = 0 \)
- \( \ddot{q}_c = 0 \)

At the end of the trajectory, the following conditions should be fulfilled:
- \( \dot{q}_c = 0 \)
- \( \ddot{q}_c = 0 \)

## Cartesian Trajectory Requirements
### Necessary Conditions
- \( T \) is proper transformation matrix
- \( -\dot{p}_{\max} < \dot{p}_c < \dot{p}_{\max} \) (Cartesian velocity)
- \( -\ddot{p}_{\max} < \ddot{p}_c < \ddot{p}_{\max} \) (Cartesian acceleration)
- \( -\dddot{p}_{\max} < \dddot{p}_c < \dddot{p}_{\max} \) (Cartesian jerk)

### Conditions Derived from Inverse Kinematics
- \( q_{\min} < q_c < q_{\max} \)
- \( -\dot{q}_{\max} < \dot{q}_c < \dot{q}_{\max} \)
- \( -\ddot{q}_{\max} < \ddot{q}_c < \ddot{q}_{\max} \)

### Recommended Conditions
Conditions derived from inverse kinematics:
- \( -T_{j_{\max}} < T_{j_d} < T_{j_{\max}} \)
- \( -\dot{T}_{j_{\max}} < \dot{T}_{j_d} < \dot{T}_{j_{\max}} \)

At the beginning of the trajectory, the following conditions should be fulfilled:
- \( \sigma_{T_{EE}} = \sigma_{T_{EEc}} \)
- \( \dot{p}_c = 0 \) (Cartesian velocity)
- \( \ddot{p}_c = 0 \) (Cartesian acceleration)

At the end of the trajectory, the following conditions should be fulfilled:
- \( \dot{p}_c = 0 \) (Cartesian velocity)
- \( \ddot{p}_c = 0 \) (Cartesian acceleration)

## Controller Requirements
### Necessary Conditions
- \( -\dot{\tau}_{j_{\max}} < \tau_{j_d} < \dot{\tau}_{j_{\max}} \)

### Recommended Conditions
- \( -T_{j_{\max}} < T_{j_d} < T_{j_{\max}} \)

At the beginning of the trajectory, the following conditions should be fulfilled:
- \( \tau_{j_d} = 0 \)

## Limits for Panda
Limits in the Cartesian space are as follows:

| Name | Translation | Rotation | Elbow |
|------|-------------|----------|-------|
| \( \dot{p}_{\max} \) | \( 1.7 \, \frac{\mathrm{m}}{\mathrm{s}} \) | \( 2.5 \, \frac{\mathrm{rad}}{\mathrm{s}} \) | \( 2.1750 \, \frac{\mathrm{rad}}{\mathrm{s}} \) |
| \( \ddot{p}_{\max} \) | \( 13.0 \, \frac{\mathrm{m}}{\mathrm{s}^2} \) | \( 25.0 \, \frac{\mathrm{rad}}{\mathrm{s}^2} \) | \( 10.0 \, \frac{\mathrm{rad}}{\mathrm{s}^2} \) |
| \( \dddot{p}_{\max} \) | \( 6500.0 \, \frac{\mathrm{m}}{\mathrm{s}^3} \) | \( 12500.0 \, \frac{\mathrm{rad}}{\mathrm{s}^3} \) | \( 5000.0 \, \frac{\mathrm{rad}}{\mathrm{s}^3} \) |

Joint space limits are:

| Name | Joint 1 | Joint 2 | Joint 3 | Joint 4 | Joint 5 | Joint 6 | Joint 7 |
|------|---------|---------|---------|---------|---------|---------|---------|
| \( q_{\max} \) | 2.8973 | 1.7628 | 2.8973 | -0.0698 | 2.8973 | 3.7525 | 2.8973 |
| \( q_{\min} \) | -2.8973 | -1.7628 | -2.8973 | -3.0718 | -2.8973 | -0.0175 | -2.8973 |
| \( \dot{q}_{\max} \) | 2.1750 | 2.1750 | 2.1750 | 2.1750 | 2.6100 | 2.6100 | 2.6100 |
| \( \ddot{q}_{\max} \) | 15 | 7.5 | 10 | 12.5 | 15 | 20 | 20 |
| \( \dddot{q}_{\max} \) | 7500 | 3750 | 5000 | 6250 | 7500 | 10000 | 10000 |
| \( \tau_{j_{\max}} \) | 87 | 87 | 87 | 87 | 12 | 12 | 12 |
| \( \dot{\tau}_{j_{\max}} \) | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 |

The arm can reach its maximum extension when joint 4 has angle \( q_{\text{elbow-flip}} \), where \( q_{\text{elbow-flip}} = -0.467002423653011 \, \mathrm{rad} \). This parameter is used to determine the flip direction of the elbow.

## Limits for Franka Research 3
Limits in the Cartesian space are as follows:

| Name | Translation | Rotation | Elbow |
|------|-------------|----------|-------|
| \( \dot{p}_{\max} \) | \( 3.0 \, \frac{\mathrm{m}}{\mathrm{s}} \) | \( 2.5 \, \frac{\mathrm{rad}}{\mathrm{s}} \) | \( 2.620 \, \frac{\mathrm{rad}}{\mathrm{s}} \) |
| \( \ddot{p}_{\max} \) | \( 9.0 \, \frac{\mathrm{m}}{\mathrm{s}^2} \) | \( 17.0 \, \frac{\mathrm{rad}}{\mathrm{s}^2} \) | \( 10.0 \, \frac{\mathrm{rad}}{\mathrm{s}^2} \) |
| \( \dddot{p}_{\max} \) | \( 4500.0 \, \frac{\mathrm{m}}{\mathrm{s}^3} \) | \( 8500.0 \, \frac{\mathrm{rad}}{\mathrm{s}^3} \) | \( 5000.0 \, \frac{\mathrm{rad}}{\mathrm{s}^3} \) |

Joint space limits are:

| Name | Joint 1 | Joint 2 | Joint 3 | Joint 4 | Joint 5 | Joint 6 | Joint 7 |
|------|---------|---------|---------|---------|---------|---------|---------|
| \( q_{\max} \) | 2.7437 | 1.7837 | 2.9007 | -0.1518 | 2.8065 | 4.5169 | 3.0159 |
| \( q_{\min} \) | -2.7437 | -1.7837 | -2.9007 | -3.0421 | -2.8065 | 0.5445 | -3.0159 |
| \( \dot{q}_{\max} \) | 2.62 | 2.62 | 2.62 | 2.62 | 5.26 | 4.18 | 5.26 |
| \( \ddot{q}_{\max} \) | 10 | 10 | 10 | 10 | 10 | 10 | 10 |
| \( \dddot{q}_{\max} \) | 5000 | 5000 | 5000 | 5000 | 5000 | 5000 | 5000 |
| \( \tau_{j_{\max}} \) | 87 | 87 | 87 | 87 | 12 | 12 | 12 |
| \( \dot{\tau}_{j_{\max}} \) | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 |

The arm can reach its maximum extension when joint 4 has angle \( q_{\text{elbow-flip}} \), where \( q_{\text{elbow-flip}} = -0.467002423653011 \, \mathrm{rad} \). This parameter is used to determine the flip direction of the elbow.

### Important
Note that the maximum joint velocity depends on the joint position. The maximum and minimum joint velocities at a certain joint position are calculated as:

**Maximum Velocities:**
\[
\dot{q}_1(q_1)_{\max} = \min \left(2.62, \max \left(0, -0.30 + \sqrt{\max \left(0, 12.0 \cdot (2.75010 - q_1)\right)}\right)\right)
\]
\[
\dot{q}_2(q_2)_{\max} = \min \left(2.62, \max \left(0, -0.20 + \sqrt{\max \left(0, 5.17 \cdot (1.79180 - q_2)\right)}\right)\right)
\]
\[
\dot{q}_3(q_3)_{\max} = \min \left(2.62, \max \left(0, -0.20 + \sqrt{\max \left(0, 7.00 \cdot (2.90650 - q_3)\right)}\right)\right)
\]
\[
\dot{q}_4(q_4)_{\max} = \min \left(2.62, \max \left(0, -0.30 + \sqrt{\max \left(0, 8.00 \cdot (-0.1458 - q_4)\right)}\right)\right)
\]
\[
\dot{q}_5(q_5)_{\max} = \min \left(2.62, \max \left(0, -0.35 + \sqrt{\max \left(0, 34.0 \cdot (2.81010 - q_5)\right)}\right)\right)
\]
\[
\dot{q}_6(q_6)_{\max} = \min \left(4.18, \max \left(0, -0.35 + \sqrt{\max \left(0, 11.0 \cdot (4.52050 - q_6)\right)}\right)\right)
\]
\[
\dot{q}_7(q_7)_{\max} = \min \left(5.26, \max \left(0, -0.35 + \sqrt{\max \left(0, 34.0 \cdot (3.01960 - q_7)\right)}\right)\right)
\]

**Minimum Velocities:**
\[
\dot{q}_1(q_1)_{\min} = \max \left(-2.62, \min \left(0, 0.30 - \sqrt{\max \left(0, 12.0 \cdot (2.750100 + q_1)\right)}\right)\right)
\]
\[
\dot{q}_2(q_2)_{\min} = \max \left(-2.62, \min \left(0, 0.20 - \sqrt{\max \left(0, 5.17 \cdot (1.791800 + q_2)\right)}\right)\right)
\]
\[
\dot{q}_3(q_3)_{\min} = \max \left(-2.62, \min \left(0, 0.20 - \sqrt{\max \left(0, 7.00 \cdot (2.906500 + q_3)\right)}\right)\right)
\]
\[
\dot{q}_4(q_4)_{\min} = \max \left(-2.62, \min \left(0, 0.30 - \sqrt{\max \left(0, 8.00 \cdot (3.048100 + q_4)\right)}\right)\right)
\]
\[
\dot{q}_5(q_5)_{\min} = \max \left(-5.26, \min \left(0, 0.35 - \sqrt{\max \left(0, 34.0 \cdot (2.810100 + q_5)\right)}\right)\right)
\]
\[
\dot{q}_6(q_6)_{\min} = \max \left(-4.18, \min \left(0, 0.35 - \sqrt{\max \left(0, 11.0 \cdot (-0.54092 + q_6)\right)}\right)\right)
\]
\[
\dot{q}_7(q_7)_{\min} = \max \left(-5.26, \min \left(0, 0.35 - \sqrt{\max \left(0, 34.0 \cdot (3.019600 + q_7)\right)}\right)\right)
\]

In order to avoid violating the safety joint velocity limits, the Max/Min Joint velocity limits for FCI are more restrictive than those provided in the Datasheet.

As most motion planners cannot deal with those functions for describing the velocity limits of each joint but they only deal with fixed velocity limits (rectangular limits), we are providing here a suggestion on which values to use for them.

In the figures below the system velocity limits are visualized by the red and blue thresholds while the suggested "position-velocity rectangular limits" are visualized in black.

### Visualization of the Joint Limits of FR3
- Velocity limits of Joint 1
- Velocity limits of Joint 2
- Velocity limits of Joint 3
- Velocity limits of Joint 4
- Velocity limits of Joint 5
- Velocity limits of Joint 6
- Velocity limits of Joint 7

Here are the parameters describing the suggested position-velocity rectangular limits:

| Name | Joint 1 | Joint 2 | Joint 3 | Joint 4 | Joint 5 | Joint 6 | Joint 7 |
|------|---------|---------|---------|---------|---------|---------|---------|
| \( q_{\max} \) | 2.3093 | 1.5133 | 2.4937 | -0.4461 | 2.4800 | 4.2094 | 2.6895 |
| \( q_{\min} \) | -2.3093 | -1.5133 | -2.4937 | -2.7478 | -2.4800 | 0.8521 | -2.6895 |
| \( \dot{q}_{\max} \) | 2 | 1 | 1.5 | 1.25 | 3 | 1.5 | 3 |

### Important
These limits are the values that are used by default in the rate limiter and in the URDF inside franka Investigation accordingly to your needs.

Since FR3 does not inherently implement any restriction to the system limits (red and blue line in the plots above), you are also free to implement your own motion generator to exploit the HW capabilities of FR3 beyond the rectangular limits imposed by existing motion generators.

## Denavit-Hartenberg Parameters
The Denavit-Hartenberg parameters for the Franka Research 3 kinematic chain are derived following Craig's convention and are as follows:

| Joint | \( a \, (\mathrm{m}) \) | \( d \, (\mathrm{m}) \) | \( \alpha \, (\mathrm{rad}) \) | \( \theta \, (\mathrm{rad}) \) |
|-------|-------------------------|-------------------------|-------------------------------|-------------------------------|
| Joint 2 | 0 | 0 | \( -\frac{\pi}{2} \) | \( \theta_2 \) |
| Joint 3 | 0 | 0.316 | \( \frac{\pi}{2} \) | \( \theta_3 \) |
| Joint 4 | 0.0825 | 0 | \( \frac{\pi}{2} \) | \( \theta_4 \) |
| Joint 5 | -0.0825 | 0.384 | \( -\frac{\pi}{2} \) | \( \theta_5 \) |
| Joint 6 | 0 | 0 | \( \frac{\pi}{2} \) | \( \theta_6 \) |
| Joint 7 | 0.088 | 0 | \( \frac{\pi}{2} \) | \( \theta_7 \) |
| Flange | 0 | 0.107 | 0 | 0 |

### Note
\( {}^0 T_1 \) is the transformation matrix which describes the position and orientation of frame 1 in frame 0. A kinematic chain can be calculated like the following: \( {}^0 T_2 = {}^0 T_1 \cdot {}^1 T_2 \).