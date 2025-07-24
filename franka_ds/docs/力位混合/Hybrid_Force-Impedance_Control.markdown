# Hybrid Force-Impedance Control for Fast End-Effector Motions

## Abstract

Controlling the contact force on various surfaces is essential in many robotic applications such as in service tasks or industrial use cases. Mostly, classical impedance and hybrid motion-force control approaches are employed for these kinds of physical interaction scenarios. In this work, an extended Cartesian impedance control algorithm is developed, which includes geometrical constraints and enables explicit force tracking in a hybrid manner. The unified framework features compliant behavior in the free (motion) task directions and explicit force tracking in the non-specified process. The subspace in contact direction is fully dynamically decoupled from dynamics in the motion subspace. The experimental validation with a torque-controlled robotic manipulator on both flat and curved surfaces demonstrates the performance during highly dynamic desired trajectories and confirms the theoretical claims of the approach.

## I. Introduction

Well-defined physical interaction with the environment is an essential requirement for a vast number of real-world applications in robotics. Interaction control can be performed directly to realize desired time-dependent or constant contact forces or indirectly by specifying a dynamic behavior of the controlled robot during physical interaction, e.g., through impedance or admittance control. Hence, interaction forces indirectly result from active control of the physical parameters of the robot, i.e., inertia, damping, and stiffness.

Explicit force control is often desirable in specific task directions while precise motion control is required in other directions. That leads to hybrid position-force control structures, enabling a direct force control loop in constrained task directions. Commonly, the operational space is partitioned into motion and force sub-spaces to control the position in the unconstrained task direction. Analysis of standard force control methods and an approach that aims at achieving passivity during interaction by superimposing the motion and force control actions are presented in prior work.

In some manipulation tasks, it is beneficial to perform rapid motions while the robot is executing a specific interaction subtask simultaneously. Surface polishing, wiping, and finishing are typical tasks that could be performed in fast motion while regulating the desired contact force. However, this ability to sensitively regulate the contact behavior while moving fast requires incorporating the respective dynamic effects in the control design. Frequently in force control tasks, the robot is stationary or only moving slowly such that the dynamical terms that depend on velocity and acceleration can be neglected. This is no longer a valid assumption when fast tasks are performed that require accurate contact force control.

This letter focuses on the development of a force-impedance framework to exert well-defined interaction forces while the end-effector is performing fast motions in parallel. The formulation enables assigning constraints in the task space. It can also be interpreted as mixed direct and indirect force control as the proposed scheme features accurate force tracking in constrained directions and compliant behavior in the free motion directions. Moreover, the controller design is carried out in a model-based fashion including these constraints explicitly. The effect of the velocity-dependent couplings is investigated. These terms are usually neglected in the literature. Furthermore, the proposed approach provides a full decoupling of the control actions in the motion and force directions.

The letter is organized as follows. Section II introduces the system model to be used throughout this work, and an overview of the standard Cartesian impedance and force control approaches is provided. The proposed hybrid control framework is presented in Section III. Experimental results and validations of the control method compared with the conventional approach are shown and discussed in Section IV. Finally, Section V concludes this letter.

## II. Fundamentals

The rigid-body dynamics of a manipulator with \( n \) degrees of freedom (DOF) can be expressed as

\[
\boldsymbol{M}(\boldsymbol{q}) \ddot{\boldsymbol{q}} + \boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}} + \boldsymbol{g}(\boldsymbol{q}) = \tau + \tau^{\text{ext}}
\]

with the generalized joint coordinates \( \boldsymbol{q} \in \mathbb{R}^n \), the symmetric and positive definite inertia matrix \( \boldsymbol{M}(\boldsymbol{q}) \in \mathbb{R}^{n \times n} \), and the Coriolis and centrifugal matrix \( \boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \in \mathbb{R}^{n \times n} \). The term \( \boldsymbol{g}(\boldsymbol{q}) \in \mathbb{R}^n \) represents the generalized gravity forces originating from the gravity potential \( V_g(\boldsymbol{q}) \) through \( \boldsymbol{g}(\boldsymbol{q}) = \left( \frac{\partial V_g(\boldsymbol{q})}{\partial \boldsymbol{q}} \right)^T \). Furthermore, the quantities \( \tau, \tau^{\text{ext}} \in \mathbb{R}^n \) describe the generalized joint forces and external forces, respectively. The joint forces are assumed to be the control inputs in the following. The applied external wrench \( \boldsymbol{F}^{\text{ext}} \in \mathbb{R}^m \) is mapped to external joint torques through a Jacobian matrix \( \boldsymbol{J}(\boldsymbol{q}) \in \mathbb{R}^{m \times n} \) according to

\[
\tau^{\text{ext}} = -\boldsymbol{J}(\boldsymbol{q})^T \boldsymbol{F}^{\text{ext}}
\]

where \( m = 6 \) if the full Cartesian space is considered. The operational space velocity \( \dot{\boldsymbol{y}} \in \mathbb{R}^m \) is described by the mapping \( \dot{\boldsymbol{y}} = \boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}} \), which in the general case contains the end-effector translational and rotational velocities. Compliant end-effector behavior can be achieved using the classical impedance control

\[
\tau_{\text{imp}} = \boldsymbol{J}(\boldsymbol{q})^T \boldsymbol{F}_{\text{imp}} + \boldsymbol{g}(\boldsymbol{q})
\]

\[
\boldsymbol{F}_{\text{imp}} = -\left( \frac{\partial V(\tilde{\boldsymbol{y}})}{\partial \boldsymbol{y}} \right)^T - \boldsymbol{D}_{\boldsymbol{y}} \dot{\boldsymbol{y}}
\]

where \( \boldsymbol{F}_{\text{imp}} \) is the task-space wrench to realize a desired spring-damper behavior, and \( V(\tilde{\boldsymbol{y}}) \) is the respective positive definite potential function with the virtual spring deflection \( \tilde{\boldsymbol{y}} = \boldsymbol{y} - \boldsymbol{y}_{\text{des}} \), where \( \boldsymbol{y} \) and \( \boldsymbol{y}_{\text{des}} \in \mathbb{R}^m \) are the actual and desired tasks, respectively. The positive definite damping matrix \( \boldsymbol{D}_{\boldsymbol{y}} \in \mathbb{R}^{m \times m} \) is commonly obtained such that critical damping is achieved for varying configurations and different settings of the Cartesian stiffness. In case the robot is equipped with torque sensing capabilities, the joint torque feedback can be used to realize desired dynamics, e.g., to scale down the motor inertia, to provide a link-side torque interface, and vibration-damping.

In case of environmental constraints, directly controlling the interaction force is beneficial and desirable in many applications. The desired force can be applied simply through

\[
\tau = \tau_{\text{imp}} + \tau_{\text{f}}, \quad \tau_{\text{f}} = \boldsymbol{J}(\boldsymbol{q})^T \boldsymbol{F}^{\text{des}}
\]

or, as commonly using an explicit force feedback loop as

\[
\tau_{\text{f}} = \boldsymbol{J}(\boldsymbol{q})^T \left( \boldsymbol{F}^{\text{des}} - \boldsymbol{K}_{\text{fp}} \tilde{\boldsymbol{F}} - \boldsymbol{K}_{\text{fi}} \int_0^t \tilde{\boldsymbol{F}} \, dt \right)
\]

where \( \tilde{\boldsymbol{F}} = \boldsymbol{F}^{\text{ext}} - \boldsymbol{F}^{\text{des}} \in \mathbb{R}^m \) represents the end-effector wrench error, \( \boldsymbol{K}_{\text{fp}}, \boldsymbol{K}_{\text{fi}} \) are positive-definite proportional and integral gain matrices, respectively. Here, \( \boldsymbol{F}^{\text{ext}} \) is defined in the same direction as \( \boldsymbol{F}^{\text{des}} \), forces exerted by the robot on the environment. Furthermore, \( \tau_{\text{f}} \) is the generated joint torque to actively achieve the desired force. In principle, this provides an enhanced Cartesian force tracking and lower steady-state error due to the proportional-integral actions. The robot dynamic effects in the force control are often neglected since the robot typically moves in a very slow motion (or even stationary) during interaction tasks with specific force characteristics. Different approaches can be used to assign a pure force control in a specific task direction and remove the combined effect of impedance control as in the above equations.

## III. Control Design

Given sufficient knowledge of the environment geometry, the robot should be able to exert the desired interaction forces accurately even while performing highly dynamic motions. The task space of the end-effector shall have the dimension \( m \leq n \). In the case of the full Cartesian space, \( m = 6 \) including three translations and three rotations. Moreover, kinematic constraints of dimension \( c < m \) are imposed on this operational space. As the considered system is kinematically redundant w.r.t. the end-effector, an additional null space of dimension \( p = n - m \) exists which can be exploited to simultaneously execute complementary subtasks.

The kinematic, holonomic constraints \( \Phi(\boldsymbol{q}) \in \mathbb{R}^c \) at the end-effector can be formulated as

\[
\Phi(\boldsymbol{q}) = 0
\]

with \( \Phi(\boldsymbol{q}) \) being a function with continuous gradient and assumed to be twice differentiable. Without loss of generality, a rigid environment is considered. Due to the above equation, one can represent the constraint as

\[
\boldsymbol{J}_{\Phi}(\boldsymbol{q}) \dot{\boldsymbol{q}} = 0
\]

with \( \boldsymbol{J}_{\Phi}(\boldsymbol{q}) = \frac{\partial \Phi(\boldsymbol{q})}{\partial \boldsymbol{q}} \in \mathbb{R}^{c \times n} \). Let \( \boldsymbol{x}(\boldsymbol{q}) \in \mathbb{R}^{m-c} \) denote the remaining, unconstrained operational space coordinates with \( \boldsymbol{J}_{\boldsymbol{x}}(\boldsymbol{q}) = \frac{\partial \boldsymbol{x}(\boldsymbol{q})}{\partial \boldsymbol{q}} \in \mathbb{R}^{(m-c) \times n} \). According to the specification above, the local null space velocities \( \boldsymbol{v}(\boldsymbol{q}) \in \mathbb{R}^p \) can be derived using the standard task hierarchy framework through a Jacobian matrix \( \boldsymbol{J}_{\boldsymbol{v}}(\boldsymbol{q}) \in \mathbb{R}^{p \times n} \). The stacked Jacobian matrix \( \tilde{\boldsymbol{J}}(\boldsymbol{q}) \in \mathbb{R}^{n \times n} \) given by

\[
\begin{pmatrix}
\Phi \\
\dot{\boldsymbol{x}} \\
\boldsymbol{v}
\end{pmatrix}
=
\underbrace{\begin{pmatrix}
\boldsymbol{J}_{\Phi}(\boldsymbol{q}) \\
\boldsymbol{J}_{\boldsymbol{x}}(\boldsymbol{q}) \\
\boldsymbol{J}_{\boldsymbol{v}}(\boldsymbol{q})
\end{pmatrix}}_{\tilde{\boldsymbol{J}}(\boldsymbol{q})}
\dot{\boldsymbol{q}}
\]

which is assumed to be invertible in the considered workspace. Due to the constraint \( \dot{\Phi} = 0 \), solving the above for \( \dot{\boldsymbol{q}} \) yields

\[
\dot{\boldsymbol{q}} = \underbrace{\tilde{\boldsymbol{J}}(\boldsymbol{q})^{-1}}_{\boldsymbol{A}(\boldsymbol{q})} \underbrace{\begin{pmatrix}
0 &

\boldsymbol{I} & 0 \\
0 & \boldsymbol{I}
\end{pmatrix}}_{\boldsymbol{A}(\boldsymbol{q})} \begin{pmatrix}
\dot{\boldsymbol{x}} \\
\boldsymbol{v}
\end{pmatrix}
\]

where \( \boldsymbol{A}(\boldsymbol{q}) \in \mathbb{R}^{n \times (n-c)} \) with rank \( n-c \). The reformulation of the dynamics yields

\[
\tilde{\boldsymbol{M}} \begin{pmatrix}
\ddot{\boldsymbol{x}} \\
\dot{\boldsymbol{v}}
\end{pmatrix}
+ \tilde{\boldsymbol{C}} \begin{pmatrix}
\dot{\boldsymbol{x}} \\
\boldsymbol{v}
\end{pmatrix}
+ \tilde{\boldsymbol{g}} = \boldsymbol{A}^T \tau - \begin{pmatrix}
\boldsymbol{F}_{\boldsymbol{x}}^{\text{ext}} \\
\boldsymbol{F}_{\boldsymbol{v}}^{\text{ext}}
\end{pmatrix}
\]

where \( \tilde{\boldsymbol{M}} = \boldsymbol{A}^T \boldsymbol{M} \boldsymbol{A} \in \mathbb{R}^{(n-c) \times (n-c)} \) is the inertia matrix for the constrained system, \( \tilde{\boldsymbol{C}} = \boldsymbol{A}^T \boldsymbol{M} \dot{\boldsymbol{A}} + \boldsymbol{A}^T \boldsymbol{C} \boldsymbol{A} \) denotes the Coriolis and centrifugal matrix, and \( \tilde{\boldsymbol{g}} = \boldsymbol{A}^T \boldsymbol{g} \) represents the gravitational force. Note that \( \tau^{\text{ext}} \) has been rephrased to describe interactions related to the coordinates \( \Phi, \dot{\boldsymbol{x}}, \) and \( \boldsymbol{v} \) by the respective collocated, generalized forces:

\[
\tau^{\text{ext}} = -\tilde{\boldsymbol{J}}(\boldsymbol{q})^T \begin{pmatrix}
\boldsymbol{F}_{\Phi}^{\text{ext}} \\
\boldsymbol{F}_{\boldsymbol{x}}^{\text{ext}} \\
\boldsymbol{F}_{\boldsymbol{v}}^{\text{ext}}
\end{pmatrix}
\]

The system dynamics can be fully described by using the above equation, and by inserting the controller that leads to the expression of actuation forces as

\[
\boldsymbol{A}^T \tau = \begin{pmatrix}
\boldsymbol{F}_{\boldsymbol{x}}^{\text{ct}} \\
\boldsymbol{F}_{\boldsymbol{v}}^{\text{ct}}
\end{pmatrix}
+ \tilde{\boldsymbol{g}}
\]

For force tracking, the control must equal the desired force profile \( \boldsymbol{F}^{\text{des}}(t) \in \mathbb{R}^c \) which counteracts the interaction forces as

\[
\dot{\boldsymbol{F}}_{\Phi}^{\text{des}} - \boldsymbol{F}^{\text{des}}(t) = 0
\]

Substituting the control into the dynamics and inserting the controller delivers

\[
\boldsymbol{F}_{\Phi}^{\text{ct}} = +\boldsymbol{F}^{\text{des}}(t)
- \boldsymbol{\Lambda}_{\Phi} \boldsymbol{J}_{\Phi} \boldsymbol{M}^{-1} \left( \boldsymbol{J}_{\boldsymbol{x}}^T \boldsymbol{F}_{\boldsymbol{x}}^{\text{ext}} + \boldsymbol{J}_{\boldsymbol{v}}^T \boldsymbol{F}_{\boldsymbol{v}}^{\text{ext}} \right)
+ \boldsymbol{\Lambda}_{\Phi} \boldsymbol{J}_{\Phi} \boldsymbol{M}^{-1} \left( \boldsymbol{J}_{\Phi}^T \boldsymbol{F}_{\Phi}^{\text{ct}} + \boldsymbol{J}_{\boldsymbol{x}}^T \boldsymbol{F}_{\boldsymbol{x}}^{\text{ct}} \right)
+ \boldsymbol{\Lambda}_{\Phi} \left( \boldsymbol{J}_{\Phi} \boldsymbol{M}^{-1} \boldsymbol{C} - \dot{\boldsymbol{J}}_{\Phi} \right) \dot{\boldsymbol{q}}
\]

One of the main differences to state-of-the-art approaches for force tracking is the last component in the above equation, which is usually neglected in the control design and becomes particularly relevant for highly dynamic motions as shown in the experiments in Section IV. Note that in frictionless contacts and the absence of external forces in \( \dot{\boldsymbol{x}} \)- and \( \boldsymbol{v} \)-direction, the terms \( \boldsymbol{F}_{\boldsymbol{x}}^{\text{ext}} \) and \( \boldsymbol{F}_{\boldsymbol{v}}^{\text{ext}} \) vanish, which significantly simplifies the control feedback. Moreover, an additional proportional-integral term can support to reduce steady-state errors in the contact force. Therefore, the optional term

\[
\boldsymbol{F}_{\text{PI}} = -\boldsymbol{k}_{\text{P}} \left( \boldsymbol{F}_{\Phi}^{\text{ext}} - \boldsymbol{F}^{\text{des}}(t) \right) - \boldsymbol{k}_{\text{I}} \int \left( \boldsymbol{F}_{\Phi}^{\text{ext}} - \boldsymbol{F}^{\text{des}}(t) \right) dt
\]

with control gains \( \boldsymbol{k}_{\text{P}}, \boldsymbol{k}_{\text{I}} \) can be added to the previous control law. Feedback of \( \boldsymbol{F}_{\Phi}^{\text{ext}} \) can be easily implemented using a modern design of end-effector external wrench observer. Furthermore, the obtained control law is gain-free in contact-force direction, which is an advantage w.r.t. the tuning effort.

Usually, the force controller is not required at all times. It is only used when contact with the environment is already established. To have continuous and smooth force-impedance transitions, an intermediate desired value can be used such that the control input does not instantly change. This is done using activation variables changing continuously from 0 to 1 and specifying which controller is selected in a smooth manner. With sufficient geometrical knowledge about the environment, one can increase the gains of the impedance controller (set higher stiffness values) in the free motion directions as the interactions in those directions are less likely. In this way, better trajectory position tracking performance can be achieved at the end-effector level.

The proposed approach shares similar aims to the widely known category of hybrid force-motion control since the motion is controlled in the free task direction, and the force is controlled in the constrained directions of the task. Here, instead of using a stiff position controller, a Cartesian impedance controller is used such that the stiffness in the free directions is also parameterized.

## IV. Simulations and Experiments

The proposed hybrid controller is validated in simulations and experiments on a DLR-KUKA LWR IV + torque-controlled robot. The controller runs at a rate of 1 kHz, while the low joint-level torque controller is executed at 3 kHz. A Cartesian impedance controller is used to close the initial contact between the robotic end-effector and the constraint surface. In the proposed framework, both force- and impedance control are assigned to the same priority level. The desired task-space translational and rotational stiffness and damping values used in simulation and experiments are summarized in the following table:

| Parameter                | Simulation       | Experiments      |
|--------------------------|------------------|------------------|
| Translational Cartesian Impedance | 5000 [N/m], 500 [N/m] | 1500 [N/m], 300 [N/m] |
| Damping ratio            | 0.7              | 0.7              |

The interaction force could be computed from different sources, e.g., via joint-space external force observers or explicit force-torque sensors. Alternatively, an extended momentum-based observer could be deployed to obtain the contact force measurement in task space directly.

In Simulation #1, a task is assigned in which the manipulator performs fast motions in the \( x \)- \( y \)-plane (free directions) and exerts a desired contact force on the surface (normal/constrained direction). MATLAB/Simulink is used for the robot simulation. Theoretically speaking, the proposed approach shows zero error as full dynamic compensation is achieved. Additionally, the effect of velocity-dependent terms in the overall hybrid-controller is highlighted by setting \( \dot{\boldsymbol{q}} \) to zero in the last component of the control feedback.

Experiment #1 demonstrates the force tracking performance by controlling the normal contact force along a flat surface and executing a highly dynamic desired motion trajectory simultaneously. The actual and desired trajectories are depicted with maximum end-effector translational velocity norm attaining values of \( \approx 0.55 \, \text{m/s} \). Notably, the proposed method features a superior force tracking performance compared to the conventional technique. The remaining force error in the proposed method is due to the fact that the force control law has been obtained based on some ideal assumptions such as interacting with rigid environments, no flexibility in the robotic system, and perfect dynamic model. These un-modeled dynamics can be treated as a disturbance, and an additional integrator might be sufficient to deal with these unpredictable errors and increase the robustness against steady-state force deviations. Note that the friction forces (e.g., surface friction) are already included through the feedback of the task-space external forces \( \boldsymbol{F}_{\boldsymbol{x}}^{\text{ext}} \) in the control law, which is obtained through observer-based methods. In the experiments, a wool-polishing tool is used as end-effector.

Experiment #2 is performed to validate the effect of including gains within the force control loop. It is worth mentioning that the inclusion of force feedback gains reduces the capability of moving at high velocity due to the presence of instabilities caused by the additional control action (PI/PID) related to contact force deviations. The end-effector, in this case, follows a simple arc-like trajectory designed to provide fair conditions for a comparison with the state-of-the-art approach. The parameters are tuned to achieve the best possible performance of the classical approach in terms of contact force error. Expectedly, the force control gains reduced the steady-state error and the dynamic force oscillations but at the cost of performing the motion with lower velocity for not losing the contact. Nevertheless, the proposed approach shows lower force error than the conventional method. It can be seen that the effect of the force feedback terms is minor for the proposed method and can be ignored while not reducing the performance. In fact, that increases the robustness and simplifies the implementation of the controller.

In Experiment #3, the capability to perform highly dynamic motions on the surface of a curved object (acrylic cylindrical part) is illustrated. The task resembles a high-speed surface polishing scenario where the contact force over a curved surface is maintained. It can be clearly seen that using the classical method caused a high deviation in the direction of the surface normal due to the loss of contact with the target object. The contact loss occurs multiple times during the applied trajectory and results in impacts with the surface. The proposed approach shows superior performance in terms of force tracking while following a dynamically demanding trajectory on a curved surface. The non-contact phases are highlighted in the results, which can be related to the Euclidean norm of the joint velocities \( \|\dot{\boldsymbol{q}}\| \). Interestingly, the high velocities lead to the contact-loss (liftoff) phases even at similar task-space velocity. The maximum joint-space velocity of \( \approx 2.9 \, \text{rad/s} \), which is very close to the saturation limit, is achieved during the peaks of the task-space rotational velocity. During the highly dynamic phases, the proposed approach shows significantly better control performance in contact force direction.

The evaluation of the force tracking performance in Experiment #4 is conducted by commanding the desired force trajectory and following a specific path on a curved surface simultaneously. The trajectory is expressed using a cylindrical angle \( \beta \) and the longitudinal axis \( x \). The time-varying force trajectory and the end-effector force tracking error are illustrated. The applied trajectory position, and task-space translational and rotational velocity norms expressed in the robot base frame are shown.

The goal of the proposed control structure is to decouple the force and motion subspaces. The control scheme allows simultaneous tracking of both contact force and end-effector-compliant motion independently and at the same priority level. Therefore, the control design was carried out through a model-based approach to be able to fully compensate for the velocity-dependent terms as well as take into account the other control actions and external forces. The importance of this method is raised during the execution of fast motion trajectories and simultaneous control of the contact force. The experimental results confirm that considering the velocity dependency in the force control law seems to be necessary in fast force-controlled tasks.

## V. Conclusion

A unified hybrid force-impedance framework for highly dynamic end-effector motions was presented. In addition to the compliant behavior in the free motion directions, the approach provides precise force tracking in constrained contact direction. The control formulation allows the straightforward integration of the end-effector geometrical constraints, yielding full dynamical decoupling between the force- and motion-subspaces. Simulations and experiments including flat and curved surfaces showed the high performance of the proposed method and confirmed the theoretical claims in highly demanding dynamic motions.