# Unified Passivity-Based Cartesian Force/Impedance Control for Rigid and Flexible Joint Robots via Task-Energy Tanks

## Abstract

In this paper we propose a novel hybrid Cartesian force/impedance controller that is equipped with energy tanks to preserve passivity. Our approach overcomes the problems of (hybrid) force control, impedance control, and set-point based indirect force control. It allows accurate force tracking, full compliant impedance behavior, and safe contact resemblance simulations, and the control system robustly handles unexpected contact loss and avoids chattering behavior that switching based approaches suffer from. Furthermore, we propose a constructive way of initiating the energy tanks via the concept of task energy. This represents an estimate of the energy consumption of a given force control task prior to execution. The controller can be applied to both rigid body and flexible joint dynamics. To show the validity of our approach, several simulations and experiments with the KUKA/DLR LWR-III are carried out.

## I. Introduction and State-of-the-Art

Nowadays, robotic systems outperform humans in terms of repetitive speed and precision tasks. In terms of sensitive force and compliance control humans still show superior performance. However, an increasing set of tasks in robotic manipulation, and in particular also in real-world applications, deals now always with sensitive object handling and assembly. This requires an intricate coordination of contact force and motion generation for which sophisticated control algorithms were developed over the last decades. In this context, impedance control has become one of the most popular concepts, which aims for mimicking human behavior by imposing mass spring-damper-like disturbance response via active control on the robot. In general, compliance in robotic systems, either achieved via active control or by deliberately introducing compliant mechanical elements into the drive train, has become very popular due to their ability to cope with process uncertainty and exert only well-defined force ranges on their environment or the objects they manipulate. In this paper we focus on extending the concepts from active compliance control.

Active interaction control can be subdivided into direct and indirect force control. Recently, indirect force controllers using set-point generation were introduced. A constraint-based access to control force, position, and impedance in different subspaces was proposed. Despite the significant progress that was made in the domain of force control, some basic problems remained. For example, an impedance controller executes desired forces either via a pure feed-forward force or through a virtual displacement. Therefore, these controllers do not take into account sensed external forces. Hence, in order to accurately apply desired forces, the surface geometry and the contact properties such as stiffness need to be known a-priori. This contradicts the core idea of impedance control to work effectively in unmodeled environments. Furthermore, this paradigm has the drawback that when applying a larger force with such a feedforward approach and then unexpectedly losing contact with the environment, this may result in an undesired and possibly very unsafe motion of the robot towards the (distant) setpoint. This happens due to an instantaneous energy release of the potential energy stored in the preloaded spring into kinetic energy.

The direct force control paradigm provides the basics to accurately exchange contact forces and thus directly manipulate objects or apply forces on surfaces. This capability is also a core necessity in industrial applications, since the rather imprecise impedance control based force regulation, or let alone position control, do not suffice due to modeling and/or planning errors, resulting in possibly large process uncertainties. This problem has led to approaches such as hybrid position/force control, which idea is to partition the task space into complementary force and motion subspaces such that force and motion control is only applied in its respective subspace.

However, a major drawback of (hybrid) force control methods is that they show very low robustness with respect to contact loss. Furthermore, the contact properties of the environment need to be modeled very accurately for good performance, which is hardly ever the case. Furthermore, in order to determine the stability properties of force controllers, the environment is typically modeled as a simple spring-damper system. A very general critique regarding hybrid force/motion control was also formulated that was based on the problem of coordinate choice and the respective choice of metric.

In this paper, we strive for a robust passivity-based approach by combining force tracking with impedance control based on the concept of energy tanks. Our approach guarantees stability for arbitrary passive environments and has no need to apply set-point variations, which show rather inaccurate behavior for general environments. Our solution allows for robust, compliant, and stable manipulations without the need to choose between force or impedance control, but rather unify the best of all. Furthermore, we present a solution that is able to get rid of the inherent drawback of force and set-point based indirect force control: the low robustness with respect to contact loss and the according possibility of unsafe abrupt robot motions. In summary, the contributions of this paper are as follows.

1) Simultaneous passivity-based impedance control and wrench regulation/tracking
2) Stability proof for arbitrary passive environments instead of model-based environments
3) Suitable for rigid and flexible joint robot models
4) Task-based energy tank design and initialization
5) Contact-non-contact stabilization

Subsequently, the considered robot dynamics and thereafter the proposed controller are outlined.

## II. Modeling

### A. Rigid Robot Dynamics

The well-known rigid body dynamics of a robot with $n$ joints can be written as

$$
M(\boldsymbol{q}) \ddot{\boldsymbol{q}}+C(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})=\boldsymbol{\tau}_m+\boldsymbol{\tau}_{\text{ext}},
$$

where $\boldsymbol{q} \in \mathbb{R}^n$ is the link position. The mass matrix is denoted by $M(\boldsymbol{q}) \in \mathbb{R}^{n \times n}$, the Coriolis and centrifugal vector by $C(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}} \in \mathbb{R}^n$, and the gravity vector by $\boldsymbol{g}(\boldsymbol{q}) \in \mathbb{R}^n$. The control input of the system is the motor torque $\tau_m \in \mathbb{R}^n$, while $\tau_{\text{ext}} \in \mathbb{R}^n$ comprises all externally applied torques. External forces are denoted via the Cartesian space wrench $\boldsymbol{F}_{\text{ext}}:=\left(\boldsymbol{f}_{\text{ext}}^T, \boldsymbol{m}_{\text{ext}}^T\right)^T \in \mathbb{R}^6$, comprising a force/torque vector. This wrench can be mapped via the contact Jacobian $J^T(\boldsymbol{q})$ to joint space external torques $\boldsymbol{\tau}_{\text{ext}}=J^T(\boldsymbol{q}) \boldsymbol{F}_{\text{ext}}$.

### B. Flexible Joint Dynamics

For lightweight or SEA-type systems, the above is not sufficiently accurate to describe the inherent dynamics due to the presence of flexible transmission. Therefore, the (reduced) flexible joint model will be considered for such structures, which is described by

$$
\begin{aligned}
M(\boldsymbol{q}) \ddot{\boldsymbol{q}}+C(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q}) & =\boldsymbol{\tau}_J+\boldsymbol{\tau}_{\text{ext}} \\
B \dot{\boldsymbol{\theta}}+\boldsymbol{\tau}_J & =\boldsymbol{\tau}_m \\
\boldsymbol{\tau}_J & \equiv K(\boldsymbol{\theta}-\boldsymbol{q}),
\end{aligned}
$$

with $\boldsymbol{\theta} \in \mathbb{R}^n$ being the motor position. The first and second equations constitute the link- and motor-side dynamics, respectively. The third equation couples them via the elastic joint torque $\tau_J \in \mathbb{R}^n$, which is considered to have linear spring-like characteristics. The matrices $K \in \mathbb{R}^{n \times n}$ and $B \in \mathbb{R}^{n \times n}$ are both constant diagonal positive definite matrices, expressing the lumped joint stiffness and motor inertia, respectively.

## III. Controller Design and Analysis

### A. Preliminaries: Flexible Joint Cartesian Impedance Control

Impedance control for the flexible joint case can be realized such that passivity is preserved. This is achieved by making the position feedback to be a function of $\theta$ only instead of both $\boldsymbol{\theta}$ and $\boldsymbol{q}$. For this, $\boldsymbol{q}$ is replaced with its static equivalent $\overline{\boldsymbol{q}}(\boldsymbol{\theta}) \equiv \zeta^{-1}(\boldsymbol{\theta})$, which is numerically obtained by a contraction mapping with the implicit function $\boldsymbol{\zeta}\left(\boldsymbol{q}_e\right) \equiv \boldsymbol{q}_e+K^{-1} \boldsymbol{g}\left(\boldsymbol{q}_e\right)$, where $\boldsymbol{q}_e$ is the joint position at the equilibrium point. Under mild assumptions $\overline{\boldsymbol{q}}(\boldsymbol{\theta})$ can be used as an estimate for $\boldsymbol{q}$. The passivity-based impedance law for the flexible joint robot can then be formulated as

$$
\begin{aligned}
& \tau_{mi}=-J^T(\overline{\boldsymbol{q}})\left(K_x \overline{\boldsymbol{x}}(\overline{\boldsymbol{q}}(\boldsymbol{\theta}))+D_x \dot{\overline{\boldsymbol{x}}}\right) \\
& \overline{\boldsymbol{x}}(\boldsymbol{\theta})=\overline{\boldsymbol{x}}(\overline{\boldsymbol{q}}(\boldsymbol{\theta}))=\boldsymbol{f}(\overline{\boldsymbol{q}}(\boldsymbol{\theta}))-\boldsymbol{x}_s=\boldsymbol{x}(\boldsymbol{\theta})-\boldsymbol{x}_s \\
& \dot{\overline{\boldsymbol{x}}}(\boldsymbol{\theta})=J(\overline{\boldsymbol{q}}) \dot{\boldsymbol{\theta}},
\end{aligned}
$$

where $\tau_m=\tau_{mi}$. For simplicity, we assume $\dot{\overline{\boldsymbol{x}}}(\boldsymbol{\theta})=$ $J(\overline{\boldsymbol{q}}) \dot{\boldsymbol{\theta}} \approx J(\boldsymbol{q}) \dot{\boldsymbol{q}}=\dot{\boldsymbol{x}}$. The mapping $\boldsymbol{f}: \mathbb{R}^n \rightarrow \mathbb{R}^6$ encodes the forward kinematics while $\boldsymbol{x}_s \in \mathbb{R}^6$ denotes the (constant) impedance set-point. The matrices $K_x \in \mathbb{R}^{6 \times 6}$ and $D_x \in \mathbb{R}^{6 \times 6}$ are positive definite matrices for stiffness and damping, respectively. $D_x$ can be chosen manually (constant, often diagonal) or obtained by appropriate damping design (time-varying, non-diagonal) such that e.g. critical damping is achieved even for varying configuration. Additionally, joint torque feedback can be used to scale the motor inertia and obtain an auxiliary control input.

### B. Basic Controller Design: Force Tracking

In our controller design we start from the following Cartesian force tracking controller

$$
\begin{aligned}
\tau_{mf}=J^T(\overline{\boldsymbol{q}}) & {\left[K _ {p} \left(\boldsymbol{F}_{ext}(t)-\boldsymbol{F}_d(t)\right)\right.} \\
& +K_d\left(\dot{\boldsymbol{F}}_{ext}(t)-\dot{\boldsymbol{F}}_d(t)\right) \\
& \left.+K_i \int_0^t\left(\boldsymbol{F}_{ext}(\sigma)-\boldsymbol{F}_d\right) d \sigma\right],
\end{aligned}
$$

where $K_p \in \mathbb{R}^{6 \times 6}, K_d \in \mathbb{R}^{6 \times 6}$ and $K_i \in \mathbb{R}^{6 \times 6}$ are diagonal positive definite matrices for proportional, derivative and integral control part, respectively. The desired wrench $\boldsymbol{F}_d(t):=\left(\boldsymbol{f}_d^T(t), \boldsymbol{m}_d^T(t)\right)^T \in \mathbb{R}^6$ is specified by the user (or generated by a planner) in order to apply a desired force/torque on the environment. For convenience, we define $\boldsymbol{h}_i\left(\boldsymbol{F}_{ext}, t\right):=\int_0^t\left(\boldsymbol{F}_d(\sigma)-\boldsymbol{F}_{ext}(\sigma)\right) d \sigma$. $\boldsymbol{F}_{ext}$ can be obtained e.g. via a force/torque sensor or sensor-less methods.

### C. Stability Analysis: Part I

In order to prove the stability we first decompose the entire system into three parts, namely

1) Environment
2) Rigid body dynamics
3) Motor dynamics + Force/Impedance Controller

It is well-known that it is sufficient to prove that each part is passive w.r.t. their input-output ports if the blocks are connected in parallel or in feedback. Since the rigid body dynamics are passive w.r.t. the input-output pair $\left[\tau_J+\tau_{ext}, \dot{q}\right]$ and we assume the environment to be passive w.r.t. $\left[\dot{\boldsymbol{q}},-\tau_{ext}\right]$. This leaves only to prove that the third block is passive w.r.t. $\left[\dot{\boldsymbol{q}},-\tau_J\right]$ for proving the entire system's stability. This block has shown to be passive without the force controller and therefore also without the feedback of $-\tau_{ext}/-\boldsymbol{F}_{ext}$. Adding the impedance and force controller for the flexible joint case via $\tau_m=\tau_{mi}+\tau_{mf}$ leads to the overall controller

$$
\begin{aligned}
\boldsymbol{\tau}_m= & -J^T(\boldsymbol{q})\left[K _ {p} \left(\boldsymbol{F}_d(t)-\boldsymbol{F}_{ext}(t)\right)+K_d\left(\dot{\boldsymbol{F}}_d(t)\right.\right. \\
& \left.\left.-\dot{\boldsymbol{F}}_{ext}(t)\right)+K_i \boldsymbol{h}_i\left(\boldsymbol{F}_{ext}, t\right)+K_x \overline{\boldsymbol{x}}(\boldsymbol{\theta})+D_x \dot{\overline{\boldsymbol{x}}}\right].
\end{aligned}
$$

It is generally sufficient to analyze the passivity w.r.t. $\left[\dot{\boldsymbol{\theta}},-\tau_J\right]$ instead of $\left[\dot{\boldsymbol{q}},-\tau_J\right]$ for a flexible joint robot due to the relation

$$
\begin{aligned}
-\dot{\boldsymbol{q}}^T \boldsymbol{\tau}_J & =\frac{d}{d t}\left(\frac{1}{2}(\boldsymbol{\theta}-\boldsymbol{q})^T K(\boldsymbol{\theta}-\boldsymbol{q})\right)-\dot{\boldsymbol{\theta}}^T \boldsymbol{\tau}_J \\
& =\dot{S}_\theta-\dot{\boldsymbol{\theta}}^T \boldsymbol{\tau}_J,
\end{aligned}
$$

where $S_\theta:=\frac{1}{2}(\boldsymbol{\theta}-\boldsymbol{q})^T K(\boldsymbol{\theta}-\boldsymbol{q})$ is a positive definite storage function. We can now conclude from the motor dynamics and overall controller

$$
\begin{aligned}
-\dot{\boldsymbol{\theta}}^T \boldsymbol{\tau}_J= & \dot{\boldsymbol{\theta}}^T B \ddot{\boldsymbol{\theta}}-\dot{\boldsymbol{\theta}}^T \boldsymbol{\tau}_m \\
= & \dot{\boldsymbol{\theta}}^T B \ddot{\boldsymbol{\theta}}+\dot{\boldsymbol{\theta}}^T J^T\left[K_p\left(\boldsymbol{F}_d-\boldsymbol{F}_{ext}\right)\right. \\
& \left.+K_d\left(\dot{\boldsymbol{F}}_d-\dot{\boldsymbol{F}}_{ext}\right)+K_i \boldsymbol{h}_i+K_x \overline{\boldsymbol{x}}+D_x \dot{\overline{\boldsymbol{x}}}\right] \\
= & \dot{S}_M+\dot{S}_I+\dot{\overline{\boldsymbol{x}}}^T D_x \dot{\overline{\boldsymbol{x}}}+\dot{\overline{\boldsymbol{x}}}^T K_p\left(\boldsymbol{F}_d-\boldsymbol{F}_{ext}\right) \\
& +\dot{\overline{\boldsymbol{x}}}^T K_d\left(\dot{\boldsymbol{F}}_d-\dot{\boldsymbol{F}}_{ext}\right)+\dot{\overline{\boldsymbol{x}}}^T K_i \boldsymbol{h}_i,
\end{aligned}
$$

with $S_M(\boldsymbol{\theta}):=\frac{1}{2} \dot{\boldsymbol{\theta}}^T B \dot{\boldsymbol{\theta}}$ and $S_I(\overline{\boldsymbol{x}}):=\frac{1}{2} \overline{\boldsymbol{x}}^T K_x \overline{\boldsymbol{x}}$. While $S_\theta+S_I+S_M$ has proven to be positive definite, it is unfortunately not possible to derive the sign of any of the last three terms. Thus, the passivity of the third block is potentially violated.

### D. Controller Design: Energy Tank Design

In order to solve the aforementioned problem and preserve the passivity at all times, we use the concept of energy tanks to modify the controller and add a tank element such that passivity is preserved. First, we introduce the virtual tank, which state is denoted as $x_t$ and its output as $y_t=x_t$. The associated tank energy is $T=\frac{1}{2} x_t^2$, while its dynamics is defined as

$$
\dot{x}_t=\frac{\beta}{x_t}\left(\dot{\boldsymbol{x}}^T D_x \dot{\boldsymbol{x}}+\gamma \dot{\boldsymbol{x}}^T K_d\left(\dot{\boldsymbol{F}}_d-\dot{\boldsymbol{F}}_{\text{ext}}\right)\right)+u_t,
$$

where $\beta$ is defined as

$$
\beta= \begin{cases}1 & \text{if } T \leq T^u \\ 0 & \text{else.} \end{cases}
$$

Its purpose is to avoid further tank loading via dissipative terms if a certain upper limit storage $T^u$ is reached. The control input of the tank is chosen to be $u_t=-\omega^T \dot{x}$ with

$$
\begin{aligned}
\boldsymbol{\omega}\left(\boldsymbol{F}_{\text{ext}}, t\right)= & \frac{\alpha}{x_t}\left(K_p\left(\boldsymbol{F}_{\text{ext}}-\boldsymbol{F}_d\right)\right. \\
& \left.+(1-\gamma) K_d\left(\dot{\boldsymbol{F}}_d-\dot{\boldsymbol{F}}_{\text{ext}}\right)-K_i \boldsymbol{h}_i\left(\boldsymbol{F}_{\text{ext}}, t\right)\right),
\end{aligned}
$$

where $\alpha$ is defined as

$$
\alpha= \begin{cases}1 & \text{if } T \geq T_l \\ 0 & \text{else.} \end{cases}
$$

$\alpha$ is responsible for detaching the energy tank from the force/impedance controller if the lower limit of the energy tank $T_l$ is reached. Therefore, in order to avoid the singularity, the lower limit needs to be greater than zero, i.e. $T_l>0$. $\gamma$ is defined as

$$
\gamma= \begin{cases}1 & \text{if } \dot{\boldsymbol{x}}^T K_d\left(\dot{\boldsymbol{F}}_d-\dot{\boldsymbol{F}}_{\text{ext}}\right) \geq 0 \\ 0 & \text{else.} \end{cases}
$$

Based on $\boldsymbol{\omega}\left(\boldsymbol{F}_{\text{ext}}, t\right)$ we can rewrite the motor dynamics as

$$
\begin{aligned}
-\boldsymbol{\tau}_J= & B \ddot{\boldsymbol{\theta}}+J^T(\boldsymbol{q})\left[K_x \overline{\boldsymbol{x}}+D_x \dot{\boldsymbol{x}}+\gamma K_d\left(\dot{\boldsymbol{F}}_d-\dot{\boldsymbol{F}}_{\text{ext}}\right)\right. \\
& \left.-\boldsymbol{\omega} x_t\right]=: B \ddot{\boldsymbol{\theta}}-\tau_m^{\prime},
\end{aligned}
$$

where $\tau_m^{\prime}$ is our proposed control law for the force/impedance controller including energy tank. Subsequently, we define the input of the force/impedance controller to be $\boldsymbol{u}_{fi}=\boldsymbol{\omega} x_t$ and the output to be $\boldsymbol{y}_{fi}=\dot{\boldsymbol{x}}$. The energy tank is connected to the force/impedance controller via a power-preserving Dirac structure

$$
\left(\begin{array}{c}
\boldsymbol{u}_{fi} \\
u_t
\end{array}\right)=\left[\begin{array}{cc}
0_{6 \times 6} & \boldsymbol{\omega} \\
-\boldsymbol{\omega}^T & 0
\end{array}\right]\left(\begin{array}{c}
\boldsymbol{y}_{fi} \\
y_t
\end{array}\right).
$$

### E. Stability Analysis: Part II

Due to the lossless connection we can write the storage function as

$$
\begin{aligned}
S\left(\dot{\boldsymbol{\theta}}, \overline{\boldsymbol{x}}, x_t\right)= & S_M(\boldsymbol{\theta})+S_I(\overline{\boldsymbol{x}})+T\left(x_t\right) \\
= & \frac{1}{2} \dot{\boldsymbol{\theta}}^T B \dot{\boldsymbol{\theta}}+\frac{1}{2} \overline{\boldsymbol{x}}^T K_x \overline{\boldsymbol{x}}+\frac{1}{2} x_t^2.
\end{aligned}
$$

Evaluating the timely evolution along the motor dynamics and tank dynamics leads to

$$
\begin{aligned}
\dot{S}\left(\dot{\boldsymbol{\theta}}, \overline{\boldsymbol{x}}, x_t\right)= & -\dot{\boldsymbol{\theta}}^T \boldsymbol{\tau}_J-\dot{\boldsymbol{x}}^T D_x \dot{\boldsymbol{x}}-\dot{\boldsymbol{x}}^T \gamma K_d\left(\dot{\boldsymbol{F}}_d-\dot{\boldsymbol{F}}_{\text{ext}}\right) \\
& +\dot{\boldsymbol{x}}^T \boldsymbol{\omega} x_t \\
& +\beta\left(\dot{\boldsymbol{x}}^T D_x \dot{\boldsymbol{x}}+\gamma \dot{\boldsymbol{x}}^T K_d\left(\dot{\boldsymbol{F}}_d-\dot{\boldsymbol{F}}_{\text{ext}}\right)\right) \\
& -x_t \boldsymbol{\omega}^T \dot{\boldsymbol{x}} \\
= & -\dot{\boldsymbol{\theta}}^T \boldsymbol{\tau}_J-\dot{\boldsymbol{x}}^T D_x \dot{\boldsymbol{x}}-\gamma \dot{\boldsymbol{x}}^T K_d\left(\dot{\boldsymbol{F}}_d-\dot{\boldsymbol{F}}_{\text{ext}}\right) \\
& +\beta\left(\dot{\boldsymbol{x}}^T D_x \dot{\boldsymbol{x}}+\gamma \dot{\boldsymbol{x}}^T K_d\left(\dot{\boldsymbol{F}}_d-\dot{\boldsymbol{F}}_{\text{ext}}\right)\right).
\end{aligned}
$$

Since $S\left(\dot{\boldsymbol{\theta}}, \overline{\boldsymbol{x}}, x_t\right) \geq 0$ and obviously $\dot{S}\left(\dot{\boldsymbol{\theta}}, \overline{\boldsymbol{x}}, x_t\right) \leq -\dot{\boldsymbol{\theta}}^T \boldsymbol{\tau}_J$ due to the definitions of $\beta$ and $\gamma$, the block is passive. Basically, the energy tank dynamically cancels out the nonpassive terms which arise during contact such that the block is passive w.r.t. $\left[\boldsymbol{\theta},-\boldsymbol{\tau}_J\right]$ and therefore $\left[\dot{\boldsymbol{q}},-\boldsymbol{\tau}_J\right]$. Hence, the overall system is shown to be stable.

### F. Contact-loss Stabilization

Although stability is now guaranteed in all cases, this does not mean that the robot could not execute unsafe motions. Unexpected loss of contact with the surface would mean that the robot still tries to regulate a force and it would do this until the tank is drained. Depending on the remaining energy in the tank, this may lead to fast and large unwanted motions. In order to prevent this problem, one could intuitively suggest to deactivate the controller when no contact is detected. However, this would lead to undesired switching behavior due to sensor noise, a typical and basically unsolved problem in force control. Here, we propose a more robust and provably stable way by introducing a controller shaping function $\rho(\psi)$, where $\psi$ is a shifted variable w.r.t. $\|\Delta p\|$ in the translational case or $\Delta \varphi$ in the rotational case. This function is incorporated into the overall controller as follows:

$$
\rho(\psi)=\left\{\rho_t(\psi), \rho_t(\psi), \rho_t(\psi), \rho_r(\psi), \rho_r(\psi), \rho_r(\psi)\right\}^T
$$

is composed of a translational part $\rho_t(\psi)$, which is defined as

$$
\rho_t(\psi)=\left\{\begin{array}{cc}
1 & \text{if } \boldsymbol{f}_d^T \Delta \boldsymbol{p} \geq 0 \\
\frac{1}{2}\left[1+\cos \left(\frac{\psi-\|\Delta \boldsymbol{p}\|}{d_{\max}} \pi\right)\right] & \text{if } \boldsymbol{f}_d^T \Delta \boldsymbol{p}<0 \\
& \wedge \psi \in\{\|\Delta \boldsymbol{p}\|, \\
& \quad\|\Delta \boldsymbol{p}\|+d_{\max }\} \\
0 & \text{else}
\end{array}\right.
$$

and a rotational part $\rho_r(\psi)$ defined as

$$
\rho_r(\psi)=\left\{\begin{array}{cc}
1 & \text{if } \boldsymbol{f}_d^T \Delta \boldsymbol{p} \Delta \boldsymbol{k}_v \geq 0 \\
\frac{1}{2}\left[1+\cos \left(\frac{\psi-\Delta \varphi}{\varphi_{\max}} \pi\right)\right] & \text{if } \boldsymbol{f}_d^T \Delta \boldsymbol{k}_0 \Delta \boldsymbol{k}_v<0 \\
& \wedge \psi \in\left[\Delta \varphi,\right. \\
& \Delta \varphi+\varphi_{\max }] \\
0 & \text{else}
\end{array}\right.
$$

We define $\boldsymbol{x}:=\left(\boldsymbol{p}^T, \boldsymbol{\varphi}^T\right)^T$ to be comprised of a translational part $\boldsymbol{p}$ and a chosen rotational representation, e.g. Euler angles $\varphi$. In the translational part, $\Delta \boldsymbol{p}=\boldsymbol{p}_s-\boldsymbol{p}$ denotes the vector that points from the end-effector position to the set-point and $\boldsymbol{F}_d:=\left(\boldsymbol{f}_d^T, \boldsymbol{m}_d^T\right)^T$ is the desired wrench. If $\Delta \boldsymbol{p}$ and $\boldsymbol{f}_d$ enclose an angle greater than $90^{\circ}$ the controller should be deactivated. In order to ensure a smooth transition instead of a chattering behavior $\rho_t(\psi)$ interpolates in a user-defined region of width $d_{\max}$. For the rotational part $\rho_r(\psi)$ we choose the singularity-free quaternion representation. The unit quaternion $\boldsymbol{k}=\left(k_0, \boldsymbol{k}_v\right)$ denotes the current orientation and the quaternion $\boldsymbol{k}_s=\left(k_{0,s}, \boldsymbol{k}_{v,s}\right)$ the desired orientation. The rotation error is then defined as $\Delta \boldsymbol{k}:=\boldsymbol{k}^{-1} \boldsymbol{k}_s$ and $\Delta \varphi:=2 \arccos \left(\Delta k_0\right)$. The user-defined rotational robustness region can then be specified as an angle $\varphi_{\max}$, which relates to the scalar component of the quaternion by $\varphi_{\max}=2 \arccos \left(k_{0,\max }\right)$.

From a stability point of view, the controller shaping function can be interpreted as shaping $\omega$ since it only scales the force controller part of the combined force/impedance controller. Therefore, $\omega$ can simply be redefined as

$$
\boldsymbol{\omega}_\rho\left(\boldsymbol{F}_{ext}, t\right):=\left(\begin{array}{c}
\rho_t(\psi) \mathbf{1}_{3 \times 1} \\
\rho_r(\psi) \mathbf{1}_{3 \times 1}
\end{array}\right) \boldsymbol{\omega}\left(\boldsymbol{F}_{ext}, t\right),
$$

thus again stability is ensured. The multiplication of $\rho(\psi)$ is understood component-wise and the passivity analysis for the system with energy tank can be carried out in a similar manner.

### G. Task-energy Based Tank Initialization

Although stability is proven and contact-loss stabilization is ensured, this does not necessarily mean that the task can be fulfilled as desired. If the tank is initially not loaded with sufficient energy, the force controller will be deactivated already during task execution. Thus, the intended task goal will not be achieved (at least not with the desired performance), since force regulation cannot be maintained properly. For this problem, we introduce the concept of task energy $E_T$, which we define as the minimum initial tank energy that is needed to fulfill the desired task (here: accurate contact force tracking). In order to estimate $E_T$ e.g. for a translational force tracking task, we make use of the static equilibrium of forces

$$
\left.\boldsymbol{f}_I\right|_{p=p_w}+\boldsymbol{f}_d(t)=\boldsymbol{f}_W,
$$

where $\boldsymbol{f}_I=K_{x,t}\left(\boldsymbol{p}-\boldsymbol{p}_s\right)$ and $\boldsymbol{f}_W$ are the forces due to the impedance stiffness and environmental counter force, respectively. $K_{x,t}$ is the translational stiffness matrix, $\boldsymbol{p}$ is the end-effector position, and $\boldsymbol{p}_s$ is the desired end-effector position. For sake of simplicity let us model the counter force $\boldsymbol{f}_W$ that is generated by the environment as a linear stiffness (without the consideration of damping) between $p$ (in the steady-state case $\boldsymbol{p}=\boldsymbol{p}_s$) and wall set-point $\boldsymbol{p}_{s,0}$ as $\boldsymbol{f}_W=K_{w,t}\left(\boldsymbol{p}_w-\boldsymbol{p}_{s,0}\right)$. Here, $\boldsymbol{f}_W$ denotes the force exerted by the wall when $\boldsymbol{p}_w$ is the respective (translational) wall position. Solving for $\boldsymbol{p}_w(t)$ yields the wall position after the force is regulated. The work required to move the wall can be calculated as

$$
E_T(t)=\int_0^t \frac{1}{2}\left(\boldsymbol{p}_w(\sigma)-\boldsymbol{p}_{w,0}\right)^T K_{w,t}\left(\boldsymbol{p}_w(\sigma)-\boldsymbol{p}_{w,0}\right) d \sigma.
$$

For the special regulation case $\boldsymbol{f}_d=$ const. we may calculate the task energy as $E_T=\frac{1}{2}\left(\boldsymbol{p}_w-\boldsymbol{p}_{w,0}\right)^T K_{w,t}\left(\boldsymbol{p}_w-\boldsymbol{p}_{w,0}\right)$ and initialize the tank energy accordingly.

## IV. Simulation

The simulation and experimental validation was carried out with the fully torque-controlled KUKA/DLR LWR-III. The simulation framework was implemented in MATLAB Simulink and the controllers are designed according to the stability analysis. The initial configuration for the simulation of the robot is $\boldsymbol{q}_0=(0,0,0,-90,0,90,0)^T[0]$. The environment model is a simple virtual wall. For the simulation, the wall is located at $\boldsymbol{p}_{w,0}=(0,0,0.65)^T[\mathrm{m}]$ with a translational stiffness of $K_{w,t}=\operatorname{diag}\{250,250,250\}[\mathrm{N/m}]$. The end-effector starting position (translation) is $\boldsymbol{p}_0=(-0.39,0,0.71)^T[\mathrm{m}]$ and the impedance set-point is located at $\boldsymbol{p}_s=(-0.39,0,0.5)^T[\mathrm{m}]$. The remaining parameters and gains can be found in Table I. $D_x$ is designed such that critical damping is achieved. The simulation results show that the force error $f_z=f_{\text{ext},z}-f_{d,z}$ is regulated to zero over time. It also depicts that the tank is filled due to the movement of the robot. Then energy is drained as the integral part draws energy from the tank. When the lower tank limit is defined as $T_l=1[\mathrm{J}]$, the energy is insufficient and the desired force cannot be regulated. By calculating the required task energy for $\boldsymbol{f}_d=(0,0,5)^T=$ const. as $E_T=\frac{1}{2}\left(\boldsymbol{p}_w-\boldsymbol{p}_{w,0}\right)^T K_{w,t}\left(\boldsymbol{p}_w-\boldsymbol{p}_{w,0}\right)$ and initializing the tank with this amount, the force can be regulated to zero again.

## V. Experiments

For the following two experiments we use a KUKA/DLR LWR-III which initial position is $\boldsymbol{q}_0=(0,30,0,-60,0,90,0)^T[0]$. An aluminum plate is mounted on the end-effector that is covered by wool. Since the robot is not equipped with a force sensor, the external force $\boldsymbol{F}_{\text{ext}}$ is obtained via an accurate observer-based estimate and low-pass filtered with a cut-off frequency of $10[\mathrm{Hz}]$. Again, $D_x$ is obtained as described and all parameters for the experiments are found in Table I. It should be noted that the damping gain in the experiments is set to zero, because otherwise the system becomes unstable. This is due to the fact that no force-torque sensor was used and instead an observer for the external forces whose dynamics is not considered in the tank design.

### A. Polishing Task

In the first experiment, the task is to polish a table with a constantly regulated normal force. The robot starts from $\boldsymbol{q}_0$ and is moved via Cartesian impedance control to the vicinity of the table without touching it. Afterwards, a contact with the table is established via impedance control. Now, the force controller is activated to apply a desired force. While applying this specified force, a constant velocity is commanded in $x$-direction inducing a movement towards the edge of the table. The results show different phases. It can be seen by observing the $z$-position that although contact is lost unexpectedly, the end-effector is still nearby the table height and does not move any further towards the ground.

## VI. Conclusion

In this paper we proposed a novel Cartesian passivity-based force/impedance controller. In order to be able to systematically fuse both concepts, we applied the concept of energy tanks such that the force tracking controller, impedance controller, energy tank, and motor dynamics together yield a passive system. Furthermore, our approach is able to cope with contact discontinuities such that no unwanted rapid motions due to contact loss may occur. To validate our theoretical results, several simulations and experiments were carried out. Conclusively, Table II summarizes the main features of the presented controller and compares it to existing approaches. As already mentioned, if stability needs to be ensured at all times, exact force regulation may not be achieved.

In order to get rid of this problem, which is equivalent to the question of how to initialize the energy tank prior to task execution, we introduced the concept of task energy. This is defined as the required energy a force tracking task consumes. By estimating its amount and starting the task after the tank has reached the respective task energy level, we were able to get rid of this known limitation of energy tanks.