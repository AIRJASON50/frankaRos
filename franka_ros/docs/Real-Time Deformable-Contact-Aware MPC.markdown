# Real-Time Deformable-Contact-Aware Model Predictive Control for Force-Modulated Manipulation

## Authors
Lasitha Wijayarathne$^{1*}$, Student Member, IEEE, Ziyi Zhou$^{2*}$, Student Member, IEEE, Ye Zhao$^{2\dagger}$, Senior Member, IEEE, and Frank L. Hammond III$^{1\ddagger}$, Member, IEEE

## Abstract
Force modulation of robotic manipulators has been extensively studied for several decades. However, it is not yet commonly used in safety-critical applications due to a lack of accurate interaction contact modeling and weak performance guarantees - a large proportion of them concerning the modulation of interaction forces. This study presents a high-level framework for simultaneous trajectory optimization and force control of the interaction between a manipulator and soft environments, which is prone to external disturbances. Sliding friction and normal contact force are taken into account. The dynamics of the most constant model and the manipulation are simultaneously incorporated in a trajectory optimizer to generate desired motion and force profiles. A constrained optimization framework based on Alternating Direction Method of Multipliers (ADMM) has been employed to efficiently generate real-time optimal control inputs and high-dimensional state trajectories in a Model Predictive Control fashion. Experimental validation of the model performance is conducted on a soft substrate with known material properties using a Cartesian space force control mode. Results show a comparison of ground truth and real-time model-based contact force and motion tracking for multiple carried mass motions in the rapid range of the friction model. It is shown that a contact model-based motion planner can compensate for frictional forces and motion disturbances and improve the overall motion and force tracking accuracy. The proposed high-level planner has the potential to facilitate the automation of medical tasks involving the manipulation of compliant, delicate, and deformable tissues.

## I. Introduction
Robotics applications in the medical domain have gained increasing attention over the past few decades [1], [2], where the planning and control of interaction forces between a robot and its environment are essential to various safety-critical tasks. For instance, interaction forces should be modulated accurately in compliant environments, such as surgical settings, micro-assembly, or biological tissue manipulation. Furthermore, force control based on identifiable physical models is crucial to identify instability modes (e.g., those caused by the bandwidth and system structure) and maintain reliable force interaction to guarantee safety. Thus, a model-based trajectory planning method with a high-fidelity contact model is essential for successful deployment with satisfactory motion and contact force tracking performance.

In the field of model-based planning and simulation, there have been extensive studies in handling rigid contact and deformable contact models. For instance, rigid contact can be modeled using numerical solutions to complementarity problems [3]–[6], or hybrid models with impact dynamics [7], [8]. Unlike in rigid body contact, soft contact models are subject to challenges posed by non-linear material properties and non-uniformity as well as intensive computation burden due to numerical computation for solutions. Considering soft contact mechanics during a physical interaction is important for safe adaptation and planning, especially in robotic surgical applications. Soft contact modeling is challenging as it depends on material properties, deformation, and rolling friction. Numerous contact models have been presented in the literature to model interactions involving elastic deformation [9]–[13]. These models have broad applications and are essential in many engineering areas such as machine design, robotics, multi-body analysis, to name a few. For contact problems that involve elasticity, Hertz adhesive contact theory has been well established [14]. In this study, we focus on robotic tasks interacting with soft tissues, the contact behavior of which is determined by not only external and viscous forces, and contact geometry, but also material properties (see Figure 1).

Many robotic tasks require motion planning in the presence of contact in constrained environments. For example, in the automation of medical applications, the environment is heavily constrained and safety is paramount [15]. In such scenarios, the robot motion and the reaction force imparted on the environment are coupled together. Similar examples are found in automated manufacturing, legged locomotion, and assistive robotics [16]–[18]. As a promising approach along this direction, trajectory optimization (TO) with either hard or soft complementarity constraints [3], [19]–[24] or explicit contact models [25]–[29] has been extensively investigated in the robotics community. By incorporating the contact forces and joint states into the optimization, contact-dynamics-consistent motions can be planned for complex robot behaviors, such as dynamic locomotion or dexterous object manipulation. While the above works demonstrated impressive results on automatically discovering contact sequences, the optimized contact force was not used for control, and only served as a way to explore the physical interaction with objects. In this work, we aim to accurately track a nominal force trajectory along with an end-effector path, which is commonly demanded but underexplored in safety-critical tasks.

Safety-critical tasks such as soft material manipulation and medical applications require extensive manipulation with accurate contact interactions. These include interaction with humans in proximity or with direct physical contact. In approaching the manipulation interaction problem, various methods have been explored, including model-based and model-free methods. Data-driven techniques have been studied to learn the interaction between robotic manipulators and the environment [30]–[33]. Unlike rigid contacts, soft environments are prone to both spatial and temporal uncertainties and it is challenging to learn the contact model and robot dynamics simultaneously through data [11], [34], [35]. In contrast, model-based approaches could capture the principal component of contact dynamics to a reasonable fidelity and be used along with an appropriate planning framework [36]–[38]. These studies present control frameworks and sensor modalities that incorporate contact models to make informed control decisions, including physics-based contact models [36], mesh-based simulation models, and tactile models [37]. However, in these studies, simultaneous motion and force tracking is largely underexplored. Our method models the contact with normal and frictional components of the force as a smooth dynamical system to augment the robot dynamics. Then, we embed it into a contact-aware TO for high-fidelity planning.

Since contact-rich environments are prone to disturbances and difficult to model or predict, it is indispensable to solve the aforementioned optimization problem online with updated contact state information. However, efficiently solving highly constrained TO with full robot and contact dynamics remains an open problem. Instead of solving a large-scale optimization in a holistic way, we propose a distributed optimization framework inspired by works from the legged locomotion community [39]–[43], but employ a different way for decomposing the original problem given the soft manipulation task. More specifically, this framework efficiently alternates between three sub-problems/sub-blocks: (i) an unconstrained sub-problem that only incorporates coupled rigid body (i.e., the robot end-effector) and soft contact dynamics for accurate force tracking; (ii) an inverse kinematics (IK) sub-problem that generates a joint trajectory to track the desired end-effector trajectory; (iii) a constraint projection sub-problem for handling inequality constraints including contact constraints. The proposed distributed framework enables an online execution in a Model Predictive Control (MPC) fashion. Furthermore, to maintain the contact and stability, an additional control layer will need to be deployed on top of the high-level trajectory planner. We add a low-level admittance force controller [44] to handle uncertainties arising from the model as well as from disturbances in the environment.

### Main Contributions
- Presentation of a dynamic interaction model based on soft contact mechanics for a predefined geometry with Hertz visco-static theory.
- Incorporation of the interaction model into a constrained TO to generate the desired Cartesian path and force profile in an efficient, distributed fashion.
- Experimental validation of the derived contact dynamical model and real-time implementation of the proposed TO algorithm with Model Predictive Control (MPC).

A conference version of the work presented in this paper was published in [45]. The work presented here extends the previous work in three respects. First, we extend the previous distributed framework into three sub-problems instead of only two, i.e., an independent IK sub-problem is extracted from the previous dynamics block to further split the dynamics (force tracking) and kinematics (end-effector tracking) planning. We benchmark different versions of solving the same constrained TO problem including the one proposed in our conference version [45]. Second, our proposed trajectory planner is executed online in an MPC fashion. We experimentally demonstrate the efficacy of this framework on simultaneous motion and force tracking tasks on a static and dynamic disturbance-induced platform. Last but not least, we implement a low-level controller to aid the high-level trajectory planner in motion and force trajectory tracking.

## II. Related Work
### Contact Models
Elastic contact mechanics [14] have been extensively studied in various research fields where contact modeling is imperative for safety and performance requirements. Existing works in [10], [46]–[48] have used soft contact models for both modeling and control. These works include quasi-static assumptions and studies of [49], [50] explore cases where high-velocity impacts on soft material are considered. In the impact cases, visco-elastic models have been widely investigated. For instance, studies in [10], [49] compared various visco-elastic models with experimental validations. A majority of these works show that the Hertzian-based Hunt-Crossey model is the one most suitable for visco-elastic cases. Furthermore, fundamentals of frictional sliding motion are established in the works of [13], [51], where the main focus is on rigid body contacts but generalizable to soft contacts. More recent works in [12], [52] propose contact-area-based models.

### Trajectory Optimization
Trajectory optimization (TO) is a powerful tool to generate reliable and intelligent robot motions. Various numerical optimization methods have been proposed to solve a TO [53]–[55]. Among them, Differential Dynamic Programming (DDP) [56] and its first-order variant iterative Linear Quadratic Regulator (iLQR) [57] have aroused much attention in solving TO in the context of unconstrained problems, where only dynamics constraint is enforced. The Riccati-like backward pass in DDP or iLQR effectively reduces the complexity by solving an approximated LQR problem over the entire horizon, and the optimization is solved in an iterative fashion. In [58], DDP is used in a balancing task of a humanoid robot with high degrees of freedom (DoFs). A follow-up work [59], demonstrates a Model Predictive Control (MPC) implementation based on DDP. However, incorporating additional constraints in standard DDP algorithms is still an open problem. In [60]–[70], DDP-type variants are proposed to cope with equality or inequality constraints containing state or control variables. Meanwhile, an augmented Lagrangian (AL)-based method, Alternating Direction Method of Multipliers (ADMM) [71], is proposed to solve constrained optimization problem in a distributed fashion. In [72], [73], ADMM is used to solve large-scale convex problems with box and cone constraints. Although the convergence is only guaranteed for convex problems, ADMM has been proven effective for solving highly non-convex problems such as collision avoidance [74], [75], mixed-integer programming [76], [77], linear complementarity problem [78], and recently employed for legged robot trajectory planning [40], [41]. Our approach leverages both the distributed nature of ADMM and the efficiency of DDP, which decomposes the original problem into unconstrained whole-body Dynamics planning with a soft contact model, kinematics planning, and constraint projection.

### Trajectory Optimization with Soft Contact
Contact-aware TO for legged locomotion and dexterous object manipulation is often built upon the assumption of rigid contact dynamics [3], [19]–[23], [25]–[29]. In [25], [26], [79], soft contact models are integrated into the system dynamics to approximate the hard contact, and the contact models are still relatively simple. In [80], a soft contact model was considered in the optimization formulation for whole-body locomotion control. Although incorporating these soft contact models has demonstrated impressive results, it is challenging to directly use these approaches for soft manipulation tasks considered in this work. Because most of them assume simple spring-damper type soft contact models, which still largely mismatch the contact surface deformation or elasticity [36]–[38], [81] in reality. Therefore, advanced planning algorithms that accurately model complex contact dynamics are imperative to enable maneuvering over complex terrain or grasping irregular objects. To date, constrained TO incorporating a high-fidelity deformable contact model remains underexplored in the field.

### Model Predictive Control (MPC)
The generation of trajectories for a given model and fixed horizon is computationally prohibitive in nature, making it difficult to deploy in real-time contact-rich applications, where model uncertainties and environmental disturbances are ubiquitous. Model Predictive Control (MPC) is a powerful strategy widely used to generate motion plans in real time and be adaptive to state changes due to environmental disturbances [82], [83]. Recent advances in fast automatic differentiation (AutoDiff) [84] and AutoDiff compatible rigid body models has enabled real-time optimal control. The study in [85] showed a hardware implementation of MPC with a DDP optimizer framework at an update rate of 1000 Hz on a 7-DOF robot for a vision-based point-to-point trajectory planning.

### Admittance Control
To cope with un-modeled modalities of the contact, we use a low-level force controller (FC) based on admittance control [44] which has a fast control update rate compared to the high-level planner. It can compensate for the uncertainties that arise spatially over the surface (e.g., stiffness, slipperiness, and damping). Admittance control has been long studied and proven to work efficiently in compliant environments [86]. Furthermore, low-level controller mitigates instabilities [87] arising from the contact caused by the control update rate, stiffness mismatch, and high gains.

## III. Deformable Contact Modeling
### A. Contact Modeling via Hertz's Theory
In this section, we model the interaction dynamics between an application tool mounted on a manipulator and a soft tissue in terms of contact geometry and mechanics. For simplicity, the tool tip of this study is of a spherical shape (but not limited to). We assume that the tool end tip used is rigid and relatively stiffer compared to the contact surface. With these assumptions, we derive a contact dynamical model based on the contact friction theory and pressure distribution. According to Hertz's theory, the largest static deformation is observed at the midpoint of the circle (as shown in the deformation in Figure 2) and can be expressed as:

$$
d = \left[ \frac{9 F^2}{16 E^2 R} \right]^{\frac{1}{3}}
$$

where $E$ is the reduced Young's modulus of the tool and surface, $R$ is the radius of the tool end, and $F$ is the force imparted on the surface by manipulator tool-tip. Combined Young's modulus of the tool-tip and the soft contact surface material can be lumped to one term as:

$$
\frac{1}{E} \equiv \frac{1 - \nu_1^2}{E_1} + \frac{1 - \nu_2^2}{E_2}
$$

where $E_1, E_2$, and $\nu_1, \nu_2$ are Young's moduli and Poisson ratios of the end-effector and contact surface material, respectively. In our scenario, we assume the contact part as a rigid object, and thus Young's modulus of the spherical tool-tip $E_1$ is comparatively high. Consequently, the lumped stiffness can be approximated as $E = E_2 / (1 - \nu_2^2)$.

The deformation and stress distributions on the contact surface are approximated by the universal Hooke's law and Hertz's theory. A detailed elaboration of normal, radial, and hoop (i.e., moving direction) stress distributions within the contact area in the cylindrical coordinate system are provided in Appendix A.

The deformation distribution is derived from the stress distribution equations as follows:

$$
\sigma_n = \begin{cases} 
\frac{3 \pi}{8 a} \left[ \frac{1 - \nu^2}{E} \right] p_m (2 a^2 - r^2), & (r \leq a) \\
\frac{3}{4 a} \left[ \frac{1 - \nu^2}{E} \right] p_m \left[ (2 a^2 - r^2) \sin^{-1} \left( \frac{a}{r} \right) + a (r^2 - a^2)^{\frac{1}{2}} \right], & (r \geq a)
\end{cases}
$$

where $p_m = F / (\pi a^2)$ is the average stress applied in the contact part by the tool-tip and $a = \sqrt{R d}$ is the radius of contact area (see Figure 2). A force vector $F$ at an angle $\theta_F$ to the normal is applied to the application tool and moves in a curved path of a radius $R$ with a uniform velocity $\mathbf{v}_e$ in the frame {sphere}. This represents a scenario of a tool interaction with a soft surface to accomplish a motion task. For the sake of simplicity, our model focuses on sliding friction (primary mode) and ignores other frictional sources such as adhesion and rolling induced by deformation. This assumption is valid for modeling purposes where adhesion and rolling are secondary and specific to the tool material and the application.

To derive frictional forces, we use principal stress on the contact. If $\sigma_\theta$ represents the principal stress within the contact circle due to the symmetry of our contact scenario, we can represent the stress tensor of any contact point $(r, \theta, z)$ in cylindrical coordinates relative to frame {sphere} via the Cauchy stress theory [14].

$$
\boldsymbol{\sigma} = \begin{bmatrix}
\sigma_r & 0 & \sigma_{rz} \\
0 & \sigma_\theta & 0 \\
\sigma_{zr} & 0 & \sigma_z
\end{bmatrix}
$$

Since the task is defined in the Cartesian frame, we convert parameters to Cartesian coordinates from cylindrical coordinates. The stress tensor in Cartesian coordinate is $\sigma_c = T^T \sigma T$, where the transformation matrix $T$ is defined in Appendix A.

At an arbitrary point on contact surface $(x, y, z)$ {sphere}, the normal vector from this point to centroid of spherical cap is $\mathbf{n} = \begin{bmatrix} s\theta & 0 & c\theta \end{bmatrix}^T$. Then, the normal stress of the contact surface is $\sigma_n = \mathbf{n}^T \sigma_c \mathbf{n}$ with

$$
\sigma_n = \sigma_r c^2 \theta s^2 \theta + \sigma_\theta s^4 \theta + \sigma_z c^2 \theta + 2 \sigma_{rz} s \theta c^2 \theta
$$

Given this stress expression, the overall friction force of the contact surface is represented as

$$
\begin{aligned}
d f &= \mu \sigma_n d S = \mu \sigma_n \times 2 \pi r \frac{d r}{c \theta} \\
f &= \int d f c \theta = 2 \pi \mu \int_0^a \sigma_n r d r
\end{aligned}
$$

where, $d f, d r, d S$ are the differential elements of the friction, $r$, and contact area. In the surface normal direction, it is assumed that the surface is in contact with the end point of the tool. As a result, Eq. (1) always holds. To derive the dynamic model in the normal direction of contact, the derivative form of Eq. (1) is taken

$$
\dot{z} = -\dot{d} = -\left[ \frac{1}{6 E^2 R F_z} \right]^{\frac{1}{2}} \dot{F}_z
$$

where $z$ represents the position along the surface normal direction of the contact point and force along the normal direction is defined as $F_z = F \cos \theta_F$. In the moving direction, Eq. (4) and $\mu F_z$ give the frictional force caused by the normal force $F_z$, which is:

$$
\mathbf{F}_f = f = \mu F_z \left[ 1 + (2 \nu - 1) \frac{3 a^2}{10 R^2} \right] \mathbf{n}_e + k_d \mathbf{v}_e
$$

where $k_d$ is a damping coefficient in the moving direction. By substituting $a = \sqrt{R d}$ and Eq. (6), we have the derivative form of Eq. (6). $\mathbf{n}_e$ is the unit vector of the velocity and $\mathbf{v}_e$ is the end-effector velocity at the contact point.

$$
\dot{\mathbf{F}}_f = \dot{f} = \mu \dot{F}_z + \frac{3 \mu (2 \nu - 1)}{10 R} \left( \dot{F}_z d - F_z \dot{z} \right) \mathbf{n}_e + k_d \dot{\mathbf{v}}_e
$$

The overall model, in simplistic terms, $\dot{\mathbf{F}}_e = \mathcal{G}(F_z, \mathbf{v}_e, \text{contact parameters})$ with the frictional and normal force components of the contact model can be written in a compact form as:

$$
\begin{aligned}
\dot{\mathbf{F}}_x &= \left( \left( 6 E^2 R F_z \right)^{\frac{1}{2}} \dot{d} \right) \mathbf{n}_z \\
&\quad + \left( \mu \dot{F}_z + \frac{3 \mu (2 \nu - 1)}{10 R} \left( \dot{F}_z d + F_z \dot{d} \right) \right) \mathbf{n}_e + k_d \dot{\mathbf{v}}_e
\end{aligned}
$$

where $\mathbf{n}_v$ is the unit vector and $d$ is the deformation at the central point of the contact circle and is calculated from Eq. (1), and $\dot{d} = \dot{x}_z$. $F_z$ is the vertical force (the surface normal direction) applied on the surface by the manipulator and $\mathbf{v}_e = \|\mathbf{J} \dot{\mathbf{q}}\|$ is the moving velocity of the tool contact point.

### B. Manipulator Dynamics
The manipulator model dynamics is expressed below:

$$
\ddot{\mathbf{q}} = \mathbf{M}(\mathbf{q})^{-1} \left( \boldsymbol{\tau}_u - \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} - \mathbf{G}(\mathbf{q}) - \mathbf{J}^T \mathbf{F}_e \right)
$$

where $\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}}, \tau_u \in \mathbb{R}^k$ are the manipulator joint position, velocity, acceleration, and torque vectors. We use $k = 7$ for our 7-DoF robotic arm. $\mathbf{M}(\mathbf{q})$ is the joint space mass matrix, $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})$ is the Coriolis term, $\mathbf{G}(\mathbf{q})$ is the gravity term, $\tau_s$ is the torque applied at joints, $\mathbf{J}$ is the Jacobian with respect to the contact point, and $\mathbf{F}_e$ is the external Cartesian wrench at the end-effector.

### C. Contact Constraint Modeling
The motion studied in this paper is primarily in the sliding mode, which an equality constraint can describe. Since the sliding is embedded in the contact model, additional constraints for sliding are not required. However, a constraint is added to make sure the robot is only sliding in the desired path where the path is curved. Figure 4 shows the two components of the force acting on the tool, namely, frictional and centripetal forces along the path of curvature $R_c$ as shown in Figure 3. The contact model provides the sliding friction, and the centripetal force constraint is added as a constraint:

$$
\frac{\mathbf{J}^{-T} \mathbf{M}(\mathbf{q}) \mathbf{J}^{-1} \|\mathbf{J} \dot{\mathbf{q}}\|^2}{\kappa} \leq \mu \mathbf{N}^T \mathbf{F}_e \mathbf{N}
$$

where $\mathbf{F}_e \in \mathbb{R}^3$ is the force vector at the end effector and $\mathbf{J}$ is the Jacobian, $\kappa$ is the curvature of the motion path (as shown in Figure 3), $\mathbf{J}^{-T} \mathbf{M}(\mathbf{q}) \mathbf{J}^{-1}$ is the effective mass at the contact point of the robot with a mass matrix of $\mathbf{M}(\mathbf{q})$ and $\mathbf{J} \dot{\mathbf{q}}$ is the moving velocity of the contact point. $\mathbf{N}$ is the surface average normal vector (as perceived by the force-torque sensor). In this study, we use $\mathbf{N} = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}^T$, which is only in the $z$ direction. The constraint represented by Eq. (10) keeps the robot in contact when the Cartesian tracking path has a curvature, and the robot is operated in a lower impedance mode. It is an effect of the resulting centripetal force on the effective mass at the contact. For instance, if the velocity at a curve is high, it would slip in the orthogonal direction of the moving direction unless high positional gains are used to compensate for it.

## IV. Problem Formulation
The optimization problem is to solve a control trajectory that would result in a desired Cartesian trajectory along a desired force profile. The overall problem is formulated in Formulation 1, where the state is represented as $\mathbf{x} = \begin{bmatrix} \mathbf{q} & \dot{\mathbf{q}} & \mathbf{F}_e \end{bmatrix}^T$ including the manipulator joint position, velocity, and the end-effector force vectors. The control input is equivalent to the joint torque vector $\mathbf{u} = \tau_u$. For simplicity, we use $\phi = (\mathbf{x}[0, \ldots, N], \mathbf{u}[0, \ldots, N-1])$ to represent the sequence of state-control pairs.

The objective function is comprised of a force tracking cost, an end-effector pose tracking cost, and a regularization term on the applied torques. The normal force error with respect to the force reference $F^d$ is expressed as $\delta \mathbf{F}[i] = (F_z[i] - F^d[i])$. Matrices $\mathbf{Q} \in \mathbb{R}^{n \times n}$ and $\mathbf{R} \in \mathbb{R}^{m \times m}$ are the state and control weighting matrices, $\mathcal{FK} \in \mathbb{R}^{4 \times 4}$ is the forward kinematics function of the manipulator, and $W_p \in \mathbb{R}^{4 \times 4}$ is the state weighting matrix for the pose tracking cost with respect to a desired end-effector trajectory $\mathbf{x}_e^d$.

### Formulation 1: Simultaneous Trajectory and Force Optimization
(Tracking Task)

$$
\min_\phi \sum_{i=0}^N \underbrace{\delta \mathbf{F}[i]^T \mathbf{Q}_F \delta \mathbf{F}[i]}_{\text{force tracking}} + \mathbf{u}[i]^T \mathbf{R} \mathbf{u}[i] + \underbrace{W_p \|\mathcal{FK}(\mathbf{q}[i]) - \mathbf{x}_e^d[i]\|_2^2}_{\text{pose tracking}}
$$

(Decision Variables) $\phi[i] = \begin{bmatrix} \mathbf{x}[i]^T & \mathbf{u}[i]^T \end{bmatrix}^T$

(Dynamics) $\mathbf{x}[i+1] = \mathcal{F}(\mathbf{x}[i], \mathbf{u}[i]), \quad \forall i = 0, 1, \ldots, N-1$

(Initial Condition) $\mathbf{x}[0] = \mathbf{x}_0$

(Constraints) $\mathbf{x}[i] \in \mathcal{X}, \mathbf{u}[i] \in \mathcal{U}, \quad \forall i = 0, 1, \ldots, N$

(Contact Constraint) $\mathcal{J}(\mathbf{x}[i], \mathbf{u}[i]) \leq 0, \quad \forall i = 0, 1, \ldots, N$

### Formulation 2: Distributed Constrained Optimization (Consensus)

$$
\begin{aligned}
&\min_{\phi, \bar{\phi}, \hat{\phi}} \sum_{i=0}^N \delta \mathbf{F}[i]^T \mathbf{Q}_F \delta \mathbf{F}[i] + \mathbf{u}[i]^T \mathbf{R} \mathbf{u}[i] + W_p \|\mathcal{FK}(\mathbf{q}[i]) - \mathbf{x}_e^d[i]\|_2^2 + I_{\mathcal{J}, \mathcal{U}, \mathcal{F}}(\mathbf{q}[i], \mathbf{u}[i], \boldsymbol{\lambda}[i]) \\
&\text{(Variables-DDP)} \quad \phi[i] = \begin{bmatrix} \mathbf{x}[i]^T & \mathbf{u}[i]^T \end{bmatrix}^T, \quad \mathbf{x}[i] = \begin{bmatrix} \mathbf{q}[i] & \dot{\mathbf{q}}[i] & \mathbf{F}_e[i] \end{bmatrix}^T \\
&\text{(Variables-IK)} \quad \bar{\phi}[i] = \mathbf{q}[i]^T \\
&\text{(Variables-Proj)} \quad \hat{\phi}[i] = \begin{bmatrix} \mathbf{q}[i]^T & \mathbf{u}[i]^T & \boldsymbol{\lambda}[i]^T \end{bmatrix}^T \\
&\forall i = 0, 1, \ldots, N-1 \\
&\text{(Dynamics)} \quad \mathbf{x}[i+1] = \mathcal{F}(\mathbf{x}[i], \mathbf{u}[i]) \\
&\text{(Initial Condition)} \quad \mathbf{x}[0] = \mathbf{x}_0 \\
&\text{(Consistency)} \quad \mathbf{q}[i] = \bar{\mathbf{q}}[i] = \hat{\mathbf{q}}[i], \quad \mathbf{u}[i] = \hat{\mathbf{u}}[i]
\end{aligned}
$$

### Formulation 3: Distributed Constrained Optimization (Sequential)

$$
\begin{aligned}
&\min_{\phi, \bar{\phi}, \hat{\phi}} \sum_{i=0}^N \delta \mathbf{F}[i]^T \mathbf{Q}_F \delta \mathbf{F}[i] + \mathbf{u}[i]^T \mathbf{R} \mathbf{u}[i] + W_p \|\mathcal{FK}(\mathbf{q}[i]) - \mathbf{x}_e^d[i]\|_2^2 + I_{\mathcal{J}, \mathcal{U}, \mathcal{F}}(\mathbf{q}[i], \mathbf{u}[i], \boldsymbol{\lambda}[i]) \\
&\text{(Variables-DDP)} \quad \phi[i] = \begin{bmatrix} \mathbf{x}[i]^T & \mathbf{u}[i]^T \end{bmatrix}^T, \quad \mathbf{x}[i] = \begin{bmatrix} \mathbf{q}[i] & \dot{\mathbf{q}}[i] & \mathbf{F}_e[i] \end{bmatrix}^T \\
&\text{(Variables-IK)} \quad \bar{\phi}[i] = \bar{\mathbf{q}}[i]^T \\
&\text{(Variables-Proj)} \quad \hat{\phi}[i] = \begin{bmatrix} \mathbf{q}[i]^T & \mathbf{u}[i]^T & \boldsymbol{\lambda}[i]^T \end{bmatrix}^T \\
&\forall i = 0, 1, \ldots, N-1 \\
&\text{(Dynamics)} \quad \mathbf{x}[i+1] = \mathcal{F}(\mathbf{x}[i], \mathbf{u}[i]) \\
&\text{(Initial Condition)} \quad \mathbf{x}[0] = \mathbf{x}_0 \\
&\text{(Consistency)} \quad \mathbf{q}[i] = \bar{\mathbf{q}}[i], \quad \mathbf{q}[i] = \hat{\mathbf{q}}[i], \quad \mathbf{u}[i] = \hat{\mathbf{u}}[i]
\end{aligned}
$$

## Experimental Results
### Static Environment
As described in Section VII-G, the same open-loop full trajectories were executed with force control as described in Section VI. Figure 9 shows experimental results, which suggest low-level force control alone (the blue trajectory) can improve the reference force tracking accuracy significantly. However, the reference motion tracking accuracy degrades due to the frictional forces encountered on the surface observed in the experimental results. The reference force and motion tracking performance improve with active force control and MPC (the green trajectory). Furthermore, Table II shows quantified results on multiple motion trajectories with different geometries with different curvatures. The motion accuracy is observed to be better without force control but the force tracking is worse. On the other hand, with force control and MPC, both motion and force accuracy are improved as indicated by the corresponding RSME values. In Cartesian geometries that contain sharp curves (e.g., rectangular geometry), the centripetal force component is an addition to the frictional force in contact force compensation, the velocity at the corners needs to vary to maintain the path and the contact as illustrated in Figure 4. Failing to compensate for it could result in sliding and deviating from the desired motion (the without FC case in Figure 9). Only friction force needs to be compensated in a straight line, while both centripetal and frictional components are present in a curved geometry.

**Table II: RSME for Motion and Force**

| Method                     | Motion Path 1 |       | Motion Path 2 |       | Motion Path 3 |       |
|----------------------------|---------------|-------|---------------|-------|---------------|-------|
|                            | Path (m)      | Force (N) | Path (m)      | Force (N) | Path (m)      | Force (N) |
| Pulsations, without FC      | 0.0142        | 2.723 | 0.0264        | 2.470 | 0.0075        | 1.377 |
| Pulsations, with FC         | 0.0839        | 1.095 | 0.0757        | 1.289 | 0.0187        | 0.8450 |
| Pulsations, MPC + FC        | 0.0258        | 0.573 | 0.0384        | 0.852 | 0.0153        | 0.3230 |

### Dynamic Environment
In Section VII-G, it was shown that force control and MPC could improve both force and motion tracking accuracy. However, the environment is subject to motion disturbances in a realistic setting. To cope with such disturbances and compensate for frictional forces simultaneously, we experimentally show the efficacy of the proposed trajectory optimization (TO) method. Figure 10 illustrates the force and motion tracking accuracy when under force control, without force control, and with force control, and MPC. Similar to the static case, low-level force control improves force tracking, but the "reactive" nature can be observed in the force tracking (in Figure 10 right column, second row subplot). As a result, the MPC with contact model information improves both force and motion tracking accuracy significantly. Moreover, the system remains stable with externally induced motion disturbances and model uncertainties. Such a success can be attributed to the lower-level admittance controller and the model-based TO run in an MPC fashion. Experiments were run on multiple Cartesian trajectories to validate on varying geometries, and quantified results are reported in Table III.

**Table III: RSME for Motion and Force (Dynamic Environment)**

| Method                     | Motion Path 1 (Circle) |       | Motion Path 2 (Eight) |       | Motion Path 3 (Line) |       |
|----------------------------|-----------------------|-------|-----------------------|-------|----------------------|-------|
|                            | Path (m)              | Force (N) | Path (m)          | Force (N) | Path (m)         | Force (N) |
| Without FC                 | 0.284                 | 1.042 | 0.392                 | 1.328 | 0.302                | 0.891 |
| With FC                    | 0.0183                | 0.384 | 0.0258                | 0.481 | 0.0172               | 0.0273 |
| MPC + FC                   | 0.0164                | 0.283 | 0.0191                | 0.319 | 0.0233               | 0.0084 |

## VIII. Conclusion and Future Work
In automation tasks requiring physically soft tissue contact, it is paramount to design soft contact interaction models where controllers can be designed to guarantee safety performance. Contact modeling is crucial in correctly identifying the contact material and performing mundane tasks such as incisions along given paths and motion disturbance compensation. This study presented a coherent framework for simultaneous motion and force modulation on compliant surfaces. Moreover, we presented a distributed (ADMM), real-time framework executed in an MPC fashion capable of handling state, control, and contact constraints. Further, we incorporated a soft contact dynamical model into the trajectory optimization (TO). Results proved that motion and force tracking accuracy is significantly improved in both static and dynamic environments. Potential applications of this work include contact manipulation in soft tissues or safety-critical environments.

Trajectories solved from the TO were experimentally validated on a soft surface (EcoFlex®) with the aid of a robot manipulator with an attached spherical shaped tooltip. Surface material properties were estimated and further used in generating optimal trajectories. Experiments were performed on a static and a motion-induced dynamic environment. Results of MPC, with and without force control, were presented. Ground truth forces were obtained using a force-torque sensor (ATI mini45) and compared against the obtained results. MPC with force control was able to track both motion and force both in a static and a dynamic environment with significant improvements. Results and discussion conclude model-based contact modeling and hierarchical TO (e.g., low-level and high level) provide a better alternative for safe simultaneous force and motion generation.

The future extension of this work is to improve the generability (i.e., "richness") of the contact model to adapt to a wide range of material properties. Furthermore, real-time estimation of the contact model properties can improve the adaptability of the planning framework. Moreover, we intend to extend the work to plan trajectories in three-dimensional surfaces to demonstrate practical applications such as planning robotic incisions on a human body.

## IX. Acknowledgements
The authors would like to thank Qie Sima for his valuable insights and contribution to developing and testing the contact model.

## Appendix A: Deformable Patch Contact Model
In this Appendix, we provide the details on several stress distributions.

**Normal Stress Distribution $\sigma_z$**:

$$
\frac{\sigma_z}{p_m} = -\frac{3}{2} \left( 1 - \frac{r^2}{a^2} \right)^{\frac{1}{2}} \quad (r \leq a)
$$

**Radial Stress Distribution $\sigma_r$**:

$$
\frac{\sigma_r}{p_m} = \frac{2 \nu}{2} \frac{1}{r^2} \left[ 1 - \left( 1 - \frac{r^2}{a^2} \right) \right] - 3 \nu \left( 1 - \frac{r^2}{a^2} \right)^{\frac{1}{2}} \quad (r \leq a)
$$

**Hoop Stress Distribution $\sigma_\theta$**:

$$
\frac{\sigma_\theta}{p_m} = \frac{1 - 2 \nu}{2} \frac{a^2}{r^2} \left[ 1 - \left( 1 - \frac{r^2}{a^2} \right) \right] - \frac{3}{2} \left( 1 - \frac{r^2}{a^2} \right)^{\frac{1}{2}} \quad (r \leq a)
$$

where $p_m = F / (\pi a^2)$ is the average stress applied in contact part by manipulation and $a = \sqrt{R d}$ is the radius of contact area (refer to Figure 2). The transformation matrix $T$ is

$$
T = \begin{bmatrix}
c \theta & s \theta & 0 \\
-s \theta & c \theta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

## Appendix B: Three-Block Sequential ADMM
Instead of establishing a consistency constraint between the decision variables from the IK sub-block and the projection sub-block (i.e., $\dot{\mathbf{q}} = \hat{\mathbf{q}}$ in Formulation 2), the sequential ADMM enforces an equality between DDP sub-block and IK sub-block, i.e., $\mathbf{q} = \bar{\mathbf{q}}$, as shown in Formulation 3.

Different from the consensus variant, the original optimization problem is separated into three sub-problems as described in Formulation 3.

## References
[1] (References omitted for brevity, but can be included if needed.)