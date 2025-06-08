# Passive Hierarchical Impedance Control Via Energy Tanks

## Abstract

Modern robotic systems with a large number of actuated degrees of freedom can be utilized to perform several tasks at the same time while following a given order of priority. The most frequently used method is to apply null space projections to realize such a strict hierarchy, where lower priority tasks are executed as long as they do not interfere with any higher priority objectives. However, introducing null space projectors inevitably destroys the beneficial and safety-relevant feature of passivity. Here, two controllers are proposed to restore the passivity.

## I. Introduction

The paper is organized as follows: after recapitulating the fundamentals in hierarchical control in Section II, the local-tank and global-tank approaches are introduced in Section III and Section IV, respectively. Proofs of passivity are given. Afterwards, simulations and experiments are conducted in Section V. The discussion in Section VI closes the paper.

## II. Fundamentals

The dynamics of a robot with $n$ DOF can be described by

$$
\boldsymbol{M}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})=\boldsymbol{\tau}+\boldsymbol{\tau}^{\text{ext}},
$$

where $\boldsymbol{q} \in \mathbb{R}^n$ describes the joint configuration. The inertia matrix $\boldsymbol{M}(\boldsymbol{q}) \in \mathbb{R}^{n \times n}$ is symmetric and positive definite, the Coriolis/centrifugal matrix $\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \in \mathbb{R}^{n \times n}$ is formulated such that $\dot{\boldsymbol{M}}(\boldsymbol{q})=\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}})+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}})^T$ holds. The generalized gravity forces are given by $\boldsymbol{g}(\boldsymbol{q}) \in \mathbb{R}^n$, the generalized forces $\boldsymbol{\tau} \in \mathbb{R}^n$ determine the control input, while $\boldsymbol{\tau}^{\text{ext}} \in \mathbb{R}^n$ are the generalized external forces. In the following, the dependencies on the states are omitted in the notations, if not strictly necessary for the understanding.

A control task hierarchy consisting of $r$ priority levels with the respective task coordinates $\boldsymbol{x}_i=\boldsymbol{f}_i(\boldsymbol{q}) \in \mathbb{R}^{m_i}$ and the associated task dimensions $m_i \in \mathbb{N}$ (for $i=1 \ldots r$) is introduced. The highest-priority level has index $i=1$, the lowest one has index $i=r$. The operational space velocities on the individual hierarchy levels are described by the mappings via the Jacobian matrices $\boldsymbol{J}_i=\partial \boldsymbol{f}_i(\boldsymbol{q}) / \partial \boldsymbol{q}$:

$$
\dot{\boldsymbol{x}}_i=\boldsymbol{J}_i \dot{\boldsymbol{q}} .
$$

All Jacobian matrices are assumed to be of full row-rank and the stacked version, the so-called augmented Jacobian matrix $\left(\boldsymbol{J}_1^T, \ldots, \boldsymbol{J}_r^T\right)^T$, is non-singular. The goal of the hierarchical controller is that lower-priority tasks do not disturb any higher-priority tasks, neither statically nor dynamically, and they are performed as well as possible under this restriction.

### A. Hierarchical Dynamics

This strict hierarchy can be expressed in the dynamic equations by an adequate coordinate transformation, which leads to the formulation

$$
\boldsymbol{\Lambda} \dot{\boldsymbol{v}}+\boldsymbol{\mu} \boldsymbol{v}=\boldsymbol{\bar{J}}^{-T}\left(-\boldsymbol{g}+\boldsymbol{\tau}+\boldsymbol{\tau}^{\text{ext}}\right)
$$

with the block-diagonal inertia matrix $\boldsymbol{\Lambda} \in \mathbb{R}^{n \times n}$, the Coriolis/centrifugal matrix $\boldsymbol{\mu} \in \mathbb{R}^{n \times n}$, and the local, hierarchy-consistent task velocities $\boldsymbol{v}_i \in \mathbb{R}^{m_i}$ for $i=1 \ldots r$ following

$$
\left(\begin{array}{c}
\boldsymbol{v}_1 \\
\vdots \\
\boldsymbol{v}_r
\end{array}\right)=\left(\begin{array}{c}
\boldsymbol{J}_1 \\
\vdots \\
\boldsymbol{J}_r
\end{array}\right) \dot{\boldsymbol{q}}=\boldsymbol{J} \dot{\boldsymbol{q}} .
$$

The hierarchy-consistent Jacobian matrices $\boldsymbol{\bar{J}}_i$ for $i=1 \ldots r$ can be stacked to obtain the invertible matrix $\boldsymbol{\bar{J}} \in \mathbb{R}^{n \times n}$. While the main task level is not restricted ($\boldsymbol{\bar{J}}_1=\boldsymbol{J}_1$), all subtasks are described by $\boldsymbol{\bar{J}}_i=\left(\boldsymbol{Z}_i \boldsymbol{M} \boldsymbol{Z}_i^T\right)^{-1} \boldsymbol{Z}_i \boldsymbol{M} \in \mathbb{R}^{m_i \times n}$ for $i=2 \ldots r$, where the null space base matrices $\boldsymbol{Z}_i \in \mathbb{R}^{m_i \times n}$ fulfill $\boldsymbol{J}_j \boldsymbol{Z}_i^T=\mathbf{0} \forall j<i$. For consistency in the notations, $\boldsymbol{Z}_1=\left(\boldsymbol{J}_1^{M+}\right)^T$ is introduced, i.e., the dynamically consistent pseudoinverse $\boldsymbol{J}_1^{M+}=\boldsymbol{M}^{-1} \boldsymbol{J}_1^T\left(\boldsymbol{J}_1 \boldsymbol{M}^{-1} \boldsymbol{J}_1^T\right)^{-1}$.

### B. Classical, Non-Passive Impedance Controller

The classical control law is

$$
\boldsymbol{\tau}=\boldsymbol{g}+\boldsymbol{\tau}_\mu+\sum_{i=1}^r \boldsymbol{\bar{J}}_i^T \boldsymbol{Z}_i \boldsymbol{J}_i^T \boldsymbol{F}_i
$$

and contains the gravity compensation $\boldsymbol{g}$, the passive feedback action $\boldsymbol{\tau}_\mu \in \mathbb{R}^n$ to annihilate the remaining velocity-dependent Coriolis/centrifugal couplings across the priority levels, and the (null-space projected) torques $\boldsymbol{\tau}_i^p \forall i$ to perform all tasks in the hierarchy. The operational space forces $\boldsymbol{F}_i \in \mathbb{R}^{m_i}$ on all levels $i=1 \ldots r$ are arbitrary and may, for example, realize desired spring-damper behaviors such as in a classical impedance controller:

$$
\boldsymbol{F}_i=-\left(\frac{\partial V_i(\tilde{\boldsymbol{x}}_i)}{\partial \boldsymbol{x}_i}\right)^T-\boldsymbol{D}_i \dot{\boldsymbol{x}}_i .
$$

Herein, $V_i(\tilde{\boldsymbol{x}}_i)$ is the positive definite potential function describing such a spring with the error $\tilde{\boldsymbol{x}}_i=\boldsymbol{x}_i-\boldsymbol{x}_i^d$, where $\boldsymbol{x}_i^d \in \mathbb{R}^{m_i}$ is the desired value in the operational space for the regulation case, and $\boldsymbol{D}_i \in \mathbb{R}^{m_i \times m_i}$ is the damping matrix.

In the two-level case, it has been shown that such a controller may generate active energy, thus the closed loop is not passive. Here, a task hierarchy with an arbitrary number of priority levels is considered, which is an active controller for the same reasons. In the following, two energy-tank-based control approaches are introduced to restore the passivity and handle this safety-critical issue.

## III. Controller with Local Energy Tanks

The joint velocities can be expressed as the sum of the contributions from all hierarchy levels:

$$
\dot{\boldsymbol{q}}=\sum_{j=1}^r \boldsymbol{Z}_j^T \boldsymbol{v}_j .
$$

Combining this with the task velocity equation yields

$$
\dot{\boldsymbol{x}}_i=\boldsymbol{J}_i \sum_{j=1}^{i-1} \boldsymbol{Z}_j^T \boldsymbol{v}_j + \boldsymbol{J}_i \boldsymbol{Z}_i^T \boldsymbol{v}_i
$$

due to the annihilation $\boldsymbol{J}_i \boldsymbol{Z}_j^T=\mathbf{0} \forall j>i$. Therefore, $\dot{\boldsymbol{x}}_i$ can be expressed as the sum of two velocity components: $\boldsymbol{w}_i \in \mathbb{R}^{m_i}$ from all higher-priority levels and $\boldsymbol{\kappa}_i \in \mathbb{R}^{m_i}$ from level $i$ itself. There are no contributions from the lower-priority levels $(i+1) \ldots r$ thanks to the dynamic consistency and the strictness of the hierarchy.

### A. Storage Function for Level $i$ and Source of Activity

The virtual spring on level $i$ is described by the potential function $V_i(\tilde{\boldsymbol{x}}_i)$, thus the storage function

$$
S_{\text{pot},i}=V_i(\tilde{\boldsymbol{x}}_i)
$$

can be established. Using the velocity decomposition, its time derivative yields

$$
\dot{S}_{\text{pot},i}=\boldsymbol{\kappa}_i^T \left(\frac{\partial V_i(\tilde{\boldsymbol{x}}_i)}{\partial \boldsymbol{x}_i}\right)^T + \boldsymbol{w}_i^T \left(\frac{\partial V_i(\tilde{\boldsymbol{x}}_i)}{\partial \boldsymbol{x}_i}\right)^T .
$$

The power transmission of the null space compliance controller w.r.t. the power port $(\dot{\boldsymbol{q}},-\boldsymbol{\tau}_i^p)$ is

$$
-\dot{\boldsymbol{q}}^T \boldsymbol{\tau}_i^p = \boldsymbol{\kappa}_i^T \left(\frac{\partial V_i(\tilde{\boldsymbol{x}}_i)}{\partial \boldsymbol{x}_i}\right)^T + (\dot{\boldsymbol{x}}_i - \boldsymbol{w}_i)^T \boldsymbol{D}_i \dot{\boldsymbol{x}}_i
$$

as $\dot{\boldsymbol{q}}^T \boldsymbol{J}_i^T \boldsymbol{Z}_i \boldsymbol{J}_i^T = \boldsymbol{\kappa}_i^T$ holds. Inserting this into the storage function derivative yields

$$
\dot{S}_{\text{pot},i} = -\dot{\boldsymbol{q}}^T \boldsymbol{\tau}_i^p - \dot{\boldsymbol{x}}_i^T \boldsymbol{D}_i \dot{\boldsymbol{x}}_i - \boldsymbol{w}_i^T \boldsymbol{F}_i .
$$

While the second term is negative semi-definite for positive definite damping matrices $\boldsymbol{D}_i$, the sign of $-\boldsymbol{w}_i^T \boldsymbol{F}_i$ cannot be determined. This latter term induces a potentially non-passive feedback action of the null space compliance controller of level $i$ w.r.t. $(\dot{\boldsymbol{q}},-\boldsymbol{\tau}_i^p)$ and is the reason for the activity of these components.

### B. Introduction of Local Energy Tanks

To restore the passivity on the levels $i=2 \ldots r$, one virtual energy tank is placed on each hierarchy level storing the locally dissipated energy of active damping. The local energy tank is defined as a virtual storage element with flow variable $\dot{s}_i$ and effort variable $s_i$ such that

$$
E_{\text{tank},i} = \frac{1}{2} s_i^2 .
$$

To preserve the passivity even in case of an empty tank ($E_{\text{tank},i}=0$), a new coordinate $\tilde{\boldsymbol{x}}_i \in \mathbb{R}^{m_i}$ and its time derivative $\dot{\tilde{\boldsymbol{x}}}_i \in \mathbb{R}^{m_i}$ are introduced from which the task torque on level $i$ is generated. This variable $\tilde{\boldsymbol{x}}_i$ may deviate from the original operational space coordinate $\boldsymbol{x}_i$ to keep the compliance controller subsystem for hierarchy level $i$ passive at the cost of some control performance due to the deliberately introduced error $\tilde{\boldsymbol{x}}_i - \boldsymbol{x}_i$. The new coordinate $\tilde{\boldsymbol{x}}_i$ is the result of the numerical integration of

$$
\dot{\tilde{\boldsymbol{x}}}_i = \beta_i \left( \boldsymbol{w}_i + \boldsymbol{K}_{\text{P},i} (\boldsymbol{x}_i - \tilde{\boldsymbol{x}}_i) \right) + \boldsymbol{\kappa}_i
$$

w.r.t. time $t$. The term $\boldsymbol{K}_{\text{P},i} (\boldsymbol{x}_i - \tilde{\boldsymbol{x}}_i)$ with gain $\boldsymbol{K}_{\text{P},i} \in \mathbb{R}^{m_i \times m_i}$ is introduced to reduce the drift and to let $\tilde{\boldsymbol{x}}_i$ converge to $\boldsymbol{x}_i$. The scalar $\beta_i$ is introduced to control the tracking behavior of $\tilde{\boldsymbol{x}}_i$:

$$
\beta_i = \begin{cases} 
0 & \text{if } E_{\text{tank},i} \leq E_{\text{tank},i}^{\text{min}} \\
1 & \text{else}
\end{cases}
$$

To avoid discontinuities, the transition between 0 and 1 can be made smooth. The new task force is constructed based on $\tilde{\boldsymbol{x}}_i$ and $\dot{\tilde{\boldsymbol{x}}}_i$:

$$
\boldsymbol{F}_i = -\left( \frac{\partial V_i(\tilde{\boldsymbol{x}}_i)}{\partial \tilde{\boldsymbol{x}}_i} \right)^T - \boldsymbol{D}_i \dot{\tilde{\boldsymbol{x}}}_i
$$

and is applied in combination with the classical control law.

### C. Bond Graph and Energy Flows

The fill level of the tank must be limited to prevent a steady increase of the stored energy. The energy flow is depicted in a bond graph, where the Dirac structure implements

$$
\left( \begin{array}{c}
\boldsymbol{F}_i^{\text{D}} \\
\dot{\boldsymbol{w}}_i \\
-\dot{s}_i^T
\end{array} \right) = \left( \begin{array}{ccc}
0 & 0 & \left( \frac{\boldsymbol{D}_i \dot{\tilde{\boldsymbol{x}}}_i}{s_i} \right) \\
0 & 0 & \left( \frac{\dot{\tilde{\boldsymbol{x}}}_i}{s_i} \right) \\
-\left( \frac{\boldsymbol{D}_i \dot{\tilde{\boldsymbol{x}}}_i}{s_i} \right)^T & -\left( \frac{\dot{\boldsymbol{w}}_i}{s_i} \right)^T & 0
\end{array} \right) \left( \begin{array}{c}
\dot{\tilde{\boldsymbol{x}}}_i \\
\dot{s}_i \\
s_i
\end{array} \right)
$$

in which the force $\boldsymbol{F}_i^{\text{D}} = \boldsymbol{D}_i \dot{\tilde{\boldsymbol{x}}}_i$ is generated and the flow

$$
\dot{s}_i^T = \frac{1}{s_i} \left( \dot{\tilde{\boldsymbol{x}}}_i^T \boldsymbol{D}_i \dot{\tilde{\boldsymbol{x}}}_i + \beta_i \dot{\boldsymbol{w}}_i^T \boldsymbol{F}_i \right)
$$

is defined. The modulated transformer $\alpha_i$ acts as an overflow valve:

$$
\alpha_i = \begin{cases} 
1 & \text{if } (E_{\text{tank},i} \geq E_{\text{tank},i}^{\text{max}}) \wedge (\dot{\tilde{\boldsymbol{x}}}_i^T \boldsymbol{D}_i \dot{\tilde{\boldsymbol{x}}}_i + \beta_i \dot{\boldsymbol{w}}_i^T \boldsymbol{F}_i > 0) \\
0 & \text{else}
\end{cases}
$$

The 0-junction fulfills

$$
\dot{s}_i = \dot{s}_i^\sigma - \alpha_i \dot{s}_i^{\text{R}} = (1 - \alpha_i^2) \dot{s}_i^\sigma .
$$

### D. Passivity Analysis

The new storage function for the entire potential energy on subtask level $i=2 \ldots r$ is

$$
S_{\text{pot},i} = V_i(\tilde{\boldsymbol{x}}_i) + E_{\text{tank},i}
$$

with its time derivative yielding

$$
\dot{S}_{\text{pot},i} = -\dot{\boldsymbol{q}}^T \boldsymbol{\tau}_i^{\text{p}} - \alpha_i^2 \left( \dot{\tilde{\boldsymbol{x}}}_i^T \boldsymbol{D}_i \dot{\tilde{\boldsymbol{x}}}_i + \beta_i \dot{\boldsymbol{w}}_i^T \boldsymbol{F}_i \right) \leq -\dot{\boldsymbol{q}}^T \boldsymbol{\tau}_i^{\text{p}} .
$$

This implies passivity of the extended null space compliance controller on level $i$ w.r.t. the power port $(\dot{\boldsymbol{q}},-\boldsymbol{\tau}_i^{\text{p}})$. For the decoupled dynamics, the storage function is

$$
S_i = \frac{1}{2} \boldsymbol{v}_i^T \boldsymbol{\Lambda}_i \boldsymbol{v}_i + S_{\text{pot},i}
$$

with

$$
\dot{S}_i \leq \boldsymbol{v}_i^T \boldsymbol{F}_i^{\text{ext}} .
$$

The overall storage function is

$$
S = \sum_{i=1}^r S_i
$$

with

$$
\dot{S} \leq \dot{\boldsymbol{q}}^T \boldsymbol{\tau}^{\text{ext}} .
$$

## IV. Controller with Global Energy Tank

Instead of filling $r-1$ local energy tanks, the dissipated energy fills one global tank ($E_{\text{tank,g}}$). If the tank is empty, all subtask operational space coordinates deviate simultaneously, i.e., $\beta_i=0$ for $i=2 \ldots r$. This exchange of energies across priority levels affects passivity properties: individual subtask levels are not guaranteed to be passive, but overall passivity of the closed loop w.r.t. $\boldsymbol{\tau}^{\text{ext}}$ (input) and $\dot{\boldsymbol{q}}$ (output) is ensured.

## V. Simulations and Experiments

### A. Simulations

A 6-DOF planar manipulator is simulated with five priority levels:

1) Level 1 ($m_1=2$): Translational Cartesian impedance (TCP in $x, y$)
2) Level 2 ($m_2=1$): Rotational Cartesian impedance (TCP about $z$)
3) Level 3 ($m_3=1$): Joint impedance (second joint)
4) Level 4 ($m_4=1$): Joint impedance (first joint)
5) Level 5 ($m_5=1$): Singularity avoidance (end-effector)

The step responses show that energy tanks prevent energy generation observed in the classical controller. The local-energy-tank solution ensures passive behavior on all subtask levels, while the global-energy-tank solution ensures passivity only for the overall system.

### B. Experiments

The controllers were implemented on Rollin’ Justin with 17 DOF and three priority levels:

1) Level 1: Cartesian impedance of both arms ($m_1=12$)
2) Level 2: Joint impedance of torso ($m_2=3$)
3) Level 3: Joint impedance of upper body ($m_3=17$)

Both energy-tank approaches ensure passivity during transients, while the classical approach leads to active periods. The global energy tank offers higher performance than local tanks but may mask poor performance on single levels.

## VI. Discussion

Energy tanks act as buffers, dissipating excess energy to maintain passivity. Local tanks ensure passivity on each hierarchy level, while the global tank allows energy exchange across levels, offering higher performance but potentially masking issues on individual levels. The control law does not require force measurements, avoiding issues with time delays.

## VII. Conclusion

A generic approach to passivate multi-objective torque control with a complex task hierarchy is presented, using local and global energy tanks. Both controllers provide high performance while preserving passivity, extending the repertoire of hierarchical controllers for safety-relevant applications involving physical interaction.