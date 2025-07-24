# An Energy Tank-Based Interactive Control Architecture for Autonomous and Teleoperated Robotic Surgery

## Abstract

Introducing some form of autonomy in robotic surgery is being considered by the medical community to better exploit the potential of robots in the operating room. However, significant technological steps have to occur before even the smallest autonomous task is ready to be presented to the regulatory authorities. In this paper, we address the initial steps of this process, in particular the safety requirements of robotic surgery, i.e., providing the robot with the necessary dexterity and a stable and smooth behavior of the surgical tool. Two specific situations are considered: the automatic adaptation to changing tissue stiffness and the transition from autonomous to teleoperated mode. These situations replicate real-life cases when the surgeon adapts the stiffness of her/his arm to penetrate tissues of different consistency and when, due to an unexpected event, the surgeon has to take over the control of the surgical robot. To address the first case, we propose a passivity-based interaction with the current architecture that allows to implement stable time-varying interactive behaviors. For the second case, we present a two-layered bilateral control architecture that ensures a stable behavior during the transition between autonomy and teleoperation and, after the switch, limits the effect of initial mismatch between master and slave poses. The proposed solutions are validated in the realistic surgical scenario developed within the EU-funded I-SUR project, using a surgical robot prototype specifically designed for the autonomous execution of surgical tasks like the insertion of needles into the human body.

**Index Terms**—Energy tanks, interactive control, medical robots and systems, telerobotics.

## I. Introduction

[Content from PAGE1, excluding manuscript details]

## II. Related Works

The growing interest in reproducing a human-like behavior in many robotic tasks fostered the research in interaction control. In this section, we focus on the works addressing the main problems arising in the considered application: time-varying admittance/impedance parameters of the interactive controller and compensation of the kinematic mismatch when switching to teleoperation.

In particular, a lot of research on impedance and admittance control with variable stiffness has been done. Many researchers have addressed the problem of properly estimating online variable stiffness and damping matrices for the execution of an interactive task with an unknown environment. The time necessary for reaching a good estimate depends on the collected data as well as on the estimation algorithm, and it is hard to predict it in practical applications. In [15], a teleoperation system for needle insertion is presented. The variable stiffness of the environment is considered, estimated, and exploited for adjusting the force perceived by the user.

Several works consider the problem of developing an impedance controller with a variable stiffness, but they address only particular cases. In [16], a tracking error-dependent variable stiffness impedance controller for parallel link manipulators has been proposed in order to increase the robustness of the controlled system with respect to unknown parameters. The way stiffness can be changed is linked to the tracking error, and in case of external disturbances, the target impedance cannot be achieved.

Iterative and adaptive control strategies have been proposed in order to compensate for disturbances and for guaranteeing a stable interaction even in presence of drastic changes in the environment. The apparent stiffness of the robot is adapted in order to match the behavior of human motor control. Nevertheless, in several surgical scenarios, special interactive patterns (e.g., stiffness variations) may be necessary to match the behavior of the surgeon or for coping with the particular structure of the patient's body that is provided by pre- and intraoperative data.

In [19] and [20], an efficient human-like force tracking impedance control scheme with varying stiffness has been developed. However, the stiffness profile depends on the force tracking error and cannot be chosen a priori.

Only a limited amount of works have exploited the concept of variable impedance controllers in surgical robotics. In [21], a variable stiffness controlled manipulator was developed in order to provide insights on the design and control of rehabilitation robots. In [22], a variable impedance control was proposed, allowing the surgeon to cooperate with the robot during the execution of high-accuracy tasks in orthopedic surgery, e.g., drilling and shaping of the femur's head. However, in this approach, several assumptions on the choice of the gain matrices are necessary.

Substantial research studies addressing the transition between autonomous and shared control of a robot and considering the kinematic compensation problem are available in the literature. In [23], a shared control teleoperation architecture, obtained by merging human input and autonomous operations, is provided. However, the input from the operator perturbs the state of the robot and the transition between autonomous and teleoperation modes is not smooth. In [24], a hybrid system allowing to switch between different control modes in a teledrilling system is proposed. Nevertheless, when switching from one mode to the other, master and slave have to wait for being synchronized, and this results in a slowly reacting system. In [25], the operation modes for cooperation between manual operation and autonomous functions in intelligent teleoperation systems are discussed. In [26], Xiong et al. describe other operation modes and discuss the case of time delay. These works are mainly related to the way the operator can change the operation modes intuitively and smoothly. They do not address the problem of compensating the position error arising between master and slave when the system switches between different modes.

Several works have addressed the problem of position offset compensation. In [27], a strategy to passively compensate the steady-state position error due to packet loss is proposed. In this case, the slave moves toward the position of the master that has to be kept still by the user. In [28], a controller for improving position tracking in bilateral teleoperation via packet-switched networks is developed, while in [29], a method to reduce the position drift without violating system passivity conditions is proposed. It is worth highlighting that the mentioned approaches do not consider autonomous modes and, therefore, always produce a bilateral action, moving both master and slave.

## III. Background on Port-Hamiltonian Systems and Energy Tanks

This section provides some background on port-Hamiltonian systems and on energy tanks. For a more detailed treatment, see [8], [10], and [30].

The port-Hamiltonian framework is a generalization of standard Hamiltonian mechanics, where energetic characteristics and power exchange between subsystems are clearly identified. All physical systems, even multidomain, can be represented using the port-Hamiltonian formalism. The most common representation of a port-Hamiltonian system is

$$
\left\{\begin{array}{l}
\dot{x}=\left[J(x)-R(x)\right] \frac{\partial H}{\partial x}+g(x) u \\
y=g^T(x) \frac{\partial H}{\partial x}
\end{array}\right.
$$

where $x \in \mathbb{R}^n$ is the state vector, and $H(x): \mathbb{R}^n \rightarrow \mathbb{R}$ is the lower bounded Hamiltonian function representing the amount of energy stored in the system. Matrices $J(x)=-J(x)^T$ and $R(x) \geq 0$ represent the internal energetic interconnections and the dissipation of the port-Hamiltonian system, respectively, and $g(x)$ is the input matrix. The input $u$ and the output $y$ are dual variables and their product is (generalized) power. The pair $(u, y)$ is called power port and is the means by which the system can energetically interact with the external world. The product $u^T y$ represents the power exchanged by the system with the external world.

It can be easily shown that the following equality holds:

$$
\dot{H}(x)+\frac{\partial^T H}{\partial x} R(x) \frac{\partial H}{\partial x}=u^T(t) y(t) .
$$

This means that the power supplied to the system is either stored or dissipated, namely that a port-Hamiltonian system is passive with respect to the pair $(u, y)$. Let

$$
D(x)=\frac{\partial^T H}{\partial x} R(x) \frac{\partial H}{\partial x} \geq 0
$$

indicate the power dissipated by the system. As pointed out in [31], $D(x)$ represents a passivity margin: the larger $D(x)$, higher the passivity of the system. In other words, the larger the passivity margin, the more the system can absorb the energy generated by nonpassive actions (e.g., changing the stiffness in a viscoelastic coupling, as discussed later) while preserving its passivity.

Energy tanks, first proposed in [9], exploit this concept for building flexible and passivity preserving controllers. The energy dissipated by the system is stored in a (virtual) energy tank and can be reused for implementing any desired control action in a passivity preserving way.

More formally, the dynamics of a port-Hamiltonian system endowed with a tank is given by

$$
\left\{\begin{array}{l}
\dot{x}=[J(x)-R(x)] \frac{\partial H}{\partial x}+g(x) u \\
\dot{x}_1=\frac{\sigma}{x_t} D(x)+\frac{1}{x_t}\left(\sigma P_{\text {in }}-P_{\text {out }}\right)+u_1 \\
y_1=\left(\begin{array}{c}
y \\
y_t
\end{array}\right)
\end{array}\terea

where $x_t \in \mathbb{R}$ is the state associated with the energy storing tank, and

$$
T\left(x_t\right)=\frac{1}{2} x_t^2
$$

is the energy stored in the tank. $P_{\text {in }} \geq 0$ and $P_{\text {out }} \geq 0$ are incoming and outgoing power flows that the tank can exchange with other tanks, respectively. The pair $\left(u_t, y_t\right)$ is a power port that the tank can use to exchange energy with the external world and $y_t=\frac{\partial F}{\partial x_t}=x_t$. The parameter $\sigma \in\{0,1\}$ is used for bounding the amount of energy that can be stored in the tank. The following power balance can be easily derived from (4):

$$
\dot{T}=\sigma D(x)+\sigma P_{\text {in }}-P_{\text {out }}+u_1^T y_t
$$

which means that, if $\sigma=1$, the tank stores the power dissipated by the system $D(x)$ and the incoming power flow $P_{\text {in }}$, while the outgoing power flow $P_{\text {out }}$ is released. Furthermore, energy can be injected in / extracted from the tank via the power port $\left(u_t, y_t\right)$. In order to avoid singularities in (4), some energy must always be present in the tank (i.e., $x_t \neq 0$). Thus, it is necessary to set an arbitrarily small threshold $\varepsilon>0$ representing the minimum amount of energy that needs to be always stored. The tank has to be initialized and managed in such a way that $T\left(x_t(0)\right)>\varepsilon$ and energy extraction is prevented if $T\left(x_t\right) \leq \varepsilon$. Finally, it is necessary to set an upper bound on the amount of energy that can be stored in the tank. In fact, as described in [32], if there is no bound, the energy available can become very large as time increases, and even if the system remains passive, it would be possible to implement behaviors that are unstable in practice. Thus, $\sigma$ is set using the following policy:

$$
\sigma= \begin{cases}1, & \text { if } T\left(x_t\right) \leq T \\ 0, & \text { otherwise }\end{cases}
$$

where $T>0$ is a suitable application-dependent upper bound on the energy that can be stored in the tank.

The energy stored in the tank can be exploited for passively implementing any desired input $w \in \mathbb{R}^n$ to the port-Hamiltonian system the tank is associated with. This can be done by joining the power ports $(u, y)$ and $\left(u_t, y_t\right)$ through the following power preserving interconnection:

$$
\left\{\begin{array}{l}
u=\frac{w}{x_t} y_t=\frac{w}{x_t} x_t=w \\
u_t=-\frac{w^T}{x_t} y
\end{array}\right.
$$

implying the balance

$$
u^T y=-u_t y_t .
$$

When using (8) the energy supplied to/extracted from the port-Hamiltonian system for implementing the desired input is exactly equal to the energy extracted from/supplied to the tank. This intuitively means that no energy is generated and that the desired input can be implemented in a way to preserve passivity as long as some energy is stored in the tank (i.e. $T\left(x_t\right)>\varepsilon$).

## IV. Variable Admittance Control

Admittance control and impedance control are very effective control schemes for implementing a desired interaction behavior. Loosely speaking, impedance control is more suitable for backdrivable robots, while admittance control is more suitable for stiff robots. The robot developed within the I-SUR project has a stiff and not backdrivable structure. Therefore, we will show how to exploit tanks for implementing a variable admittance control. However, all the results developed in this section can be easily adapted for implementing a variable impedance control.

A standard admittance control scheme is reported in Fig. 1. Given a desired interaction model, namely a dynamic relation between the applied force and the pose error, given the external force and the desired pose setpoint, the corresponding position of the robot is generated and tracked by the robot by means of a lower level motion controller. We expect the latter to be designed and tuned to minimize the tracking error and optimize the dynamic response so that we can assume that the actual pose $x \in \mathbb{R}^n, n \leq 6$, of the robot end-effector is identical to its reference position $x_{\text {ref }}$. Thus, in the following, we will consider $x=x_{\text {ref }}$.

More formally, consider the following Euler-Lagrange dynamic model of a fully actuated $n$-degree-of-freedom (DOF) manipulator in the task space:

$$
\Lambda(x) \ddot{x}+\mu(x, \dot{x}) \dot{x}+F_g(x)=F_\tau+F_{\text {ext }}
$$

where $x=f(q)$ is the pose of the end-effector, obtained from the joint positions $q \in \mathbb{R}^m, m \geq n$, through the forward kinematic map $f(\cdot), F_{\text {ext }} \in \mathbb{R}^n$ is the external wrench applied to the end-effector, and $F_\tau \in \mathbb{R}^n$ is the wrench due to the controlled joint torques $\tau \in \mathbb{R}^m . \Lambda(x)=\Lambda^T(x)>0$ is the $n$-dimensional positive-definite inertia matrix, $\mu(x, \dot{x}) \in \mathbb{R}^{n \times n}$ is the matrix of the centrifugal and Coriolis terms, and $F_g(x) \in \mathbb{R}^n$ is the wrench due to the gravity. The control wrench $F_\tau$ is set by the motion controller for implementing a desired interactive behavior.

A very common interaction model adopted in standard admittance control is the multidimensional mass-spring-damper system described by

$$
\Lambda_d \ddot{x}+D_d \dot{x}+K_d \ddot{x}=F_{\text {ext }}
$$

where $\dot{x}(t)=x(t)-x_d(t)$ is the pose error. $\Lambda_d, D_d$, and $K_d$ are the $n$-dimensional symmetric and positive-definite inertia, damping, and stiffness matrices characterizing the interactive behavior. The controlled robot behaves as (11), and it is passive with respect to the pair $\left(F_{\text {ext }}, \dot{x}\right)$. In fact, consider

$$
V(\dot{x}, \dot{\bar{x}})=\frac{1}{2} \dot{x}^T \Lambda_d \dot{x}+\frac{1}{2} \dot{x}^T K_d \dot{x}
$$

as a nonnegative storage function. We have that

$$
\dot{V}=\dot{x}^T \Lambda_d \ddot{x}+\dot{x}^T K_d \dot{x} .
$$

Using (11) in (13), we obtain

$$
\dot{V}=\dot{x}^T F_{\text {ext }}-\dot{x}^T D_d \dot{x} \leq \dot{x}^T F_{\text {ext }}
$$

which implies the passivity condition

$$
V(t)-V(0) \leq \int_0^t \dot{x}^T(\tau) F_{\text {ext }}(\tau) d \tau .
$$

The passivity of the controlled behavior is a crucial characteristic of admittance control. In fact, passivity is a sufficient condition for ensuring a stability of the controlled robot both in free motion and during the interaction with any passive, possibly unknown, environment.

## V. Experimental Results

Since during the puncturing task, the robot has to behave in different ways depending on the environment it has to interact with, the entries for the stiffness matrix change during the operation. Indeed, for example, the robot can be compliant while it is in free motion, while it has to be stiff for penetrating the skin. To preserve the clarity of the presentation, the following plots will show only the results regarding the translational coordinates $\mathrm{x}, \mathrm{y}$, and $\mathrm{z}$. Similar results have been obtained for the rotational coordinates.

The evolution over time of the variable part $K_v(t)$ of the stiffness matrix is shown in Fig. 6, while the constant part is chosen as

$$
K_e=\operatorname{diag}\left\{K_{e 1}, K_{e 2}\right\}
$$

where

$$
\begin{aligned}
& K_{e 1}=\operatorname{diag}\{10,10,10\} \quad|\mathrm{N} / \mathrm{m}| \\
& K_{e 2}=\operatorname{diag}\{10,10,10\} \quad|\mathrm{N} \cdot \mathrm{m} / \mathrm{rad}| .
\end{aligned}
$$

To demonstrate that the system remains stable despite the stiffness changes, we even consider different ways of varying the stiffness profile. For example, during the movement of the robot to the position of needle change, the stiffness is augmented gradually, whereas when the robot is waiting for the needle to be mounted, the stiffness is changed instantly.

The desired Cartesian translational positions computed by the admittance controller are reported in Fig. 7. As expected, the commanded motion does not diverge over time, and the system remains stable despite the many changes of stiffness. Fig. 8(a) shows that the tracking error during the insertion of the needle (phase E) is below the acceptable value of $0.0011 \mathrm{~m}$, thanks to the high values of the stiffness in this phase, while Fig. 8(b) shows the forces measured during the same time interval.

Fig. 9 shows the behavior of the tank energy $T$. The energy thresholds are chosen as $T=10 \mathrm{~J}$ and $\varepsilon=0.1 \mathrm{~J}$. When the stiffness changes instantly, at $t=10 \mathrm{~s}$ and $t=36 \mathrm{~s}$ (yellow regions in Fig. 9), the required energy to implement this behavior is extracted from the tank. On the other hand, the gradual changes of stiffness, red areas at $t=28 \mathrm{~s}$ and $t=83.5 \mathrm{~s}$, are dissipative actions, and thus, the tank energy rises. The energy level has a big increment when the needle penetrates the skin $(t=93 \mathrm{~s})$ since this action is dissipative, but then, during the motion of the needle inside the phantom, the tank is emptied again to perform the movement, and the tank energy decreases accordingly (green region). Little energy is extracted or inserted into the tank because, as shown, for example, in Fig. 8(a), the trajectory error $\tilde{x}(t)$ is small, and then, from (24), the values of $w(t)$ used in (23) are small too.

### Switching to Teleoperation for Manual Needle Extraction and Reinsertion

In this section, we will take into account the second part of the experiment: The needle has been autonomously inserted, but due to an unexpected event, the target is missed. The system recognizes this event and switches to teleoperation. The switching instant is $t_s=137.3 \mathrm{~s}$.

According to the theory developed in the previous sections, the following design choices were made.

1) The control parameters in Fig. 2 are $K=50 \mathrm{~N} / \mathrm{m}, B=1.1 \mathrm{~N} \cdot \mathrm{s} / \mathrm{m}, R_{\mathrm{m}}=R_{\mathrm{s}}=5 \mathrm{~N} \cdot \mathrm{s} / \mathrm{m}$.
2) The following values for the energy thresholds have been selected: $T_i=20 \mathrm{~J},{ }^i T_{\text {ava }}=10 \mathrm{~J},{ }^i T_{\text {req }}=5 \mathrm{~J}, \varepsilon_i=1 \mathrm{~J}$, where $i=m, s$.
3) The nonincreasing functions $f_1(e(t))$ and $f_2(\lambda(t))$ in (35) and (36) are defined as

$$
\begin{aligned}
& f_1(e(t))=\frac{1}{2}\left[\cos \left(\frac{x(t)-\varepsilon_1}{\varepsilon_1}\right)+1\right] \\
& f_2(\lambda(t))=\frac{1}{2}\left[\cos \left(\frac{x(t)-\dot{\varepsilon}_1}{\dot{\varepsilon}_1}\right)+1\right] .
\end{aligned}
$$

With this choice, the functions smoothly change from 0 (when $e(t)=\bar{e}_2$ and $\lambda(t)=\dot{\bar{\varepsilon}}_2$ ) to 1 (when $e(t)=\bar{e}_1$ and $\lambda(t)=\bar{\lambda}_1$ ), with $\bar{e}_1=\frac{\dot{\varepsilon}_2}{2}$ and $\bar{\lambda}_1=\frac{\dot{\lambda}_2}{2}$. Indeed, the cosine is a continuous function that allows smooth transitions between the two values, while avoiding abrupt variations.
4) The thresholds related to the functions $\alpha_1(e(t))$ and $\alpha_2(\lambda(t))$ are chosen as $\bar{e}_1=0.005 \mathrm{~m}, \bar{e}_2=0.01 \mathrm{~m}, \lambda_1=0.005 \mathrm{~m}$, and $\lambda_2=0.01 \mathrm{~m}$. Such small values are justified by the surgical context, prescribing that the bilateral is progressively reduced so that the accuracy is increased.

## VI. Conclusion

Finally, since the overall teleoperation system has been formally proven to be passive, stability and reliability are guaranteed. The proposed control architecture is based on a two-layer bilateral control architecture that guarantees stability and smooth variation of the critical variables. The freedom in the choice of control parameters and functions can be exploited to adapt the proposed solution to different conditions. The proposed architecture has been implemented on a semiautonomous robotic surgical system designed within the I-SUR project, and the results of related experiments have been discussed. The techniques developed in the paper are general, and they can be extended to other more complex automated surgical scenarios where a robot has to interact with a soft tissue and where a switch between autonomous mode and teleoperation mode is necessary.

Future work aims at selecting a proper stiffness profile that reflects an online estimation of the environmental stiffness. Furthermore, it has to be investigated how the energy extracted from the tank can be modulated when its filling level is close to the minimum (i.e., degraded operating mode). The methodologies presented in this paper ensure stability of the system, which is a necessary prerequisite for safety in robotic surgery. However, to make the procedure completely safe for the patient, other higher level safety measures must be in place to monitor the proper execution of the task.