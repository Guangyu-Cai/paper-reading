---
title: "Learning Dynamic Rope Manipulation Using Task-Level Iterative Learning Control"
source: "https://arxiv.org/html/2602.21302v2"
author:
published:
created: 2026-07-14
description:
tags:
  - "clippings"
---
Krishna Suresh and Chris Atkeson  
Carnegie Mellon University  
Email: ksuresh2,cga@andrew.cmu.edu  
[https://flying-knots.github.io](https://flying-knots.github.io/)

###### Abstract

We introduce a Task-Level Iterative Learning Control method for dynamic manipulation of ropes. We demonstrate this method on a non-planar rope manipulation task called the flying knot. Using a single human demonstration and a simplified rope model, the method learns directly on hardware without reliance on large amounts of demonstration data or massive amounts of simulation. At each iteration, the algorithm inverts a model of the robot and rope by solving a quadratic program to propagate task-space errors into action updates. We evaluate performance across 7 different kinds of ropes, including chain, latex surgical tubing, and braided and twisted ropes, ranging in thicknesses of 7–25 mm and densities of 0.013–0.5 kg/m. Learning achieves a 100% success rate within 10 trials on all ropes. Furthermore, the method can successfully transfer between most rope types in 2–5 trials. [https://flying-knots.github.io](https://flying-knots.github.io/)

## I Introduction

Dynamic manipulation of deformable objects is a challenging domain for both robots and humans. Deformable objects such as ropes and cloth have many unactuated degrees of freedom and are expensive to model accurately. We enable a robot to learn the dynamic task of tying a flying knot as seen in Fig. 1. This task is performed on a rope by executing a one-handed upward and twisting motion to form a loop and then arcing to strike near the end of the rope to flip it into the loop, completing a knot.

We adapt Iterative Learning Control (ILC) [^7] to improve a command trajectory through real-world trials. ILC is a powerful tool for enabling a robot to improve its performance, as introduced by [^5]. We find that typical ILC formulations fail in deformable-object manipulation. Model-based iterative learning control uses an approximate system model to convert task errors from real trials into command corrections. When ILC is learning to track rope target motions, however, equal weighting of task errors causes learning to fail.

Task-Level ILC uses feedback of task states in addition to or instead of robot states during learning [^2]. We leverage Task-Level ILC with two key features to enable success on dynamic rope manipulation:

- Critical point objective: Instead of weighting trajectory errors equally throughout the task, we focus the robot’s attention on the error in achieving a single rope state in the trajectory. This improves learning performance and enables the transfer of the task across large parameter variations.
- Object trajectory learning: Our approach corrects the trajectory of state variables of the manipulated object, which goes beyond the usual ILC approach of correcting the robot trajectory.

We demonstrate the effectiveness of Task-Level ILC for dynamic rope manipulation. We show that learning adapts to variations in rope dynamics, demonstrations, and the system model. Overall, the key contributions of this work include the following:

- We extend ILC to refining the trajectories of unactuated degrees of freedom of manipulated objects.
- We introduce Task-Level ILC for dynamic rope manipulation, demonstrating that learning typically requires a small number of real robot trials ($<10$) and transfers across different ropes.
- We develop a new approach to ILC, which focuses attention on critical points of the error history.

![Refer to caption](raw/assets/suresh-2026-task-level-ilc/flying_knot_seq.png)

Figure 1: Stages of a flying knot by both a human and an xArm 7 robot arm over 0.56s

## II The Flying Knot Task

For this case study, we considered a variety of one dimensional object manipulations, including whipping, lassoing, throwing a rope to a target, fly casting, and attaching a rope to a distant object, such as a cleat or tree branch by manipulating one end. The flying knot task was chosen because it is impressive but achievable by humans and fast robots, and has been previously explored in robotics by [^41].

There are several types of flying knots. We focus on the Overhand Knot, in which one hand manipulates the end of a rope to tie an overhand knot, without changing the grip. As seen in Fig. 1, the flying knot consists of multiple stages. The rope starts hanging from the hand. The hand is moved upward and twisted to form a loop. The rope collides with itself, and the end of the rope is flipped through the loop. The knot tightens as the rope falls. Often, a weight is added to the end of the rope to make the task easier.

### II-A Flying Knot Objective

In this case study, we develop the idea that it is useful to define a proxy goal for learning dynamic tasks, which we refer to as a “critical point”. The use of a proxy goal can simplify the learning task by making it easier to evaluate task success. Often the conclusion of a dynamic task results in a crash or a state where error information is lost. To our surprise, paying attention to errors equally throughout task execution is often not as useful as focusing on a particular point or phase of the task. The selected proxy goal can be based on an event such as the initiation of a particular contact, on some measure of task phase, or wall clock time from some trigger event. There are many such moments in the execution of a flying knot: the initial state, the first instant the loop is formed, the maximum outward swing or height of the rope end or any other part of the rope, the rope-rope collision, the rope end passing through the loop, or the final knotted state.

We chose the moment the rope collides with itself as the critical point in this case study. We were heavily influenced by instructional materials which emphasized this collision state or “strike point” as a crucial step, as seen in Fig. 2. The overhand flying knot task can be performed in several ways, but the variations share the same qualitative topological contact event, as visualized in Fig. 2, allowing the use of a single definition of critical point. We also chose this collision point because it is easy to detect and measure and occurs before our rope tracking system gets confused by the overlap of the rope during knot formation. While this moment in flying knot execution is not a perfect predictor of task success, correctly matching this state will likely lead to a knot, so we selected this contact initiation and the corresponding rope shape and velocity as the critical point.

We provide the robot with a single demonstration of the flying knot, with the critical point manually annotated, for an example of the rope state at the critical point, to provide a goal for learning the task.

![Refer to caption](raw/assets/suresh-2026-task-level-ilc/demos.png)

Figure 2: Critical point for the flying knot shown in instructional videos 24 8. The bottom row is the critical point state across 4 demonstration variations. The shape of the rope at collision is used as the learning objective.

## III Related Work

### III-A Iterative Learning Control

Iterative learning control is a command update algorithm in which the system iteratively executes a feedforward command in the real world and maps errors into feedforward command corrections. ILC leverages a system’s repeatability to accumulate corrections across trials. Model-based ILC maps task errors to command updates with an inverse model [^4].

Classical Iterative Learning Control (ILC) studies how to improve performance on tasks that repeat over trials by updating a feedforward command based on robot trajectory errors. Early work formalized the idea of “bettering” robot operation via iterative error-based updates [^5]. In robotics, model-based ILC-style trajectory learning through practice was demonstrated on manipulators using nonlinear rigid-body robot models [^6] [^4], and subsequent work analyzed convergence and design choices for robot manipulators and other repetitive systems [^21] [^25].

Optimization-based ILC or Norm-Optimal ILC formulations showed how to incorporate constraints and compute command corrections via an optimization objective. [^30] demonstrated aggressive quadrotor trajectory tracking by leveraging convex programs to reduce errors, and [^28] applies norm-optimal methods for real-time control of industrial gantry robots.

Event-based ILC methods focus learning on a subset of the task, such as the terminal states [^13], point-to-point [^11], time windows [^35], and event triggers [^23]. In general, placing learning attention at key moments of the task has worked in many domains, including non-robotics industrial domains such as CNC machine tools, wafer-stage motion systems, injection molding, induction motor drives, automotive braking/vehicle control, rapid thermal processing, and semi-batch chemical reactors [^7].

Modern learning methods have recently been combined with ILC: using approximate simulators to propose local policy improvements [^1], framing ILC through regret minimization [^3], and explaining when ILC can outperform planning with a misspecified model [^36]. Additional work introduces ILC-style model-gradient updates in deep off-policy reinforcement learning to improve sample efficiency [^16]. In our work, we leverage optimization-based and event-based techniques to focus learning on the critical points of the task.

### III-B Deformable Object Manipulation

Quasi-static deformable object manipulation, such as the handling of rope, hair, and cloth, has been addressed in a number of works [^29] [^44] [^47] where many approaches focus on domains with strong perception and geometric priors [^32] [^45] and imitation/representation learning for rope manipulation [^27] [^18] [^20].

Dynamic deformable object manipulation focuses on faster tasks, such as whipping [^26], cloth flinging [^10] [^42], and dynamic rope knotting [^41]. Iterative Residual Policy proposes online action refinement via a learned residual dynamics model [^12]. Related works study learning for whip targeting, combining structured motion primitives with online adaptation or reinforcement learning [^40] [^39].

Recent work has further explored dynamic cable and cloth behaviors with self-supervised data collection. Real2Sim2Real learning enabled planar robot casting of cables by fitting models to physical rollouts [^22], and subsequent systems learned to dynamically manipulate fixed-endpoint cables via arcing motions [^46] or free-end cables in planar settings [^38].

In [^17], [^12], and [^46], approximate system models are learned with large-scale simulated data, but our use of model-based ILC eliminates the need for large-scale simulated data collection.

In these works, commands are executed as feedforward trajectories and demonstrate the repeatability of dynamic deformable object manipulation. Similarly, we leverage feedforward trajectories in the flying knot task.

Algorithm 1 summarizes Task-Level ILC.

Algorithm 1 Task-Level Iterative Learning Control (Task-Level ILC)

 Inputs: initial command $\mathbf{u}_{0}(t)$; reference critical-point state $\mathbf{x}^{\text{demo}}(t_{c})$; critical point $t_{c}$; inverse model $\mathcal{M}^{-1}$; $K$ iterations

 for $k=0$ to $K$ do

   $\mathbf{x}_{k}(t)\leftarrow\texttt{Trial}(\mathbf{u}_{k}(t))$

  Critical-point error $\tilde{\mathbf{x}}_{k}(t_{c})\leftarrow\mathbf{x}_{k}(t_{c})-\mathbf{x}^{\text{demo}}(t_{c})$

  Command update $\Delta\mathbf{u}_{k}(t)\leftarrow\mathcal{M}^{-1}\!\left(\tilde{\mathbf{x}}_{k}(t_{c})\right)$

  Update $\mathbf{u}_{k+1}(t)\leftarrow\mathbf{u}_{k}(t)-\Delta\mathbf{u}_{k}(t)$

 end for

## IV Method: Task-Level Iterative Learning Control

### IV-A Iterative Learning Control

Iterative Learning Control (ILC) is a data-efficient learning method for repetitive tasks. ILC maps error trajectories $\mathbf{\tilde{x}}(t)$ to command trajectory updates $\Delta\mathbf{u}(t)$ to iteratively refine a full sequence of commands. ILC can improve performance beyond methods that generate commands solely from a system model, since it can correct for unmodeled dynamics. ILC is often applied to robot trajectory-tracking problems to minimize tracking errors caused by unmodeled system dynamics (e.g., friction, cogging torques, and gear backlash and play).

ILC is well-suited when several conditions hold: the task has a fixed goal across trials, the dynamics are repeatable enough that errors persist from one trial to the next, the system can be reset to a consistent initial condition, the outcome is measureable, and the initial trajectory is close enough to the desired trajectory for the particular update operator to converge [^4]. Model-based ILC works well when the model is good enough to get the initial trajectory close enough to the desired trajectory, and for learning to converge [^4].

The command trajectory $\mathbf{u}(t)$ that ILC corrects is a function of task phase (or time) rather than a control policy, which is a function of state. A time-dependent feedforward command has O(N) parameters, where N is the number of samples. A feedback control policy typically has several orders of magnitude more parameters. We further reduce the dimensionality of the action by representing the command trajectory with knot points of splines $\mathbf{\kappa}$.

Model-based ILC uses a system model $\mathcal{M}$ which maps commands to predictions of task performance. This system model may be inaccurate, but its gradient information accelerates learning, since knowledge of the command update direction need not be obtained from the real system. ILC inverts the system model, $\mathcal{M}^{-1}$, to map the real system’s task errors into estimated command errors, which are subtracted from the current command. An overview of ILC is seen in Fig. 3. If the system model is perfect, then all errors would be removed in one step (of course, the initial command would have been perfect and there would be no errors to correct). Even when the system model is inaccurate, if the command update direction (gradient) has a component in the required command update direction, command updates accumulate and iteratively reduce task errors.

![Refer to caption](raw/assets/suresh-2026-task-level-ilc/x1.png)

Figure 3: Task-Level Iterative Learning Control System: A demonstration is converted to an initial command. The command trajectory 𝐮 ( t ) \\mathbf{u}(t) is executed on the real system, and the resulting trajectory 𝐱 \\mathbf{x}(t) is measured. The task error ~ \\mathbf{\\tilde{x}}(t) at the critical point is mapped through the inverse model ℳ − 1 \\mathcal{M}^{-1} to command trajectory corrections Δ \\Delta\\mathbf{u}(t), which are applied to the current feedforward command, closing the learning loop.

![Refer to caption](raw/assets/suresh-2026-task-level-ilc/critvseq.png)

Figure 4: Critical point vs equally-weighted objective learned commands. Each row is a real trial after 8 iterations with the corresponding objective. The green rope is the measured state from the real trial, and the red rope is the corresponding rope state from the demonstration. The rope state at 0.46s is the critical point. The objective results in a successful flying knot, while the equally-weighted objective results in failure. During learning, the robot learns to deviate from the demonstration motion to reduce tracking error along the rope, which is why the demonstration rope handle position does not match the measured rope handle position.

### IV-B Task-Level Iterative Learning Control

ILC is often used for robot trajectory tracking. In robot trajectory tracking, the objective and the command are related by the robot dynamics, which are often modeled accurately. Trajectory-tracking ILC usually weights errors along the trajectory equally.

In Task-Level ILC, task errors are similarly mapped to feedforward robot command improvements. In this work, task errors are defined as deviations in the state variables of a deformable manipulated object from a goal trajectory. In addition, we explore the use of critical point objectives, which focus learning on the error at a critical point along the trajectory.

### IV-C Critical Point Objective

The critical point is defined as a key moment in the task at which all task error reduction is targeted. Let $t_{c}$ be the selected task phase and let $\mathbf{x}^{\text{demo}}(t_{c})$ be the demonstrated task state. The corresponding objective minimizes the weighted error $\|\mathbf{x}(t_{c})-\mathbf{x}^{\text{demo}}(t_{c})\|^{2}_{\mathbf{Q}}$ rather than the integral of tracking error over the full trajectory. For the flying knot task, we manually select the state of the rope at collision time $t_{c}$ in the demonstration as the critical point as described in Section II-A. Task-Level ILC attempts to generate a feedforward trajectory before the collision to match the rope state of the demonstration only at the moment of collision. Therefore, rope-tracking errors before and after the critical point are ignored as seen in Fig. 4.

Eliminating task errors from the objective for times after contact also simplifies the modeling problem, as we only need to model free rope motion rather than the more complex contact physics during the collision.

### IV-D Command Parameterization

We parameterize the feedforward robot command as 10 Bezier curves (7 joints + 3 base translation dimensions) with 8 knot points equally spaced in time: $\mathbf{\kappa}\in\mathbb{R}^{10\times 8}$. The base translation dimensions ensure translation invariance of the task objective, with constraints that prevent the robot base location from moving Eq. 2e. The fixed total execution time of the command ($T$) is defined by the total length of the demonstration hand motion. Given $\mathbf{\kappa}$ and $T$, a Bezier curve is computed using the Bezier function $\mathcal{B}$. This spline generation of the robot’s desired trajectory command is described in more detail in Appendix Section -D.

### IV-E Robot Model

We model the robot as a kinematic chain parametrized by robot joint angles $\mathbf{q}$. Because the robot’s controller is driven by commanding desired robot joint angles, any desired trajectory $\mathbf{q}_{d}(t)$ that satisfies the robot’s joint position, velocity, and acceleration limits is modeled with the robot motion matching the command, $\mathbf{q_{d}=q}$. The desired trajectory is executed by the control system provided by the robot manufacturer, which mostly consists of joint-level PD servos. In reality, the robot motion does not exactly follow the commanded trajectory, but this error is compensated for indirectly during task-level ILC. Joint trajectories $\mathbf{q}(t)$ are mapped through the robot’s forward kinematics $\mathcal{K}(\mathbf{q}(t))$ to determine the trajectory of the fingertip (a tube holding the rope). While the real robot has more elements that can be modeled (e.g., link dynamics, actuation model), we find that this simple model is sufficient for ILC to improve the command. The robot model does not need to account for rope motion since the robot has joint actuators with a 100:1 gear ratio, so the rope’s impact on the robot’s dynamics is negligible.

### IV-F Rope Model

We model the rope as a 3D serial chain of point masses $m$ connected by fixed-distance constraints of length $l$. Each joint in the rope has a bending stiffness $k$ and a damping coefficient $b$. Since ropes are approximately self-similar, we use a single set of $m,k,b$ parameter for all but the last rope links and joints. All the ropes have a weight attached to the end, which is approximated with a different end link mass $m_{e}$. Our approximate rope model has fewer degrees of freedom than the actual ropes, with only $N=11$ links. The unit scaling of these parameters is with respect to $m$, which we set to 1, resulting in only 5 model parameters ($k,b,m_{e},l,N$).

The first point on the rope model is kinematically driven by the robot model’s fingertip position and orientation trajectory. We can define the rope dynamics model as a function $f$, which simulates the rope state given the initial rope configuration $\mathbf{z}_{0}$ and a trajectory of the first link (full derivation found in Appendix Section -C). This makes the overall system model, as seen in Fig. 5, the following:

$$
\displaystyle\mathcal{M}(\mathbf{\kappa},\mathbf{z}_{0})=f(\mathcal{K}(\mathcal{B}(\mathbf{\kappa},T)),\mathbf{z}_{0})=\hat{\mathbf{x}}(t)
$$

where $\hat{\mathbf{x}}$ is the model prediction of the rope state and $\mathbf{\kappa}$ are the command spline knot points. To simulate the rope dynamics, we implement a maximal coordinate variational integrator dynamics model from [^9]. The same fixed parameter set in Table I is used for all rope types; we do not fit model parameters to individual ropes.

### IV-G Optimization-Based Inverse Model

Methods for an optimization-based inverse model were introduced for trajectory tracking in [^30] and [^31]. A Quadratic Program (QP) is formulated as a local inverse model to minimize a quadratic task objective while satisfying linear constraints by taking the measured critical-point error from the real trial and returns a command correction that is predicted to remove that error. Previous model-based ILC methods typically use an inverse model $\mathcal{M}^{-1}$ that equally weights all parts of the trajectory, which is well-suited for trajectory tracking but is insufficient for Task-Level ILC. Our full inverse model QP is formulated as follows:

$$
\displaystyle\left(\Delta\mathbf{\kappa}_{k}^{\star},\Delta\mathbf{x}_{k}^{\star}(t)\right)
$$
 
$$
\displaystyle=\operatorname*{arg\,min}_{\begin{subarray}{c}\Delta\mathbf{\kappa}\\
\Delta\mathbf{x}(t)\end{subarray}}\bigl\|\Delta\mathbf{x}(t_{c})-\tilde{\mathbf{x}}_{k}(t_{c})\bigr\|_{\mathbf{Q}}^{2}
$$
 
$$
\displaystyle\qquad+\sum_{t\in[t_{c},T]}\|\Delta\mathbf{\kappa}\|_{\mathbf{Q}_{ft},\mathbf{b}_{ft}}^{2}
$$
 
$$
\displaystyle\qquad+\sum_{t\in[0,T]}\|\Delta\mathbf{\kappa}\|_{\mathbf{R}}^{2}
$$
 
$$
\displaystyle\Delta\mathbf{x}(t)=\mathbf{M}\,\Delta\mathbf{\kappa}
$$
 
$$
\displaystyle\Delta\mathbf{\kappa}_{\text{base}}=0
$$
 
$$
\displaystyle\mathbf{q}_{\text{min}}\leq\mathbf{J}_{p}\Delta\mathbf{\kappa}+\mathcal{B}(\mathbf{\kappa}_{k})\leq\mathbf{q}_{\text{max}}
$$
 
$$
\displaystyle\mathbf{\dot{q}}_{\text{min}}\leq\mathbf{J}_{v}\Delta\mathbf{\kappa}+\dot{\mathcal{B}}(\mathbf{\kappa}_{k})\leq\mathbf{\dot{q}}_{\text{max}}
$$
 
$$
\displaystyle\mathbf{\ddot{q}}_{\text{min}}\leq\mathbf{J}_{a}\Delta\mathbf{\kappa}+\ddot{\mathcal{B}}(\mathbf{\kappa}_{k})\leq\mathbf{\ddot{q}}_{\text{max}}
$$
 
$$
\displaystyle\tau_{\text{min}}\leq\mathbf{J}_{\tau}\Delta\mathbf{\kappa}+\mathcal{T}(\mathbf{\kappa}_{k})\leq\tau_{\text{max}}
$$
![Refer to caption](raw/assets/suresh-2026-task-level-ilc/system_model.png)

Figure 5: Left: Kinematic robot model at various stages of a command. The opaque robot is at the point of contact. Right: Graphical representation of the point mass rope model. The robot’s fingertip trajectory kinematically drives the first red dot. The remaining links are bound by distance constraints, and each joint has stiffness and damping. Together, the robot and rope models define our system dynamics model. All 3D visualizations are created using Viser 43.

The QP decision variables are the command correction $\Delta\mathbf{\kappa}$ and the predicted state correction $\Delta\mathbf{x}(t)$. Based on the measured robot motion, we simulate the rope dynamics and linearize the system model $\mathcal{M}$ about the current simulated rope trajectory $\hat{\mathbf{x}}_{k}(t)$ and current command $\mathbf{\kappa}_{k}$:

$$
\displaystyle\mathbf{M}=\left.\frac{\partial\mathcal{M}}{\partial\mathbf{\kappa}}\right|_{(\hat{\mathbf{x}}_{k}(t),\mathbf{\kappa}_{k})}
$$

The linearized dynamics constraint $\Delta\mathbf{x}(t)=\mathbf{M}\Delta\mathbf{\kappa}$ (Eq. 2d) enforces a model-based match between the command and trajectory corrections.

Since the ILC correction subtracts the returned correction from the current command, the first cost term penalizes the mismatch of the model-predicted state correction at $t_{c}$ to match the measured real-system error $\tilde{\mathbf{x}}_{k}(t_{c})=\mathbf{x}_{k}(t_{c})-\mathbf{x}^{\text{demo}}(t_{c})$. The term Eq. 2a is the state-error-tracking cost applied only at the critical point $t_{c}$ of the demonstration.

We apply a linearized quadratic tracking cost (Eq. 2b) to the fingertip trajectory at $t>t_{c}$ to match the demonstrator’s follow-through motion. The weighting matrices $\mathbf{Q}_{ft},\mathbf{b}_{ft}$ are derived in Appendix Section -A2.

A control cost (Eq. 2c) penalizes updates to the 7 arm joints at each knot point for all $t$.

Constraint terms Eqs. 2f to 2i enforce position, velocity, acceleration, and torque limits on the commanded trajectory. Each has the form

$$
\mathbf{x}_{\min}\;\leq\;\mathbf{J}_{x}\,\Delta\mathbf{\kappa}+\mathbf{B}_{x}(\mathbf{\kappa}_{k})\;\leq\;\mathbf{x}_{\max},
$$

where $\mathbf{B}_{x}(\mathbf{\kappa}_{k})$ evaluates the current spline-parameterized command (or its derivative for velocity and acceleration, or its torque prediction $\mathcal{T}$ from the rigid-body inverse dynamics), and $\mathbf{J}_{x}$ is the linearization of that mapping with respect to the command knot points. The total quantity is bounded so that the updated command stays within the robot’s joint position, velocity, acceleration, and torque limits.

The base-translation constraint (Eq. 2e) prevents the 3 base-translation degrees of freedom from varying across timesteps within a single command, so the robot base stays at a fixed location throughout each trial. ILC can still shift this fixed base location between iterations.

A detailed list of all cost and constraint parameters can be found in Appendix Section -A1. The inverse model returns the command component of the QP solution, $\mathcal{M}^{-1}(\tilde{\mathbf{x}}_{k}(t_{c}))=\Delta\mathbf{\kappa}_{k}^{\star}$.

After linearization, the objective is quadratic and the constraints are linear, so this inverse-model subproblem is a convex QP. We formulate the QP using the Drake optimization toolbox [^33] and leverage the Clarabel Solver [^15] to solve for the command update.

TABLE I: Rope Model Parameters

| Parameter | Value |
| --- | --- |
| Stiffness | $1\times 10^{5}$ |
| Damping | $50$ |
| Link Mass | $1$ |
| End Mass | $5$ |
| Simulation Timestep | $0.005$ |
| \# Links | $11$ |
| Link Length | $0.1$ m |

The unit scaling of stiffness, damping, and end mass is defined with respect to a link mass of 1.

### IV-H Demonstration

We provide the learning system with a single demonstration of the flying knot, performed on rope 1. When evaluating learning on other rope types, the same rope 1 demonstration is used as the starting point. For a given demonstration, we track the full hand trajectory and the trajectory of the rope up to collision. We use Vicon Vantage 16 motion capture [^37] to track 11 markers in 3D, which correspond to the joints in the rope model. The collision point in the demonstration is manually annotated and used as the critical point $t_{c}$; the algorithm does not discover this event autonomously. A search procedure is then performed to find the start and end time of the demonstration as described in Appendix Section -E. $\mathbf{h}(t)$ is the 3D pose trajectory of the hand, $\mathbf{x}^{\text{demo}}(t)$ is the rope trajectory, and $t_{c}$ is the collision point.

### IV-I Initial Guess

Given a demonstration of the flying knot, we initialize learning with $\mathbf{\kappa}_{0}$ to track the hand fingertip trajectory $h(t)$. We solve the following trajectory optimization problem to minimize fingertip tracking error while satisfying the joint dynamics limits:

$$
\displaystyle\min_{\mathbf{\kappa}}\quad
$$
 
$$
\displaystyle\sum_{t_{i}\in[0,T]}\bigl\|\mathbf{e}_{h}(t_{i})\bigr\|_{\mathbf{W}_{h}}^{2}+w_{j}\sum_{t_{i}\in[0,T]}\bigl\|\dddot{\mathbf{q}}(t_{i})\bigr\|^{2}
$$
 
$$
\displaystyle\mathbf{q}(t)=\mathcal{B}(\mathbf{\kappa},T)
$$
 
$$
\displaystyle\mathbf{q}_{\min}\leq\mathbf{q}(t)\leq\mathbf{q}_{\max}
$$
 
$$
\displaystyle\mathbf{\dot{q}}_{\min}\leq\mathbf{\dot{q}}(t)\leq\mathbf{\dot{q}}_{\max}
$$
 
$$
\displaystyle\mathbf{\ddot{q}}_{\min}\leq\mathbf{\ddot{q}}(t)\leq\mathbf{\ddot{q}}_{\max}
$$
 
$$
\displaystyle\dot{\mathbf{q}}(0)=\dot{\mathbf{q}}(T)=\mathbf{0}
$$
 
$$
\displaystyle z_{\mathrm{tip}}(\mathbf{q_{0}})\geq z_{\min}
$$

where $\mathbf{q_{0}}$ is the initial configuration of the robot. A detailed explanation of the hand-tracking objective $\mathbf{e}_{h}$, weighting $\mathbf{W}_{h}$, and constraints is provided in Appendix Section -F. The initial-guess optimization is non-convex because the hand-tracking objective depends nonlinearly on the command through the Bezier curve, robot forward kinematics, and rotation error. We therefore formulate this trajectory optimization problem using the Drake toolbox [^33] and solve using the SNOPT nonlinear solver [^14].

![Refer to caption](raw/assets/suresh-2026-task-level-ilc/ropes.png)

Figure 6: 7 different rope types used for evaluation, including 4 braided ropes of various thickness and mechanical properties, a large twisted rope, a chain, and latex surgical tubing (very elastic); ranging in thicknesses of 7–25 mm and densities of 0.013–0.5 kg/m. Each rope has a mass affixed to aid in the formation of the knot.

## V Evaluation

We evaluate the learning algorithm’s performance across 7 rope types (Fig. 6) and provide 4 variations of the flying knot demonstration (Fig. 2). We define a successful flying knot as achieving a topological overhand knot in a rope hanging down at the end of motion (e.g. a knot being tied then untied during motion is a failure). Given a flying-knot demonstration, we compare the success of our Task-Level ILC approach against two common approaches: direct tracking of the human demonstrator (hand motion) and equally-weighted ILC learning of task progression (rope motion). The resulting commands are tested for robustness with 40 trials. We then evaluate the transfer of a learned command across rope types and, finally, assess the system’s learning robustness with respect to model parameters.

### V-A Experiment Setup

We conduct flying knot experiments using the xArm 7 robot arm [^34]. Both command execution and robot motion measurement occur at 250Hz. Commands that exceed the robot’s joint dynamics limits in position, velocity, acceleration and torque result in mid-motion faults and task failure.

Each rope is marked with 11 rings of 12 mm wide retroreflective tape wrapped around the rope evenly spaced 10 cm apart along its length. These markers are tracked with a Vicon Vantage 16 motion capture system [^37] at 200Hz. Tracking of rope markers often fails once the rope-rope collision occurs due to occlusion and marker misidentification across the different cameras, resulting in bad 3D reconstructions. Hand position is also tracked using 4 motion-capture markers. All ropes are 1.1 meters long. The rope is always started in a static state hanging down from the fingertip, though some types of ropes are stiff enough to have curved resting states.

![Refer to caption](raw/assets/suresh-2026-task-level-ilc/x2.png)

Figure 7: Initial command fingertip trajectory: positions of the demonstration fingertip trajectory are dashed, and the robot’s commanded initial attempt to track the demonstration is solid. The robot fails to track the demonstration motion due to kinematic and dynamic constraints.

### V-B Demonstration Hand Tracking

For each demonstration type, we apply our initial-guess generation procedure, as described in Section IV-I, to assess how well the robot performs the task solely by following the demonstrated hand motion. In every rope-demonstration type for rope 1, the robot is unable to tie the flying knot when attempting to follow the demonstration trajectory. However, for certain ropes, such as 4 and 7, following the demonstration nearly achieves a flying knot, even though the demonstration provided was using rope 1.

Generating a command to exactly track the demonstrated hand motion is not possible due to the limits of the robot’s joint ranges, maximum velocities, accelerations, and torques, as well as the differences between human and robot morphology. An example demonstrated fingertip trajectory and the corresponding initial attempt by the robot are visualized in Fig. 7. While the robot can capture the demonstrator’s overall motion, it cannot match it exactly, resulting in failures when attempting to tie the flying knot.

### V-C Weighted Error Objectives

We compare the performance of the learning algorithm under two objectives: the critical point objective (ours) and the equally-weighted objective. The critical point objective, which focuses on errors at one point in the trajectory, is described in Section IV-C. The equally-weighted objective weights rope tracking errors equally along the pre-collision trajectory and is commonly found in trajectory tracking and behavior cloning.

As seen in Fig. 4, the critical-point objective aligns the rope state at a single point, at the collision, allowing for different rope motion beforehand, and results in success.

Overall, we find that the critical point objective is crucial to task success. The equal weighting objective increases errors at the critical point, compared to the critical point objective, to reduce errors at earlier parts of the trajectory that matter less for task success. Over trials, the error at the critical point is reduced as shown in Fig. 8. This error difference at the critical point for equally-weighted learning causes failure.

![Refer to caption](raw/assets/suresh-2026-task-level-ilc/iterations.png)

Figure 8: Rope configuration at the critical point over learning iterations: Real rope (green) and demonstration rope (red) states for 5 iterations of Task-Level ILC where trial 5 resulted in a flying knot.

![Refer to caption](raw/assets/suresh-2026-task-level-ilc/all_ropes.png)

Figure 9: Left: Critical point objective for 7 rope types over 10 iterations (rope 3 and 7 end early due to marker tracking failures). Squares represent the first successful flying knot, and every large solid dot represents a subsequent successful knot. Right: Execution on the real robot of successful flying knot execution on 7 rope types overlaid. The red highlighted frame is the critical point.

### V-D Learned Command Success Rate

Once a successful trajectory is learned, the robot has a 100% success rate with the learned command. We evaluated this success rate by repeating 40 trials of a learned command for each condition. Execution on the real robot is performed immediately after learning, and longer-term changes to the system dynamics, such as rope wear or breaking in (changes to stiffness and damping), and robot motor temperature, might require additional learning iterations. Continued learning can degrade the command, so learning is stopped after the first success (see Section VI-C for discussion of high-frequency error amplification over iterations).

### V-E Learning Across Rope Types

We evaluate the learning of the flying knot across 7 different ropes (as shown in Fig. 6) to demonstrate the robustness of Task-Level ILC to varying system dynamics. A detailed list of parameters of each rope can be found in Appendix Section -B. In general, ropes span a thickness of 7–25 mm, a density of 0.013–0.5 kg/m, and are composed of materials such as cotton, polyester, steel, and latex. All ropes have a mass fixed to the end with masses selected to allow the human demonstrator to execute the flying knot.

Task-Level ILC successfully learns the flying knot on all 7 rope types within 10 trials, starting from the same single rope 1 demonstration in all cases. The successful flying knot commands are shown in Fig. 9. Each command starts the rope in a different initial state and performs the upward twisting motion at different speeds. Once the critical moment is reached, all ropes form the same loop shape and allow the knot to be formed. The differences in learned commands show that variation in the rope dynamics requires variation in the command. As seen in Fig. 9, the learning algorithm succeeds in fewer trials for some rope types (3, 6, 7) and requires more iterations for others (2, 5), but does not exceed 10 trials on any rope. Overall, the learning process reduces the cost of the critical point objective across trials; however, because we are not performing a line search on the real-system objective, the cost can increase after a poor command update, as with rope 4. Furthermore, repeated updates after success can lead to failure, as in trial 9 of rope 2; model errors are well known to cause ILC to slowly amplify high-frequency errors over iterations [^6].

### V-F Learning Across Demonstration Variation

For rope 1, we test whether variations in the demonstration impact learning with the 4 different flying knot demonstration types seen in Fig. 2 (Fast, Slow, Swipe, and Inverse). The total demonstration time lengths vary, ranging from 0.69s to 1.04s. The learning algorithm learns the flying knot in all 4 cases. Fig. 10 shows the learning progression, with Demo Type 2 requiring the most trials.

![Refer to caption](raw/assets/suresh-2026-task-level-ilc/x3.png)

Figure 10: Learning cost over trials for 4 demonstration types. Large solid dots represent times when the flying knot succeeded, and squares are the first successful command. Learning on Demo Types 1 and 3 is truncated due to rope tracking failures.

### V-G Transfer of Learned Command

Given a demonstration on rope 1 and a successful command on all 7 ropes, we attempt to transfer the learned commands between rope types. The successful command on rope A initiates learning on rope B and has a maximum of 10 trials to improve. A grid of the number of trials required to adapt the command to rope B is shown in Fig. 11. For the majority of transfers, the number of trials is larger than zero, showing that the dynamics of the ropes are different enough to require different commands. For rope 7 (9mm latex surgical tubing), no additional learning is required, indicating that rope 7 is more robust to command variations. Additionally, learning fails when commands from ropes 5 and 6 are transferred to ropes 2 and 3. In all three cases, learning iteratively reduces the objective but requires larger adjustments and cannot fully correct the command within the 10 allotted trials.

![Refer to caption](raw/assets/suresh-2026-task-level-ilc/x4.png)

Figure 11: Number of trials to transfer a successful command from rope A to rope B. Red cells are for transfer that did not succeed within 10 trials.

TABLE II: Effect of model parameters on learning performance.

| Stiffness | End Mass | Trials Until Success |
| --- | --- | --- |
| $\mathbf{1\times 10^{5}}$ | 5 | 4 |
| $1\times 10^{4}$ | 5 | 4 |
| $1\times 10^{3}$ | 5 | 4 |
| $1\times 10^{2}$ | 5 | $>10$ |
| $1\times 10^{5}$ | 0.05 | 8 |
| $1\times 10^{5}$ | 0.005 | 6 |

### V-H Sensitivity to Model Parameters

ILC can make command corrections in a sample-efficient manner by leveraging a system model. We evaluate the effects of changes to the model parameters by learning on rope 1 with a single demonstration, while varying the system model stiffness $k$ and end mass $m_{e}$. These parameters were selected for their dominant impact on the model dynamics. In Table II, we evaluate learning performance with different rope model parameters (bold represents the default parameters used across all experiments). For order-of-magnitude changes in the model parameters, learning can still succeed. There are cases in which learning fails, such as when stiffness is too low, in which a single command update places the trials in an unrecoverable execution regime. Similarly, for runs with low end-mass values, updates after a successful knot quickly drive the command away from success.

## VI Discussion

### VI-A Learning on the Real System

Task-Level ILC enables task learning on real systems in a sample-efficient manner, using approximate system models to convert real-system errors into command updates. In simulation-based learning methods, such as policy learning on a simulated system model and Sim2Real transfer with domain randomization, issues often arise due to gaps between the system model and the real system. Model learning attempts to overcome this by adapting the model used for policy optimization with real data, but often suffers from limitations of the model structure. Domain randomization builds robustness into the policy by optimizing over a range of models. Policy optimization with domain randomization is a worst-case robust control design procedure, where performance is degraded in all situations relative to the performance with an accurate model for that situation.

Learning directly on the real system eliminates these issues by allowing the real system to be captured in the learning process. The models used for learning do not need to accurately predict future system states, as illustrated by the large discrepancies between the model predictions and the real rope, yet learning still occurs. These approximate models still provide sufficiently accurate gradient information. As shown in Section V-H, the learning process is robust to large variations in the parameters, eliminating challenges in accurate model parameter estimation and domain-randomization.

### VI-B Critical Points for Task-Level Learning

In evaluating the learning system on the flying knot task, we demonstrate that a critical point enables learning, robustness, and transfer. In Section V-C, we show that an equally-weighted objective failed to achieve the task because errors at the critical point are larger than with a critical point objective (as visualized in Fig. 4).

Would critical point objectives improve Behavior Cloning (BC)? BC achieves tasks not by iteratively improving performance but instead by capturing the task distribution with a range of demonstrations. Direct teleoperation of the robot is often required for BC to ensure real system dynamics are captured in demonstrations. In BC, task transfer often struggles across domains and robots. [^19] addresses some of these challenges by leveraging an instructor to guide learning toward subgoals. One can view the subgoals as critical points.

### VI-C Command Learning

For tying a flying knot, we are learning a feedforward command. Repeated execution of commands results in reproducible behavior. By learning a feedforward command, the learning problem is reduced to searching for commands as a function of time ($\mathbf{u}(t)$) rather than a general feedback policy ($\mathbf{u}(\mathbf{x})$). We further reduce the dimensionality by representing $\mathbf{u}(t)$ with a set of knot points $\mathbf{\kappa}$. This reduction in the dimensionality of the learning problem comes at a cost of the approach being unable to handle unstable systems or situations with large random perturbations. ILC can be applied to unstable systems that have been stabilized [^30].

ILC can suffer from high frequency instabilities. Mechanical systems with energy loss are inherently low-pass filters. Inverse models of these systems become high-pass filters. Learning using an inverse model as the learning operator across trials amplifies high-frequency modeling inaccuracies [^6].

We parameterize the command trajectory using knot points of splines ($\mathbf{\kappa}$) producing motions that are smooth and continuous. This reduces the effect of high frequency instabilities.

Manipulation of deformable objects is well-suited for command learning, as errors are often repeatable, and with energy loss, the system dynamics are usually stable.

### VI-D Limitations and Future Work

Our learning system is robust to changes in rope dynamics, different demonstrations, and model parameters. Further robustness evaluations can assess changes in rope length, non-uniform density, and end mass.

Our current method has several main failure modes that degrade learning. First, because the system model does not account for collisions, rope collisions with the robot body or the rope handle can cause the inverse model to apply unrecoverable command corrections. Second, the selected critical point of the rope collision is sometimes insufficient for fully completing the flying knot, as the rope may tie the knot and execute a follow-through motion, which unties the knot before the rope settles. Similarly, lighter ropes can get caught on the finger surface and be unable to tighten the knot. Third, marker tracking is only accurate up to rope collision, and since we set the critical point to be at a fixed time point from the start of motion, if the rope contact happens early, rope state estimation may fail. Lastly, learning can diverge and get stuck in a local minimum. Learning divergence occurs more frequently when task errors are too large at the critical point, and the linearized model is not accurate enough.

Avenues for future research include autonomous selection of critical points from demonstrations or instructional materials, identification of model structures for a given task, and learning from lower-fidelity demonstrations/measurements. Specifying the critical point as the rope collision point is a crucial input to our learning approach, and the autonomous identification of these critical points will enable a robot to learn efficiently. Potential selection criteria for critical points could include: human instruction, changes in contact state; dynamical extrema (position, velocity, acceleration, jerk, curvature, etc.); closest/furthest approach points; dynamic bifurcations/branch points; and dynamical funneling points.

## VII Conclusion

We demonstrate the success of Task-Level Iterative Learning Control for dynamic rope manipulation by learning to tie overhand flying knots in less than 10 trials. We introduce the notion of a critical-point objective and demonstrate task-level learning of unactuated system dynamics. Given a single demonstration of the task, our learning system learns to tie a flying knot across 7 different rope types and 4 demonstration variations. We show that a learned command achieves a 100% success rate and transfers across most rope types in 2–5 trials.

## References

### \-A Inverse Model Additional Details

#### \-A1 Inverse Model Parameters

TABLE III: Inverse Model Parameters

| Parameter | Value |
| --- | --- |
| $w_{\text{control}}$ | $\;0.5\;$ |
| $w_{\text{critical pos}}$ | $\;25\;$ |
| $w_{\text{critical vel}}$ | $\;0.00375\;$ |
| $w_{pc}$ | $\;100\;$ |
| $w_{vc}$ | $\;0.1\;$ |
| $w_{Rc}$ | $\;5\;$ |
| $w_{pft}$ | $\;1\;$ |
| $w_{vft}$ | $\;0.1\;$ |
| $w_{Rft}$ | $\;0.1\;$ |
| $w_{\text{ft velocity}}$ | $\;0.5\;$ |
| $\mathbf{q_{\min}}$ | $[\;-6.28,-1.8,-6.28,-0.19,-6.28,-1.69,-6.28\;]$ |
| $\mathbf{q_{\max}}$ | $[\;6.28,1.9,6.28,3.92,6.28,3.14,6.28\;]$ |
| $\mathbf{\dot{q}_{\min}}$ | $-[3.14,3.14,3.14,3.14,3.14,3.14,3.14]$ |
| $\mathbf{\dot{q}_{\max}}$ | $[3.14,3.14,3.14,3.14,3.14,3.14,3.14]$ |
| $\mathbf{\ddot{q}_{\min}}$ | $-[100,100,100,100,100,100,100]$ |
| $\mathbf{\ddot{q}_{\max}}$ | $[100,100,100,100,100,100,100]$ |
| $\mathbf{\tau_{\min}}$ | $-[130,130,40,40,40,20,20]$ |
| $\mathbf{\tau_{\max}}$ | $[130,130,40,40,40,20,20]$ |

We use the shorthand $\|\mathbf{a}\|_{\mathbf{W}}^{2}:=\mathbf{a}^{\top}\mathbf{W}\mathbf{a}$. The critical-point objective weights the rope-marker position and velocity errors at $t_{c}$ with a diagonal matrix

$$
\displaystyle\mathbf{Q}:=\operatorname{diag}\!\left(w_{\text{critical pos}}\mathbf{I}_{3N},\;w_{\text{critical vel}}\mathbf{I}_{3N}\right),
$$

where $N$ is the number of rope markers (links). In general, $w$ is a diagonal cost element for a cost matrix. The control-update regularizer is $\|\Delta\mathbf{\kappa}\|_{\mathbf{R}}^{2}$ with a diagonal $\mathbf{R}$ applying $w_{\text{control}}$ to the 7 arm-joint update variables. For the follow-through, we penalize the end-effector tracking error in Section -F with a time-varying diagonal weight matrix

$$
\displaystyle\mathbf{W}(t):=\operatorname{diag}\!\bigl(w_{p}(t)\mathbf{I}_{3},\;w_{R}(t)\mathbf{I}_{3},\;w_{v}(t)\mathbf{I}_{3}\bigr),
$$

using $(w_{p},w_{R},w_{v})=(w_{pc},w_{Rc},w_{vc})$ at $t=t_{c}$ and $(w_{p},w_{R},w_{v})=(w_{pft},w_{Rft},w_{vft})$ for $t>t_{c}$. The parameter $\mathbf{q}$ corresponds to the seven robot joint angles, with minimum and maximum constraints imposed on angles, velocities, and accelerations. $\tau$ is the predicted joint torque as given by the inverse dynamics of a full robot model. All values used in our experiments are listed in Table III.

#### \-A2 QP Hand Tracking Objective

For $t\in[t_{c},T]$, we encourage the robot to match the demonstrator’s follow-through motion by penalizing the end-effector tracking error. The follow-through reference is continuously matched to the robot motion at $t_{c}$, avoiding discontinuities that would alter the tracking cost. Let $\mathbf{e}_{ft}(t;\mathbf{\kappa})$ denote the end-effector error vector defined in Section -F; this error depends on $\mathbf{\kappa}$ nonlinearly through the Bezier spline $\mathcal{B}(\mathbf{\kappa},T)$ and the robot forward kinematics. We linearize $\mathbf{e}_{ft}$ about the current knot points $\mathbf{\kappa}_{k}$ with respect to a small command update $\Delta\mathbf{\kappa}$:

$$
\displaystyle\mathbf{e}_{ft}\bigl(t;\mathbf{\kappa}_{k}+\Delta\mathbf{\kappa}\bigr)
$$
 
$$
\displaystyle\approx\mathbf{e}_{ft}\bigl(t;\mathbf{\kappa}_{k}\bigr)+\mathbf{J}_{ft}(t)\,\Delta\mathbf{\kappa},
$$
$$
\displaystyle\mathbf{J}_{ft}(t)
$$
 
$$
\displaystyle:=\left.\frac{\partial\mathbf{e}_{ft}(t;\mathbf{\kappa})}{\partial\mathbf{\kappa}}\right|_{\mathbf{\kappa}=\mathbf{\kappa}_{k}}.
$$

For notational simplicity, we write $\mathbf{e}_{ft}(t):=\mathbf{e}_{ft}(t;\mathbf{\kappa}_{k})$. We apply a weighted least-squares cost

$$
\displaystyle\bigl\|\mathbf{e}_{ft}\bigl(t;\mathbf{\kappa}_{k}+\Delta\mathbf{\kappa}\bigr)\bigr\|_{\mathbf{W}(t)}^{2}
$$
 
$$
\displaystyle\qquad\approx\bigl\|\mathbf{e}_{ft}(t)+\mathbf{J}_{ft}(t)\,\Delta\mathbf{\kappa}\bigr\|_{\mathbf{W}(t)}^{2}.
$$

Expanding and dropping the constant term $\|\mathbf{e}_{ft}(t)\|_{\mathbf{W}(t)}^{2}$ yields a quadratic function of the command update:

$$
\displaystyle\|\Delta\mathbf{\kappa}\|_{\mathbf{Q}_{ft}(t),\mathbf{b}_{ft}(t)}^{2}
$$
 
$$
\displaystyle:=\Delta\mathbf{\kappa}^{\top}\mathbf{Q}_{ft}(t)\,\Delta\mathbf{\kappa}
$$
 
$$
\displaystyle\quad+\mathbf{b}_{ft}(t)^{\top}\Delta\mathbf{\kappa},
$$
$$
\displaystyle\mathbf{Q}_{ft}(t)
$$
 
$$
\displaystyle=\mathbf{J}_{ft}(t)^{\top}\mathbf{W}(t)\mathbf{J}_{ft}(t),
$$
$$
\displaystyle\mathbf{b}_{ft}(t)
$$
 
$$
\displaystyle=2\,\mathbf{J}_{ft}(t)^{\top}\mathbf{W}(t)\mathbf{e}_{ft}(t).
$$

TABLE IV: List of Rope parameters

| ID | Name | Material | Diameter \[mm\] | Density \[kg/m\] | End Weight \[g\] |
| --- | --- | --- | --- | --- | --- |
| 1 | #10 Sash Spot Cord | Cotton | 9 | 0.040 | 18 |
| 2 | #14 Spot Cord | Cotton | 12 | 0.081 | 80 |
| 3 | Soft Braided | Cotton | 15 | 0.076 | 80 |
| 4 | Shoe Lace | Cotton | 7 | 0.014 | 5 |
| 5 | Thick Twisted | Cotton | 25 | 0.139 | 50 |
| 6 | 3/8” Chain | Steel | 20 | 0.514 | 50 |
| 7 | 3/8” Surgical Tubing | Latex | 9 | 0.026 | 18 |

### \-B Rope Parameters

Rope types were selected to evaluate the algorithm robustness to diameter, material, and density. The end weight for each rope was chosen as the lightest weight that allowed the human demonstrator to verify the flying knot was achievable on that rope; the only demonstration used as the learning starting point is the rope 1 demonstration. A full list of parameters for each rope type used in evaluation can be found in Table IV.

### \-C Rope Dynamics Model

We model the rope as a serial chain of $N$ point masses in maximal coordinates and simulate it with the first-order variational integrator described by [^9]. In maximal coordinate dynamics formulations, each link of the rigid body mechanism is tracked in a global frame. Interactions between links such as joints or contacts are represented explicitly in an optimization problem solved at each timestep. Similarly, in our rope model, we do not track the joint angles between links but instead only track the global position and velocity of each link, then impose constraints to enforce the serial structure of the rope. Let $\mathbf{p}_{k}^{(i)}\in\mathbb{R}^{3}$ and $\mathbf{v}_{k}^{(i)}\in\mathbb{R}^{3}$ denote the position and velocity of link $i\in\{1,\dots,N\}$ at discrete timestep $k$. We stack all link positions and velocities as

$$
\displaystyle\mathbf{p}_{k}
$$
 
$$
\displaystyle:=\begin{bmatrix}{\mathbf{p}_{k}^{(1)}}^{\top}&\cdots&{\mathbf{p}_{k}^{(N)}}^{\top}\end{bmatrix}^{\top}\in\mathbb{R}^{3N},
$$
$$
\displaystyle\mathbf{v}_{k}
$$
 
$$
\displaystyle:=\begin{bmatrix}{\mathbf{v}_{k}^{(1)}}^{\top}&\cdots&{\mathbf{v}_{k}^{(N)}}^{\top}\end{bmatrix}^{\top}\in\mathbb{R}^{3N},
$$

and define the rope state (used throughout the paper) as $\mathbf{x}_{k}:=\begin{bmatrix}\mathbf{p}_{k}^{\top}&\mathbf{v}_{k}^{\top}\end{bmatrix}^{\top}$. The integrator also introduces constraint multipliers $\boldsymbol{\lambda}_{k}$, and we denote the full maximal-coordinate variable by $\mathbf{z}_{k}:=\begin{bmatrix}\mathbf{p}_{k}^{\top}&\mathbf{v}_{k}^{\top}&\boldsymbol{\lambda}_{k}^{\top}\end{bmatrix}^{\top}$. In Eq. 1, $\mathbf{z}_{0}$ denotes the initial rope configuration, with $\boldsymbol{\lambda}_{0}=\mathbf{0}$.

The rope is driven by the robot fingertip position $\mathbf{p}_{\mathrm{tip},k}\in\mathbb{R}^{3}$, which we treat as an input. The fingertip orientation is used to compute the bending stiffness and damping forces applied to the first link of the rope, capturing how the hand’s orientation steers the rope at the grip. We enforce inextensibility via fixed-distance constraints between adjacent links of the rope and between the fingertip and the first link:

$$
\displaystyle g(\mathbf{p}_{k},\mathbf{p}_{\mathrm{tip},k})
$$
 
$$
\displaystyle:=\begin{bmatrix}\|\mathbf{p}_{k}^{(1)}-\mathbf{p}_{\mathrm{tip},k}\|^{2}-l^{2}\\
\|\mathbf{p}_{k}^{(2)}-\mathbf{p}_{k}^{(1)}\|^{2}-l^{2}\\
\vdots\\
\|\mathbf{p}_{k}^{(N)}-\mathbf{p}_{k}^{(N-1)}\|^{2}-l^{2}\end{bmatrix}=\mathbf{0}.
$$

Let $\mathbf{G}(\mathbf{p}_{k},\mathbf{p}_{\mathrm{tip},k}):=\frac{\partial g}{\partial\mathbf{p}}(\mathbf{p}_{k},\mathbf{p}_{\mathrm{tip},k})\in\mathbb{R}^{N\times 3N}$ be the constraint Jacobian, and let $\boldsymbol{\lambda}_{k}\in\mathbb{R}^{N}$ be the corresponding Lagrange multipliers, so that constraint forces on the links are $\mathbf{G}^{\top}\boldsymbol{\lambda}_{k}$.

The mass matrix is block diagonal,

$$
\displaystyle\mathbf{M}_{\mathrm{mass}}:=\operatorname{diag}\!\left(m\mathbf{I}_{3(N-1)},\;m_{e}\mathbf{I}_{3}\right)\in\mathbb{R}^{3N\times 3N},
$$

and we collect gravity and internal bending forces (stiffness $k$ and damping $b$) into a single force vector $\mathbf{f}(\mathbf{p},\mathbf{v})\in\mathbb{R}^{3N}$. With these definitions, a single timestep of the variational integrator can be written as an implicit system in the unknowns $(\mathbf{p}_{k+1},\mathbf{v}_{k+1},\boldsymbol{\lambda}_{k+1})$:

$$
\displaystyle\mathbf{0}
$$
 
$$
\displaystyle=\mathbf{p}_{k+1}-\mathbf{p}_{k}-\Delta t\,\mathbf{v}_{k+1},
$$
$$
\displaystyle\mathbf{0}
$$
 
$$
\displaystyle=\mathbf{M}_{\mathrm{mass}}(\mathbf{v}_{k+1}-\mathbf{v}_{k})-\Delta t\,\mathbf{f}(\mathbf{p}_{k+1},\mathbf{v}_{k+1})
$$
 
$$
\displaystyle\quad-\Delta t\,\mathbf{G}(\mathbf{p}_{k+1},\mathbf{p}_{\mathrm{tip},k+1})^{\top}\boldsymbol{\lambda}_{k+1},
$$
$$
\displaystyle\mathbf{0}
$$
 
$$
\displaystyle=g(\mathbf{p}_{k+1},\mathbf{p}_{\mathrm{tip},k+1}).
$$

We solve Eqs. 11, 12 and 13 with Newton’s method at each timestep to roll out the rope trajectory, which defines the simulator $f$ used in Eq. 1.

To compute the local linear model used in the inverse-model QP, we differentiate an implicit system. Let $\tilde{\mathbf{z}}$ stack all per-timestep unknowns $(\mathbf{p}_{k},\mathbf{v}_{k},\boldsymbol{\lambda}_{k})$ over the horizon and let $\tilde{\mathbf{u}}$ stack the driven fingertip positions $\mathbf{p}_{\mathrm{tip},k}$. Concatenating Eqs. 11, 12 and 13 over time yields an implicit residual $\tilde{f}(\tilde{\mathbf{z}},\tilde{\mathbf{u}})=\mathbf{0}$. For a solution $(\tilde{\mathbf{z}}^{\ast},\tilde{\mathbf{u}}^{\ast})$, the implicit function theorem gives

$$
\displaystyle\frac{\partial\tilde{\mathbf{z}}^{\ast}}{\partial\tilde{\mathbf{u}}}=-\left(\frac{\partial\tilde{f}}{\partial\tilde{\mathbf{z}}}\right)^{-1}\frac{\partial\tilde{f}}{\partial\tilde{\mathbf{u}}}.
$$

In practice we do not form the inverse explicitly and instead solve linear systems with the KKT Jacobian $\frac{\partial\tilde{f}}{\partial\tilde{\mathbf{z}}}$. We implement the residual and its derivatives in Drake using automatic differentiation [^33], and apply the chain rule through the Bezier spline and forward kinematics to obtain the system-model linearization $\mathbf{M}$ in Eq. 3.

### \-D Command Parametrization

The Bezier trajectory $\mathcal{B}$ is parametrized by the 8 knot points $\mathbf{\kappa}\in\mathbb{R}^{10\times 8}$ equally spaced in the time interval from 0 to $T$. $\mathcal{B}$ is defined as follows, with $b_{i}$ representing the Bernstein basis functions:

$$
\displaystyle\mathcal{B}(\mathbf{\kappa},T)
$$
 
$$
\displaystyle=\sum_{i=0}^{N}\mathbf{\kappa}_{i}\,b_{i}^{N}\!\left(\frac{t}{T}\right),\quad t\in[0,T]
$$
 
$$
\displaystyle b_{i}^{N}(s)
$$
 
$$
\displaystyle=\binom{N}{i}s^{i}(1-s)^{N-i},\quad s\in[0,1].
$$

### \-E Demonstration Timing Selection

Given a full demonstration of the flying knot, we select a time window from $t_{0}$ to $t_{f}$ to limit demonstration motion to between the start of hand motion and the end of the follow-through motion. We select these times given a manually annotated rope collision time $t_{c}$ with the following search procedure:

1. Select a temporary range $\tilde{t}_{0}$ and $\tilde{t}_{f}$ that excludes noisy data from the human picking up and placing the rope on the floor.
2. Search for maximum hand velocity in the range $[\tilde{t}_{0},\tilde{t}_{f}]$ and label as $t_{\text{peak}}$
3. Step backwards in time from $t_{\text{peak}}$ and update $\tilde{t}_{0}$ to the first point when the hand velocity is near-zero (3% of velocity at $t_{\text{peak}}$).
4. Set $t_{f}=t_{c}+35\text{ms}$ as a fixed follow-through time length.
5. Integrate along the hand path motion between $\tilde{t}_{0}$ and $t_{f}$, then set $t_{0}$ to the time when 5% of the total path length is traveled.

$h(t)$ is then defined as 3D pose trajectory of the hand between $t_{0}$ and $t_{f}$. $\mathbf{x}^{\text{demo}}(t)$ is the rope trajectory from $t_{0}$ to $t_{c}$. Each demonstration type has a different execution speed and overall time length.

TABLE V: Demonstration Tracking Parameters

| Parameter | Value |
| --- | --- |
| $\mathbf{q_{\min}}$ | $[\;-6.28,-1.8,-6.28,-0.19,-6.28,-1.69,-6.28\;]$ |
| $\mathbf{q_{\max}}$ | $[\;6.28,1.9,6.28,3.92,6.28,3.14,6.28\;]$ |
| $\mathbf{\dot{q}_{\min}}$ | $-[3.14,3.14,3.14,3.14,3.14,3.14,3.14]$ |
| $\mathbf{\dot{q}_{\max}}$ | $[3.14,3.14,3.14,3.14,3.14,3.14,3.14]$ |
| $\mathbf{\ddot{q}_{\min}}$ | $-[100,100,100,100,100,100,100]$ |
| $\mathbf{\ddot{q}_{\max}}$ | $[100,100,100,100,100,100,100]$ |
| $w_{j}$ | $5.0\times 10^{-7}$ |
| $w_{p}$ | $\;10\;$ |
| $w_{R}$ | $\;0.2\;$ |
| $w_{v}$ | $\;0.5\;$ |
| $z_{\min}$ | $\;1.2\;$ |

### \-F Demonstration Tracking

The first 7 rows of $\mathbf{q}(t)$ correspond to the robot joint trajectory and the remaining 3 rows correspond to the robot base location. $\mathbf{e}_{h}(t)$ is the end-effector tracking error with respect to the desired fingertip trajectory $h(t)$ weighted by the cost matrix $\mathbf{W}_{h}$:

$$
\displaystyle\mathbf{e}_{h}(t):=\begin{bmatrix}\mathbf{p}_{\mathrm{tip}}(t)-\mathbf{p}_{h}(t)\\
\operatorname{Log}\!\bigl(\mathbf{R}_{h}(t)^{\top}\mathbf{R}_{\mathrm{tip}}(t)\bigr)\\
\dot{\mathbf{p}}_{\mathrm{tip}}(t)-\dot{\mathbf{p}}_{h}(t)\end{bmatrix}
$$
 
$$
\displaystyle\mathbf{W}_{h}:=\operatorname{diag}(w_{p}\mathbf{I}_{3},\;w_{R}\mathbf{I}_{3},\;w_{v}\mathbf{I}_{3}),
$$

with $\mathbf{p}_{\mathrm{tip}}(t)$, $\mathbf{R}_{\mathrm{tip}}(t)$, and $\dot{\mathbf{p}}_{\mathrm{tip}}(t)$ given by the robot forward kinematics $\mathcal{K}$. Values for $w_{p},w_{R},w_{v}$ can be found in Table V. The joint dynamics limits in Table V and Table III are the same. The demonstration-tracking objective tracks the hand motion without any critical-point weighting, so the cost terms are applied equally throughout the hand motion.

The $z_{\mathrm{tip}}$ constraint specifies that the initial configuration of the robot at $t_{0}$ must have enough distance for the rope to dangle without hitting the floor.

[^1]: P. Abbeel, M. Quigley, and A. Y. Ng (2006) Using inaccurate models in reinforcement learning. In Proceedings of the 23rd International Conference on Machine Learning, pp. 1–8. External Links: [Link](https://doi.org/10.1145/1143844.1143845) Cited by: §III-A.

[^2]: E.W. Aboaf, C.G. Atkeson, and D.J. Reinkensmeyer (1988) Task-level robot learning. In Proceedings. 1988 IEEE International Conference on Robotics and Automation, Vol., pp. 1309–1310 vol.2. External Links: [Link](https://doi.org/10.1109/ROBOT.1988.12245) Cited by: §I.

[^3]: N. Agarwal, E. Hazan, A. Majumdar, and K. Singh (2021-07-01) A regret minimization approach to iterative learning control. In Proceedings of the 38th International Conference on Machine Learning, pp. 100–109. External Links: [Link](https://proceedings.mlr.press/v139/agarwal21b.html) Cited by: §III-A.

[^4]: C. H. An, C. G. Atkeson, and J. Hollerbach (1988-04-07) Model-based control of a robot manipulator. Artificial Intelligence Series, MIT Press. External Links: [Link](https://mitpress.mit.edu/9780262510578/model-based-control-of-a-robot-manipulator/) Cited by: §III-A, §III-A, §IV-A.

[^5]: S. Arimoto, S. Kawamura, and F. Miyazaki (1984) Bettering operation of robots by learning. Journal of Robotic Systems 1 (2), pp. 123–140. External Links: [Link](https://doi.org/10.1002/rob.4620010203) Cited by: §I, §III-A.

[^6]: C. Atkeson and J. McIntyre (1986-04) Robot trajectory learning through practice. In 1986 IEEE International Conference on Robotics and Automation Proceedings, Vol. 3, pp. 1737–1742. External Links: [Link](https://ieeexplore.ieee.org/document/1087423) Cited by: §III-A, §V-E, §VI-C.

[^7]: D.A. Bristow, M. Tharayil, and A.G. Alleyne (2006) A survey of iterative learning control. IEEE Control Systems Magazine 26 (3), pp. 96–114. External Links: [Link](https://doi.org/10.1109/MCS.2006.1636313) Cited by: §I, §III-A.

[^8]: V. Bruce (2018-06) Ropes & whips: over hand knot. Note: YouTube video, BookofCoolOfficialAccessed: May 11, 2026. \[Online\]. Available: [https://www.youtube.com/watch?v=7gya0XP53tU](https://www.youtube.com/watch?v=7gya0XP53tU) Cited by: Figure 2.

[^9]: J. Brüdigam and Z. Manchester (2021) Linear-time variational integrators in maximal coordinates. In Algorithmic Foundations of Robotics XIV, S. M. LaValle, M. Lin, T. Ojala, D. Shell, and J. Yu (Eds.), Vol. 17, pp. 194–209. Note: Series Title: Springer Proceedings in Advanced Robotics External Links: [Link](http://link.springer.com/10.1007/978-3-030-66723-8_12) Cited by: §-C, §IV-F.

[^10]: L. Y. Chen, H. Huang, E. Novoseller, D. Seita, J. Ichnowski, M. Laskey, R. Cheng, T. Kollar, and K. Goldberg (2022-09-24) Efficiently learning single-arm fling motions to smooth garments. arXiv. External Links: [Link](http://arxiv.org/abs/2206.08921), 2206.08921 \[cs\] Cited by: §III-B.

[^11]: Y. Chen, B. Chu, and C. T. Freeman (2018) Point-to-point iterative learning control with optimal tracking time allocation. IEEE Transactions on Control Systems Technology 26 (5), pp. 1685–1698. External Links: [Link](https://doi.org/10.1109/TCST.2017.2735358) Cited by: §III-A.

[^12]: C. Chi, B. Burchfiel, E. Cousineau, S. Feng, and S. Song (2022-04-22) Iterative residual policy: for goal-conditioned dynamic manipulation of deformable objects. arXiv. External Links: [Link](http://arxiv.org/abs/2203.00663), 2203.00663 \[cs\] Cited by: §III-B, §III-B.

[^13]: R. Chi, D. Wang, F. L. Lewis, Z. Hou, and S. Jin (2015) Adaptive terminal ilc for iteration-varying target points. Asian Journal of Control 17 (3), pp. 952–962. External Links: [Link](https://doi.org/10.1002/asjc.943) Cited by: §III-A.

[^14]: P. E. Gill, W. Murray, and M. A. Saunders (2002) SNOPT: an SQP algorithm for large-scale constrained optimization. SIAM Journal on Optimization 12 (4), pp. 979–1006. External Links: [Link](https://doi.org/10.1137/S1052623499350013), https://doi.org/10.1137/S1052623499350013 Cited by: §IV-I.

[^15]: P. J. Goulart and Y. Chen (2024) Clarabel: an interior-point solver for conic programs with quadratic objectives. External Links: 2405.12762, [Link](https://arxiv.org/abs/2405.12762) Cited by: §IV-G.

[^16]: S. Gurumurthy, J. Z. Kolter, and Z. Manchester (2023-06-06) Deep off-policy iterative learning control. In Proceedings of The 5th Annual Learning for Dynamics and Control Conference, pp. 639–652. External Links: [Link](https://proceedings.mlr.press/v211/gurumurthy23b.html) Cited by: §III-A.

[^17]: H. Ha and S. Song (2021) FlingBot: the unreasonable effectiveness of dynamic manipulation for cloth unfolding. External Links: 2105.03655, [Link](https://arxiv.org/abs/2105.03655) Cited by: §III-B.

[^18]: E. Hannus, T. N. Le, D. Blanco-Mulero, and V. Kyrki (2024) Dynamic manipulation of deformable objects using imitation learning with adaptation to hardware constraints. In 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 12655–12662. External Links: [Link](https://ieeexplore.ieee.org/document/10802478) Cited by: §III-B.

[^19]: Z. Hu, R. Wu, N. Enock, J. Li, R. Kadakia, Z. Erickson, and A. Kumar (2025-09-09) RaC: robot learning for long-horizon tasks by scaling recovery and correction. arXiv. External Links: [Link](http://arxiv.org/abs/2509.07953), 2509.07953 \[cs\] Cited by: §VI-B.

[^20]: Z. Huang, X. Lin, and D. Held (2023) Self-supervised cloth reconstruction via action-conditioned cloth tracking. In 2023 IEEE International Conference on Robotics and Automation (ICRA), Vol., pp. 7111–7118. External Links: [Link](https://doi.org/10.1109/ICRA48891.2023.10160653) Cited by: §III-B.

[^21]: T. Kuc, K. Nam, and J.S. Lee (1991) An iterative learning control of robot manipulators. IEEE Transactions on Robotics and Automation 7 (6), pp. 835–842. External Links: [Link](https://doi.org/10.1109/70.105392) Cited by: §III-A.

[^22]: V. Lim, H. Huang, L. Y. Chen, J. Wang, J. Ichnowski, D. Seita, M. Laskey, and K. Goldberg (2022-05) Real2Sim2Real: self-supervised learning of physical single-step dynamic actions for planar robot casting. In 2022 International Conference on Robotics and Automation (ICRA), pp. 8282–8289. External Links: [Link](https://ieeexplore.ieee.org/document/9811651) Cited by: §III-B.

[^23]: N. Lin, R. Chi, B. Huang, and Z. Hou (2020) Event-triggered nonlinear iterative learning control. IEEE Transactions on Neural Networks and Learning Systems 32 (11), pp. 5118–5128. External Links: [Link](https://ieeexplore.ieee.org/document/9222356) Cited by: §III-A.

[^24]: D. Mink (2021-01) Trick roping: the flying overhand knot. Note: YouTube video, The Rhinestone RoperAccessed: May 11, 2026. \[Online\]. Available: [https://www.youtube.com/watch?v=lwsydbq3SWU](https://www.youtube.com/watch?v=lwsydbq3SWU) Cited by: Figure 2.

[^25]: K. L. Moore, M. Dahleh, and S. Bhattacharyya (1992) Iterative learning control: a survey and new results. Journal of Robotic Systems 9 (5), pp. 563–594. External Links: [Link](https://doi.org/10.1002/rob.4620090502) Cited by: §III-A.

[^26]: M. C. Nah, A. Krotov, M. Russo, D. Sternad, and N. Hogan (2020-11) Dynamic primitives facilitate manipulating a whip. In 2020 8th IEEE RAS/EMBS International Conference for Biomedical Robotics and Biomechatronics (BioRob), Vol., pp. 685–691. External Links: [Link](https://doi.org/10.1109/BioRob49111.2020.9224399) Cited by: §III-B.

[^27]: A. Nair, D. Chen, P. Agrawal, P. Isola, P. Abbeel, J. Malik, and S. Levine (2017) Combining self-supervised learning and imitation for vision-based rope manipulation. In 2017 IEEE International Conference on Robotics and Automation (ICRA), Vol., pp. 2146–2153. External Links: [Link](https://doi.org/10.1109/ICRA.2017.7989247) Cited by: §III-B.

[^28]: J. D. Ratcliffe, P. L. Lewin, E. Rogers, J. J. Hatonen, and D. H. Owens (2006) Norm-optimal iterative learning control applied to gantry robots for automation applications. IEEE Transactions on Robotics 22 (6), pp. 1303–1307. External Links: [Link](https://doi.org/10.1109/TRO.2006.882927) Cited by: §III-A.

[^29]: G. Salhotra, I. A. Liu, M. Dominguez-Kuhne, and G. S. Sukhatme (2022) Learning deformable object manipulation from expert demonstrations. IEEE Robotics and Automation Letters 7 (4), pp. 8775–8782. External Links: [Link](https://doi.org/10.1109/LRA.2022.3187843) Cited by: §III-B.

[^30]: A. P. Schoellig, F. L. Mueller, and R. D’Andrea (2012-08-01) Optimization-based iterative learning for precise quadrocopter trajectory tracking. 33 (1), pp. 103–127. External Links: [Link](https://doi.org/10.1007/s10514-012-9283-2) Cited by: §III-A, §IV-G, §VI-C.

[^31]: A. Schöllig and R. D’Andrea (2009) Optimization-based iterative learning control for trajectory tracking. In 2009 European Control Conference (ECC), Vol., pp. 1505–1510. External Links: [Link](https://doi.org/10.23919/ECC.2009.7074619) Cited by: §IV-G.

[^32]: T. Tang, C. Wang, and M. Tomizuka (2018) A framework for manipulating deformable linear objects by coherent point drift. IEEE Robotics and Automation Letters 3 (4), pp. 3426–3433. External Links: [Link](https://ieeexplore.ieee.org/document/8403315) Cited by: §III-B.

[^33]: R. Tedrake and the Drake Development Team (2019) Drake: model-based design and verification for robotics. External Links: [Link](https://drake.mit.edu/) Cited by: §-C, §IV-G, §IV-I.

[^34]: UFACTORY (2026) xArm 7 Robotic Arm. Note: [https://www.ufactory.us/product/ufactory-xarm-7](https://www.ufactory.us/product/ufactory-xarm-7) Accessed: 2026-05-11 Cited by: §V-A.

[^35]: C.L. van Oosten, O.H. Bosgra, and B.G. Dijkstra (2004) Reducing residual vibrations through iterative learning control, with application to a wafer stage. In Proceedings of the 2004 American Control Conference, Vol. 6, pp. 5150–5155 vol.6. External Links: [Link](https://doi.org/10.23919/ACC.2004.1384669) Cited by: §III-A.

[^36]: A. Vemula, W. Sun, M. Likhachev, and J. A. Bagnell (2022-05-11) On the effectiveness of iterative learning control. In Proceedings of The 4th Annual Learning for Dynamics and Control Conference, pp. 47–58. External Links: [Link](https://proceedings.mlr.press/v168/vemula22a.html) Cited by: §III-A.

[^37]: Vicon Motion Systems Ltd. (2024) V16 Camera Specifications. Note: [https://help.vicon.com/space/Vantage/15041618/V16%2Bcamera%2Bspecifications](https://help.vicon.com/space/Vantage/15041618/V16%2Bcamera%2Bspecifications) Accessed: 2026-05-11 Cited by: §IV-H, §V-A.

[^38]: J. Wang, H. Huang, V. Lim, H. Zhang, J. Ichnowski, D. Seita, Y. Chen, and K. Goldberg (2024-05-28) Self-supervised learning of dynamic planar manipulation of free-end cables. arXiv. External Links: [Link](http://arxiv.org/abs/2405.09581), 2405.09581 \[cs\] Cited by: §III-B.

[^39]: J. Wang and X. Xiong (2024-09) A learning-based control framework for human-like whip targeting. In 2024 10th IEEE RAS/EMBS International Conference for Biomedical Robotics and Biomechatronics (BioRob), pp. 550–555. Note: ISSN: 2155-1782 External Links: [Link](https://ieeexplore.ieee.org/document/10719935/) Cited by: §III-B.

[^40]: X. Xiong, M. C. Nah, A. Krotov, and D. Sternad (2021-09) Online impedance adaptation facilitates manipulating a whip. In 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 9297–9302. External Links: [Link](https://ieeexplore.ieee.org/document/9636663) Cited by: §III-B.

[^41]: Y. Yamakawa, A. Namiki, and M. Ishikawa (2010) Motion planning for dynamic knotting of a flexible rope with a high-speed robot arm. In 2010 IEEE/RSJ International Conference on Intelligent Robots and Systems, Vol., pp. 49–54. External Links: [Link](https://doi.org/10.1109/IROS.2010.5651168) Cited by: §II, §III-B.

[^42]: Y. Yamakawa, A. Namiki, and M. Ishikawa (2011) Motion planning for dynamic folding of a cloth with two high-speed robot hands and two high-speed sliders. In 2011 IEEE International Conference on Robotics and Automation, Vol., pp. 5486–5491. External Links: [Link](https://doi.org/10.1109/ICRA.2011.5979606) Cited by: §III-B.

[^43]: B. Yi, C. M. Kim, J. Kerr, G. Wu, R. Feng, A. Zhang, J. Kulhanek, H. Choi, Y. Ma, M. Tancik, and A. Kanazawa (2025) Viser: imperative, web-based 3d visualization in Python. External Links: 2507.22885, [Link](https://arxiv.org/abs/2507.22885) Cited by: Figure 5.

[^44]: H. Yin, A. Varava, and D. Kragic (2021) Modeling, learning, perception, and control methods for deformable object manipulation. Science Robotics 6 (54), pp. eabd8803. External Links: [Link](https://www.science.org/doi/abs/10.1126/scirobotics.abd8803), https://www.science.org/doi/pdf/10.1126/scirobotics.abd8803 Cited by: §III-B.

[^45]: U. Yoo, A. Hung, J. Francis, J. Oh, and J. Ichnowski (2024) RoPotter: toward robotic pottery and deformable object manipulation with structural priors. External Links: 2408.02184, [Link](https://arxiv.org/abs/2408.02184) Cited by: §III-B.

[^46]: H. Zhang, J. Ichnowski, D. Seita, J. Wang, H. Huang, and K. Goldberg (2024-05-02) Robots of the lost arc: self-supervised learning to dynamically manipulate fixed-endpoint cables. arXiv. External Links: [Link](http://arxiv.org/abs/2011.04840), 2011.04840 \[cs\] Cited by: §III-B, §III-B.

[^47]: C. Zhao, U. Yoo, A. N. Chaudhury, G. Nam, J. Francis, J. Ichnowski, and J. Oh (2026) DYMO-hair: generalizable volumetric dynamics modeling for robot hair manipulation. External Links: 2510.06199, [Link](https://arxiv.org/abs/2510.06199) Cited by: §III-B.