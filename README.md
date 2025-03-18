# Introduction to Robotics

## Final Assignment: *Automated Microphone Delivery with a Giraffe Robot*

In this project, we design a giraffe robot to automate microphone delivery in a small conference room. Mounted at the center of a $4$‑meter-high ceiling, the robot covers a $5 \times 12$‑meter area and targets a $5 \times 5$‑meter region with a fixed $30^\circ$ pitch for comfortable speaking.

Key design features include $5$ degrees of freedom:
- *Spherical base joint* (2 intersecting revolute joints)
- *Prismatic joint* for extended reach
- 2 *revolute joints* for precise microphone orientation

![robotica](https://github.com/NicolaMaestri00/Fundamentals-of-Robotics/assets/104208237/9a2242cb-c52f-4931-928e-d110162f5ee9)

The project is approached incrementally:
1. **URDF Modeling:** Construct the URDF model by selecting appropriate link lengths and arranging frames.
2. **Kinematics:** Compute forward kinematics (position/orientation) and the differential kinematics (Jacobian) of the end-effector.
3. **Simulation:** Use the Pinocchio library’s RNEA function to simulate the robot's motion.
4. **Trajectory Planning:** Design a polynomial trajectory in task space to transition from the home configuration $$q_{home}$$ to the desired end-effector state $$p_{des} + \Theta_{des}$$.
5. **Control Strategy:** Implement an inverse-dynamics (computed torque) control to linearize the system and achieve precise tracking.
6. **Controller Tuning:** Set the Cartesian PD gains to ensure a $7$‑second settling time without overshoot.
7. **Null Space Optimization:** Minimize deviation in the null space relative to a chosen configuration $$q_0$$.
8. **Simulation Execution:** Demonstrate the robot reaching $$p_{des} = [1, 2, 1]$$ from $$q_{home} = [0, 0, 0, 0]$$.

**Tools**: *Robot Operative System* (ROS), *Pinocchio* Library, *Locosim*, *Python*

## Course Syllabus  

- **Introduction to Robotics** (Robot classification, industrial evolution, mechanical structure, joint organization)  
- **Sensors** (Measurement principles, sensor characteristics, proprioceptive sensors, exteroceptive sensors, signal processing)  
- **Actuators** (Pneumatic, hydraulic, electric motors, series elastic actuators, motor technologies, transmissions, non-idealities, simulation)  
- **Automatic Control Fundamentals** (Open-loop and feedback control, PID controllers, digital implementation challenges)  
- **Kinematics** (Rigid body motion, forward and inverse kinematics, velocity analysis, inverse kinematics techniques)  
- **Dynamics** (Statics vs. dynamics, Lagrangian formulation, dynamic models, contact dynamics)  
- **Joint-Space Control** (Control challenges, PD and PID control, centralized vs. decentralized strategies, feedback linearization)  
- **Task-Space Control** (Inverse kinematics/dynamics, redundant manipulator control, interaction control, force and impedance control)  

## Laboratories

### Lab 1: Robot Visualization & Kinematics/Dynamics
- Compute and display forward/inverse kinematics of a 4-DoF manipulator.
- Analyze its dynamics using the Recursive Newton-Euler Algorithm (RNEA).

### Labs 2-3: Joint Space Motion Control & Environmental Interaction
- Compare decentralized vs. centralized (feedback linearization) control approaches.
- Implement compliant control for interacting with the environment.

### Labs 4-5-6: Floating Base Dynamics & Stability
- Understand invariant properties and fixed-base dynamics.
- Apply quasi-static control methods to ensure locomotion stability in floating base robots.

### Lab 7: Admittance Control & Obstacle Avoidance
- Develop an admittance controller for the end-effector during contact tasks.
- Implement an obstacle avoidance algorithm using potential fields.

## References
- Robotics Modelling, Planning and Control - Siciliano, B., Sciavicco, L., Villani, L., Oriolo, G.
- O. Kathib: Introduction to Robotics: http://videolectures.net/stanfordcs223aw08_introduction_robotics/
- MIT 2.04A Systems and Controls, George Barbastathis: https://ocw.mit.edu/courses/mechanical-engineering/2-04a-systems-and-controls-spring-2013/
