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


## Laboratories
### Lab 1
- Learning the basic procedure to visualize a robot model using the Unified Robot Description Format (URDF)
- Compute and visualize the direct/inverse kinematics of a 4-DoF serial manipulator
- Compute and analyze the forward/inverse dynamics of a 4-DoF serial manipulator using the Recursive Newton-Euler Algorithm (RNEA)
### Lab 2-3
- Learning the basic procedure to design a motion controller in the joint space for a manipulator in free-motion (i.e. not in contact)
- Analyze the advantages/disadvantages of decentralized/ centralized approaches (i.e. feedback linearization)
- Implement the interaction with the environment with a compliant contact mode
### Lab 4-5-6
- Acquire confidence in some invariant properties of floating base dynamics.
- Contact consistent (fixed) base dynamics
- Floating base robot: quasi-static control of locomotion stability
### Lab 7
- Learning the basic procedure to design an admittance controller for the end-effector of a manipulator in contact with the environment with the purpose to control the interaction with a human.
- Implement an obstacle avoidance planning algorithm base on potential fields.







## Syllabus
Introduzione
- informazioni generali, contatti, riferimenti.
- Classificazione dei robot. Evoluzione dei robot industriali. Varie tipologie di robot.
- Struttura meccanica di un robot, classificazione dei robot in base all’organizzazione dei giunti. Panoramica sulle unità funzionali.

Sensori
- proprietà dei sistemi di misura, caratteristiche dei sensori, tipi di errori di misura. Non-idealità nei sensori.
- tipi di sensori propriocettivi. Sensori di posizione, sensori inerziali.
- tipi di sensori esterocettivi. Sensori di forza. Sensori di visione. Telecamere passive, stereocamere, stereovisione. Modelli per le telecamere. Sensori attivi: LiDAR, sensori a luce strutturata.
- segnali analogici/digitali. Campionamento, quantizzazione e ricostruzione (conversioni A/D e D/A). Problemi nell’implementazione digitale. Filtri passa-basso.

Attuatori
- attuatori per la robotica: pneumatici, idraulici, elettro-idraulici, motori elettrici, attuatori elastici in serie.
- Motori sincroni e asincroni in corrente alternata. Motori con e senza spazzole in corrente continua. Modelli. Risposta a regime permanente. Controllo dei motori: tensione/corrente.
- Tipi di trasmissioni. Modellazione di trasmissioni. Riduttori. Scelta ottima del rapporto di riduzione. Modellazione dell’elasticità nelle trasmissioni.
- Non-idealità negli attuatori: modelli per attrito, gioco negli ingranaggi, zona morta.
- Simulazione degli attuatori: spazio di stato di un motore in corrente continua, modello discreto equivalente, integrazione della dinamica, tempi di risposta.

Elementi di controlli automatici
- controllo in catena aperta, il concetto di retroazione, controllo bang-bang, risposta transitoria e regime permanente, specifiche di controllo statiche e dinamiche, progetto di un controllore.
- Controllo P, PI e PID, controllo in corrente, tecnica anti-windup.
- problema della realizzazione del PID, implementazione digitale, tecniche di taratura del PID.

Cinematica
- posizione e orientamento di un corpo rigido. Rappresentazioni dell’orientamento.
- definizione della cinematica diretta e inversa. Spazi di giunto, di task e di attuazione. Coordinate generalizzate. Cinematica diretta dei manipolatori. Trasformazioni omogenee. Spazio di lavoro.
- velocità lineare e angolare del link di un manipolatore. Lo Jacobiano geometrico e analitico.
- definizione di cinematica inversa. Spazio di lavoro. Soluzioni in forma chiusa (analitica).
- definizione di ridondanza. Manipolatori ridondanti. Ridondanza e spazio nullo. Definizione e tipi di singolarità. Cinematica differenziale inversa e singolarità. Metodo dei minimi quadrati. Cinematica inversa numerica. Confronto fra diversi algoritmi di cinematica inversa.

Dinamica
- statica vs. dinamica. Principio dei lavori virtuali. Dualità cineto-statica. Trasformazioni di velocità e forza.
- modelli dinamici dei robot. Dinamica diretta e inversa. Modello dinamico Lagrangiano. Accoppiamenti inerziali. Algoritmo ricorsivo di Newton-Eulero
- modelli di contatto rigido e non-rigido. Dinamica vincolata dei robot.
- definizione ed esempi di robot sottoattuati

Controllo nello spazio dei giunti
- panoramica dei problemi di controllo in robotica. Il concetto di stabilità. Controllo PD, PD + compensazione di gravità, PID.
- controllo centralizzato e decentralizzato. Feedback linearization in robotica. Dinamica inversa.

Controllo nello spazio dei task
- controllo a cinematica inversa, dinamica inversa nello spazio Cartesiano. Controllo di manipolatori ridondanti
- controllo dell’orientamento con diverse parametrizzazioni dell’orientamento.
- controllo diretto di forza, controllo Cartesiano di impedenza.






## References
Algebra lineare:
- Introduction to Linear Algebra – Gilbert Strang (book) and course: https://ocw.mit.edu/courses/mathematics/18-06-linear-algebra-spring-2010/

Robotica:
- Robotics Modelling, Planning and Control - Siciliano, B., Sciavicco, L., Villani, L., Oriolo, G.
- Sistemi di Automazione Industriale..Architetture e Controllo, Bonivento, Gentili, Paoli, 2010.
- Robotics 1, A. De Luca: http://www.diag.uniroma1.it/~deluca/rob1_en.php
- Robotics 2, A. De Luca: http://www.diag.uniroma1.it/deluca/rob2_en.php
- O. Kathib: Introduction to Robotics: http://videolectures.net/stanfordcs223aw08_introduction_robotics/
- Introduction to Robotics: Mechanics and Control – J.Craig

Sistemi lineari:
- Feedback Control of Dynamic Systems, Gene F. Franklin, J. David Powell, Abbas Emami-Naeini
-Franklin, Powell, Workman - Digital Control of Dynamic Systems
- EE263 - Introduction to Linear Dynamical Systems, S. Boyd: https://see.stanford.edu/Course/EE263
- MIT 2.04A Systems and Controls, George Barbastathis: https://ocw.mit.edu/courses/mechanical-engineering/2-04a-systems-and-controls-spring-2013/
