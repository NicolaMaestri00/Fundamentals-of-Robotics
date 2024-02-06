# Introduction to Robotics
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

## Final Assignment
Handing a microphone to random people that want to ask questions after a talk can be a duly task, 
and we want to automatize it. The assignment for the final project is to design a giraffe robot that 
is able to place a microphone in front of a person in a small theatre/conference room. 
The robot is located in the middle of the room and attached to the ceiling. The room is 4 m high 
and the robot should be able to reach 1 m high locations in a 5x12 square meters area.
The robot should have 5 degrees of freedom: a spherical joint at the base (2 revolute joints with 
intersecting axes), one prismatic joint that can achieve a long extension and 2 revolute joints 
to properly orient the microphone (not necessarily with intersecting axes). 
We want to be able to locate the microphone at any point in the 5x5 conference room, with a certain
pitch orientation (30 deg) with respect to the horizontal (the task is 4D), to allow people to talk 
comfortably in the microphone. 

The project can be approached through the following incremental steps:
1. Construct the URDF model of the robot, selecting appropriate link lengths and
arranging frames suitably.
2. Compute the forward kinematics (position/orientation) and differential kinematics
(Jacobian) of the end-effector.
3. Use Pinocchio library’s RNEA native function to create a simulator of the motion.
4. Plan a polynomial trajectory (in the task space) to move from a coming configuration $q_{home}$ to a given end-effector configuration-orientation $p_{des}+ \Theta_{des}$.
5. Write an inverse-dynamics (computed torque) control action in the task space to linearize the system and achieve tracking of the task.
6. Set the PD gains of the Cartesian controller implemented on the linearized system to achieve a settling time of 7s without overshoot.
7. In the null space of the task minimize the distance with respect to a given configuration q0 of your choice.
8. Simulate the robot to reach the location $p_{des} = [1, 2, 1]$ from the homing configuration $q_{home}= [0, 0, 0, 0]$.
![robotica](https://github.com/NicolaMaestri00/Fundamentals-of-Robotics/assets/104208237/9a2242cb-c52f-4931-928e-d110162f5ee9)

## Tools
- Robot Operative System (ROS)
- Pinocchio Library
- Locosim
- Python

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
