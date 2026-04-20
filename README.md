# ME 449 — Robotic Manipulation
**Northwestern University · McCormick School of Engineering**

Graduate-level robotics course covering the mathematical foundations of rigid-body motion, forward/inverse kinematics, trajectory planning, and feedback control for robot manipulators.

---

## Final Project — Mobile Manipulation of a KUKA youBot

### Overview
Programmed a KUKA youBot mobile manipulator to autonomously pick up a block from a known start location and place it at a target location — entirely from scratch in Python. The solution integrates trajectory generation, full-body kinematics, odometry-based localization, and closed-loop feedback control into a single end-to-end pipeline.

### Pipeline
```
1. Reference Trajectory Generation
   └─ 8-segment SE(3) screw trajectory (approach → grasp → carry → place)

2. youBot Kinematics
   ├─ Forward kinematics via product of exponentials (PoE)
   └─ Jacobian computation (body + chassis combined)

3. Feedback Control
   └─ PI controller in SE(3) — twist error → wheel + joint velocities

4. Odometry
   └─ Chassis state integration from mecanum wheel velocities

5. Simulation
   └─ CoppeliaSim (V-REP) validation
```

### Key Concepts
- **Modern Robotics** formalism (Lynch & Park) — screw theory, se(3), adjoint maps
- **Pseudoinverse Jacobian** control with joint limit handling
- **Feedforward + feedback** control law: `V = [Ad] * Vd + Kp*Xerr + Ki*∫Xerr`
- **Mecanum wheel** kinematics for omnidirectional chassis control

### Results
Successfully demonstrated pick-and-place with smooth trajectories and stable grasp. Controller converges from initial configuration error within the first few timesteps.

---

## Stack
`Python` `NumPy` `CoppeliaSim` `Modern Robotics Library`

---

*Northwestern University — ME 449, Fall 2024 · Robotic Manipulation*
