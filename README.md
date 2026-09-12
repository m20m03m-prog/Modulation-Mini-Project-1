# MMM 5162 - Mini Project 1

## Two-Link Robot Pick-Cycle Kinematic Simulation

**Course:** MMM 5162 - Modelling and Simulation  
**Track:** Mechatronics & Robotics (MR)  
**Student-specific number:** S = 18  

---

## Project Overview

This project implements a kinematic simulation of a two-link planar robot performing one smooth pick-and-place motion.

The supplied cubic motion law is used to generate continuous joint trajectories, and the end-effector position is calculated using forward kinematics.

The project also performs a parameter study to investigate how changing the robot cycle time affects the required joint angular velocities and end-effector speed.

The three simulated cycle-time cases are:

- 0.8T
- T
- 1.2T

where the nominal cycle time is:

```text
T = 3.12 s
