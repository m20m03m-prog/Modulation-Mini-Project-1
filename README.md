# MMM 5162 – Modelling and Simulation

## Mini Project 1: Two-Link Robot Pick-Cycle Kinematic Simulation

**Track:** Mechatronics & Robotics (MR)  
**Student-Specific Number:** S = 18  
**Software:** Python 3  
**Course:** MMM 5162 – Modelling and Simulation  
**Institution:** Islamic University of Madinah – Faculty of Engineering

---

## Project Overview

This repository contains the complete Python implementation for **Mini Project 1 – Computational Simulation and Parameter Study**.

The project models the kinematic motion of a two-link planar robot performing one smooth pick-and-place transfer. A cubic motion law is used to move both joints smoothly from their initial angles to their final angles while maintaining zero joint velocity at the beginning and end of the motion.

The simulation also investigates how changing the cycle time affects the required joint angular velocities and end-effector speed.

---

## Project Objectives

The project performs the following tasks:

- Implements the robot equations as a reusable Python function.
- Simulates the assigned baseline case using `S = 18`.
- Performs a parameter study using cycle times `0.8T`, `T`, and `1.2T`.
- Calculates the end-effector trajectory using forward kinematics.
- Calculates joint angular velocities analytically.
- Extracts quantitative engineering performance measures.
- Performs automatic model and verification checks.
- Generates engineering plots with labels and units.
- Saves numerical results in CSV format.

---

## Assigned Parameters

For `S = 18`, the robot parameters are:

| Parameter | Value |
|---|---:|
| Link 1 length, L1 | 0.470 m |
| Link 2 length, L2 | 0.372 m |
| Joint 1 start angle | 38° |
| Joint 1 end angle | 66° |
| Joint 2 start angle | -40.6° |
| Joint 2 end angle | 24° |
| Nominal cycle time, T | 3.120 s |

The parameter study uses:

| Case | Cycle Time |
|---|---:|
| 0.8T | 2.496 s |
| T | 3.120 s |
| 1.2T | 3.744 s |

---

## Mathematical Model

Normalized time is defined as

```text
u = t / T
```

and the cubic motion law is

```text
s(u) = 3u² - 2u³
```

The joint positions are calculated from

```text
qj(t) = qj,start + (qj,end - qj,start) s(u)
```

The end-effector position is obtained using two-link planar forward kinematics:

```text
x = L1 cos(q1) + L2 cos(q1 + q2)

y = L1 sin(q1) + L2 sin(q1 + q2)
```

All angular calculations are performed internally in radians.

---

## Repository Structure

```text
Mini_Project_1_MR/
│
├── code/
│   ├── robot_model.py
│   └── main_simulation.py
│
├── results/
│   ├── figures/
│   └── tables/
│
├── report/
│
└── test_python.py
```

### Main Files

`robot_model.py`

Contains the reusable mathematical and kinematic model of the two-link robot.

`main_simulation.py`

Runs the baseline simulation and parameter study, calculates engineering performance measures, performs verification checks, generates plots, and saves the numerical results.

---

## Python Requirements

The project requires:

- Python 3
- NumPy
- pandas
- Matplotlib

Install the required packages using:

```bash
pip install numpy pandas matplotlib
```

---

## How to Run the Simulation

Open a terminal and move into the `code` folder:

```bash
cd Mini_Project_1_MR/code
```

Then run:

```bash
python main_simulation.py
```

The simulation automatically creates the required result files.

---

## Generated Outputs

The program generates the following figures:

```text
baseline_end_effector_path.png
baseline_joint_angles.png
baseline_joint_velocities.png
cycle_time_speed_comparison.png
```

It also generates the numerical results table:

```text
parameter_study_results.csv
```

---

## Key Simulation Results

| Case | Cycle Time (s) | Max \|q1_dot\| (rad/s) | Max \|q2_dot\| (rad/s) | Path Length (m) |
|---|---:|---:|---:|---:|
| 0.8T | 2.496 | 0.29369 | 0.67757 | 0.82045 |
| T | 3.120 | 0.23495 | 0.54206 | 0.82045 |
| 1.2T | 3.744 | 0.19579 | 0.45172 | 0.82045 |

For the nominal cycle, the maximum end-effector speed is approximately:

```text
0.39869 m/s
```

The end-effector moves approximately from:

```text
Start: (0.74198, 0.27249) m
```

to:

```text
End:   (0.19117, 0.80137) m
```

---

## Engineering Conclusion

Reducing the cycle time increases the required joint angular velocities and end-effector speed without changing the prescribed geometric path.

Compared with the nominal cycle:

- Using `0.8T` increases the maximum joint speeds by approximately **25%**.
- Using `1.2T` decreases the maximum joint speeds by approximately **16.7%**.
- The end-effector path length remains approximately **0.82045 m** in all three cases.

Therefore, faster execution of the same pick-and-place trajectory requires greater actuator speed capability.

---

## Verification

The program automatically verifies that:

- Robot link lengths are positive.
- Cycle time is positive.
- Simulation time remains within the allowed interval.
- Initial joint angles match the assigned starting conditions.
- Final joint angles match the assigned ending conditions.
- Joint velocities are zero at the beginning of the cycle.
- Joint velocities are zero at the end of the cycle.

These checks help ensure that the simulation is reproducible and consistent with the supplied engineering model.

---

## Reproducibility

All reported numerical results and figures are generated directly by the submitted Python code.

Running:

```bash
python main_simulation.py
```

reproduces the parameter-study results, figures, verification checks, and CSV output used in the technical report.

---

## Responsible Use of AI

Generative AI was used as a supporting tool for code explanation, debugging guidance, and documentation. The mathematical model, simulation execution, numerical outputs, engineering interpretation, and verification of the submitted work remain the student's responsibility.
