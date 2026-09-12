# File: robot_model.py
# Course: MMM 5162 Modelling and Simulation
# Purpose: Kinematic model of a two-link planar robot
# Inputs: time, cycle time, link lengths, start/end joint angles
# Outputs: joint angles, joint velocities, and end-effector position
# Author: Student
# Notes: All angles are handled internally in radians

import numpy as np


def robot_model(t, T, L1, L2,
                q1_start, q1_end,
                q2_start, q2_end):

    # Convert time into a NumPy array
    t = np.asarray(t, dtype=float)

    # Automatic model checks
    if T <= 0:
        raise ValueError("Cycle time T must be positive.")

    if L1 <= 0 or L2 <= 0:
        raise ValueError("Robot link lengths must be positive.")

    if np.any(t < 0) or np.any(t > T):
        raise ValueError("Time must remain between 0 and T.")

    # Normalized time
    u = t / T

    # Cubic smooth-motion law
    s = 3 * u**2 - 2 * u**3

    # Derivative of the cubic motion law
    ds_dt = (6 * u - 6 * u**2) / T

    # Joint angles
    q1 = q1_start + (q1_end - q1_start) * s
    q2 = q2_start + (q2_end - q2_start) * s

    # Joint angular velocities
    q1_dot = (q1_end - q1_start) * ds_dt
    q2_dot = (q2_end - q2_start) * ds_dt

    # Forward kinematics
    x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
    y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)

    return q1, q2, q1_dot, q2_dot, x, y