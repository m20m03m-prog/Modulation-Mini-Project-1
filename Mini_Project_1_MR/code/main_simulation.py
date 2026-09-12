# File: main_simulation.py
# Course: MMM 5162 Modelling and Simulation
# Project: Mini Project 1 - Track MR
# Purpose: Simulate a two-link robot pick cycle and perform a cycle-time study
# Author: Student
# Notes: Student-specific number S = 18

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

from robot_model import robot_model


# -------------------------------------------------
# 1. PROJECT FOLDERS
# -------------------------------------------------

project_root = Path(__file__).resolve().parents[1]

figures_folder = project_root / "results" / "figures"
tables_folder = project_root / "results" / "tables"

figures_folder.mkdir(parents=True, exist_ok=True)
tables_folder.mkdir(parents=True, exist_ok=True)


# -------------------------------------------------
# 2. STUDENT-SPECIFIC PARAMETERS
# -------------------------------------------------

S = 18

L1 = 0.380 + 0.005 * S
L2 = 0.300 + 0.004 * S

q1_start_deg = 20 + S
q1_end_deg = 75 - 0.5 * S

q2_start_deg = -55 + 0.8 * S
q2_end_deg = 15 + 0.5 * S

T_nominal = 2.40 + 0.04 * S


# Convert joint angles from degrees to radians
q1_start = np.deg2rad(q1_start_deg)
q1_end = np.deg2rad(q1_end_deg)

q2_start = np.deg2rad(q2_start_deg)
q2_end = np.deg2rad(q2_end_deg)


# -------------------------------------------------
# 3. PARAMETER STUDY
# -------------------------------------------------

cycle_factors = [0.8, 1.0, 1.2]

results = []
simulation_data = {}


for factor in cycle_factors:

    T = factor * T_nominal

    # Use 1001 equally spaced simulation points
    t = np.linspace(0, T, 1001)

    # Run the robot model
    q1, q2, q1_dot, q2_dot, x, y = robot_model(
        t,
        T,
        L1,
        L2,
        q1_start,
        q1_end,
        q2_start,
        q2_end
    )

    # ---------------------------------------------
    # 4. PERFORMANCE MEASURES
    # ---------------------------------------------

    max_q1_speed = np.max(np.abs(q1_dot))
    max_q2_speed = np.max(np.abs(q2_dot))

    dx = np.diff(x)
    dy = np.diff(y)

    path_length = np.sum(
        np.sqrt(dx**2 + dy**2)
    )

    # Numerical end-effector velocity
    x_dot = np.gradient(x, t)
    y_dot = np.gradient(y, t)

    end_effector_speed = np.sqrt(
        x_dot**2 + y_dot**2
    )

    max_end_effector_speed = np.max(
        end_effector_speed
    )

    # ---------------------------------------------
    # 5. AUTOMATIC VERIFICATION CHECKS
    # ---------------------------------------------

    assert np.isclose(q1[0], q1_start)
    assert np.isclose(q1[-1], q1_end)

    assert np.isclose(q2[0], q2_start)
    assert np.isclose(q2[-1], q2_end)

    assert np.isclose(q1_dot[0], 0.0)
    assert np.isclose(q1_dot[-1], 0.0)

    assert np.isclose(q2_dot[0], 0.0)
    assert np.isclose(q2_dot[-1], 0.0)

    # ---------------------------------------------
    # 6. SAVE RESULTS FOR THIS CASE
    # ---------------------------------------------

    label = f"{factor:.1f}T"

    simulation_data[label] = {
        "t": t,
        "q1": q1,
        "q2": q2,
        "q1_dot": q1_dot,
        "q2_dot": q2_dot,
        "x": x,
        "y": y
    }

    results.append({
        "Case": label,
        "Cycle Time (s)": T,
        "Max |q1_dot| (rad/s)": max_q1_speed,
        "Max |q2_dot| (rad/s)": max_q2_speed,
        "Path Length (m)": path_length,
        "Maximum End-Effector Speed (m/s)": max_end_effector_speed,
        "Start x (m)": x[0],
        "Start y (m)": y[0],
        "End x (m)": x[-1],
        "End y (m)": y[-1]
    })


# -------------------------------------------------
# 7. CREATE RESULTS TABLE
# -------------------------------------------------

results_df = pd.DataFrame(results)

results_file = tables_folder / "parameter_study_results.csv"
results_df.to_csv(results_file, index=False)


# -------------------------------------------------
# 8. BASELINE END-EFFECTOR PATH
# -------------------------------------------------

baseline = simulation_data["1.0T"]

plt.figure(figsize=(7, 6))

plt.plot(
    baseline["x"],
    baseline["y"],
    linewidth=2
)

plt.scatter(
    baseline["x"][0],
    baseline["y"][0],
    label="Start"
)

plt.scatter(
    baseline["x"][-1],
    baseline["y"][-1],
    label="End"
)

plt.xlabel("x Position (m)")
plt.ylabel("y Position (m)")
plt.title("Baseline End-Effector Path")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.tight_layout()

plt.savefig(
    figures_folder / "baseline_end_effector_path.png",
    dpi=300
)

plt.close()


# -------------------------------------------------
# 9. BASELINE JOINT ANGLES
# -------------------------------------------------

plt.figure(figsize=(7, 5))

plt.plot(
    baseline["t"],
    np.rad2deg(baseline["q1"]),
    label="Joint 1"
)

plt.plot(
    baseline["t"],
    np.rad2deg(baseline["q2"]),
    label="Joint 2"
)

plt.xlabel("Time (s)")
plt.ylabel("Joint Angle (deg)")
plt.title("Baseline Joint Angles")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig(
    figures_folder / "baseline_joint_angles.png",
    dpi=300
)

plt.close()


# -------------------------------------------------
# 10. BASELINE JOINT VELOCITIES
# -------------------------------------------------

plt.figure(figsize=(7, 5))

plt.plot(
    baseline["t"],
    baseline["q1_dot"],
    label="Joint 1"
)

plt.plot(
    baseline["t"],
    baseline["q2_dot"],
    label="Joint 2"
)

plt.xlabel("Time (s)")
plt.ylabel("Joint Angular Velocity (rad/s)")
plt.title("Baseline Joint Angular Velocities")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig(
    figures_folder / "baseline_joint_velocities.png",
    dpi=300
)

plt.close()


# -------------------------------------------------
# 11. PARAMETER-STUDY COMPARISON PLOT
# -------------------------------------------------

cases = results_df["Case"]

plt.figure(figsize=(7, 5))

plt.plot(
    cases,
    results_df["Max |q1_dot| (rad/s)"],
    marker="o",
    label="Max |q1 dot|"
)

plt.plot(
    cases,
    results_df["Max |q2_dot| (rad/s)"],
    marker="o",
    label="Max |q2 dot|"
)

plt.xlabel("Cycle-Time Case")
plt.ylabel("Maximum Joint Speed (rad/s)")
plt.title("Effect of Cycle Time on Maximum Joint Speed")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig(
    figures_folder / "cycle_time_speed_comparison.png",
    dpi=300
)

plt.close()


# -------------------------------------------------
# 12. PRINT SUMMARY
# -------------------------------------------------

print("\nMINI PROJECT 1 - MR TRACK")
print("-------------------------")

print(f"Student number S = {S}")
print(f"L1 = {L1:.3f} m")
print(f"L2 = {L2:.3f} m")
print(f"q1 start = {q1_start_deg:.1f} deg")
print(f"q1 end   = {q1_end_deg:.1f} deg")
print(f"q2 start = {q2_start_deg:.1f} deg")
print(f"q2 end   = {q2_end_deg:.1f} deg")
print(f"Nominal cycle time = {T_nominal:.3f} s")

print("\nParameter Study Results:")
print(results_df.to_string(index=False))

print("\nAutomatic verification checks passed.")
print(f"\nResults saved to:\n{project_root / 'results'}")