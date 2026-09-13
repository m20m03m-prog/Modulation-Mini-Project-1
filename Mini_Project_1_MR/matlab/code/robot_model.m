% File: robot_model.m
% Course: MMM 5162 Modelling and Simulation
% Project: Mini Project 1 - Track MR
% Purpose: Kinematic model of a two-link planar robot
% Inputs:
%   t          - time vector (s)
%   T          - cycle time (s)
%   L1, L2     - robot link lengths (m)
%   q1_start   - Joint 1 starting angle (rad)
%   q1_end     - Joint 1 ending angle (rad)
%   q2_start   - Joint 2 starting angle (rad)
%   q2_end     - Joint 2 ending angle (rad)
% Outputs:
%   q1, q2         - joint angles (rad)
%   q1_dot, q2_dot - joint angular velocities (rad/s)
%   x, y           - end-effector coordinates (m)

function [q1, q2, q1_dot, q2_dot, x, y] = robot_model( ...
    t, T, L1, L2, ...
    q1_start, q1_end, ...
    q2_start, q2_end)

    % --------------------------------------------------------
    % 1. AUTOMATIC MODEL CHECKS
    % --------------------------------------------------------

    if T <= 0
        error('Cycle time T must be positive.');
    end

    if L1 <= 0 || L2 <= 0
        error('Robot link lengths must be positive.');
    end

    if any(t < 0) || any(t > T)
        error('Time must remain between 0 and T.');
    end

    % --------------------------------------------------------
    % 2. NORMALIZED TIME
    % --------------------------------------------------------

    u = t ./ T;

    % --------------------------------------------------------
    % 3. CUBIC SMOOTH-MOTION LAW
    % --------------------------------------------------------

    s = 3 .* u.^2 - 2 .* u.^3;

    % Derivative of the cubic motion law
    ds_dt = (6 .* u - 6 .* u.^2) ./ T;

    % --------------------------------------------------------
    % 4. JOINT ANGLES
    % --------------------------------------------------------

    q1 = q1_start + (q1_end - q1_start) .* s;

    q2 = q2_start + (q2_end - q2_start) .* s;

    % --------------------------------------------------------
    % 5. JOINT ANGULAR VELOCITIES
    % --------------------------------------------------------

    q1_dot = (q1_end - q1_start) .* ds_dt;

    q2_dot = (q2_end - q2_start) .* ds_dt;

    % --------------------------------------------------------
    % 6. FORWARD KINEMATICS
    % --------------------------------------------------------

    x = L1 .* cos(q1) + ...
        L2 .* cos(q1 + q2);

    y = L1 .* sin(q1) + ...
        L2 .* sin(q1 + q2);

end