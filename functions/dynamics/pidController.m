function [tau, e, e_dot, e_int] = pidController(q, q_dot, q_des, q_dot_des, q_ddot_des, dt, Kp, Kd, Ki, e_int_prev, model_params)
% PIDCONTROLLER Computed torque PID controller with inverse dynamics
%
% Implements a PID controller with inverse dynamics compensation:
% tau = M(q)(q̈_des + Kp*e + Kd*ė + Ki*∫e dt) + C(q,q̇)q̇ + G(q)
%
% Inputs:
%   q          - Current joint positions [7x1]
%   q_dot      - Current joint velocities [7x1]
%   q_des      - Desired joint positions [7x1]
%   q_dot_des  - Desired joint velocities [7x1]
%   q_ddot_des - Desired joint accelerations [7x1]
%   dt         - Time step for integral term
%   Kp         - Proportional gain matrix [7x7]
%   Kd         - Derivative gain matrix [7x7]
%   Ki         - Integral gain matrix [7x7]
%   e_int_prev - Previous integral error [7x1]
%   model_params - Structure with robot model parameters
%
% Outputs:
%   tau     - Computed joint torques/forces [7x1]
%   e       - Position error [7x1]
%   e_dot   - Velocity error [7x1]
%   e_int   - Updated integral error [7x1]

% Extract model parameters
L_45 = model_params.L_45;
L_6 = model_params.L_6;
m1 = model_params.m1;
m2 = model_params.m2;
m3 = model_params.m3;
m4 = model_params.m4;
m5 = model_params.m5;
m6 = model_params.m6;
m7 = model_params.m7;
g = model_params.g;

% Calculate position and velocity errors
e = q_des - q;
e_dot = q_dot_des - q_dot;

% Update integral error term
e_int = e_int_prev + e * dt;

% Anti-windup for integral term
max_integral = 10.0; % Maximum allowed integral error
for i = 1:length(e_int)
    if abs(e_int(i)) > max_integral
        e_int(i) = sign(e_int(i)) * max_integral;
    end
end

% Calculate inverse dynamics components
M = robot_inertia(q, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
C = robot_coriolis(q, q_dot, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
G = robot_gravity(q, g, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);

% Compute feedback terms
feedback = Kp * e + Kd * e_dot + Ki * e_int;

% Compute feedforward term (desired acceleration)
feedforward = q_ddot_des;

% Compute control action using inverse dynamics
tau = M * (feedforward + feedback) + C * q_dot + G;

max_torque = [10.0; 100.0; 100.0; 5.0; 5.0; 5.0; 1.0]; % Example limits
for i = 1:length(tau)
    if abs(tau(i)) > max_torque(i)
        tau(i) = sign(tau(i)) * max_torque(i);
    end
end
end