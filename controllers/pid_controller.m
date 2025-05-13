function [tau, e_int] = pid_controller(q_d, qd_d, qdd_d, q, qd, e_int, dt, params)
% PID_CONTROLLER Implements a PID controller with feedforward compensation
%
% Implements the control law:
% τ = Kp*(qᵈ-q) + Ki*∫(qᵈ-q)dt + Kd*(q̇ᵈ-q̇) + M(q)*q̈ᵈ + C(q,q̇)*q̇ + G(q)
%
% Inputs:
%   q_d    - Desired joint positions
%   qd_d   - Desired joint velocities
%   qdd_d  - Desired joint accelerations
%   q      - Current joint positions
%   qd     - Current joint velocities
%   e_int  - Error integral (state variable)
%   dt     - Time step
%   params - Structure containing controller parameters
%
% Outputs:
%   tau    - Control torques/forces
%   e_int  - Updated error integral
%
% Example:
%   [tau, e_int] = pid_controller(q_d, qd_d, qdd_d, q, qd, e_int, dt, params);

% Extract controller gains
Kp = params.Kp;  % Proportional gain matrix
Ki = params.Ki;  % Integral gain matrix
Kd = params.Kd;  % Derivative gain matrix
windup_limit = params.windup_limit;  % Anti-windup limit

% PID with feedforward control law implementation
% τ = Kp*(qᵈ-q) + Ki*∫(qᵈ-q)dt + Kd*(q̇ᵈ-q̇) + M(q)*q̈ᵈ + C(q,q̇)*q̇ + G(q)

% Calculate errors
e_pos = q_d - q;    % Position error
e_vel = qd_d - qd;  % Velocity error

% Update error integral with anti-windup protection
e_int = e_int + e_pos * dt;

% Apply anti-windup limits to integral term
for i = 1:length(e_int)
    if abs(e_int(i)) > windup_limit
        e_int(i) = sign(e_int(i)) * windup_limit;
    end
end

% Check that dynamics functions exist and are on the path
if ~exist('robot_inertia', 'file') || ~exist('robot_coriolis', 'file') || ~exist('robot_gravity', 'file')
    error(['Dynamics functions not found. Generate them first using dynamics.m:\n', ...
           '- robot_inertia.m\n', ...
           '- robot_coriolis.m\n', ...
           '- robot_gravity.m']);
end

% Get dynamic matrices for feedforward terms
% Note: Generated functions expect symbolics, so we need to convert the joint values
q_sym = sym('q', [7, 1]);
qd_sym = sym('qd', [7, 1]);

% Create substitution maps for symbolic variables
subs_map_q = struct();
subs_map_qd = struct();

for i = 1:7
    subs_map_q.(sprintf('q%d', i)) = q(i);
    subs_map_qd.(sprintf('qd%d', i)) = qd(i);
end

% Call the generated dynamics functions with numeric values
M = double(subs(robot_inertia(q_sym), subs_map_q));      % Inertia matrix
C = double(subs(robot_coriolis(q_sym, qd_sym), [subs_map_q, subs_map_qd]));  % Coriolis matrix
G = double(subs(robot_gravity(q_sym), subs_map_q));      % Gravity vector

% Calculate control torques/forces
tau_p = Kp * e_pos;  % Proportional term
tau_i = Ki * e_int;  % Integral term
tau_d = Kd * e_vel;  % Derivative term

% Feedforward terms
tau_ff_inertia = M * qdd_d;           % Inertia compensation
tau_ff_coriolis = C * qd;             % Coriolis and centrifugal compensation
tau_ff_gravity = G;                   % Gravity compensation

% Combine all terms
tau = tau_p + tau_i + tau_d + tau_ff_inertia + tau_ff_coriolis + tau_ff_gravity;

% Apply torque/force limits
tau_limits = params.torque_limits;
for i = 1:length(tau)
    tau(i) = min(max(tau(i), -tau_limits(i)), tau_limits(i));
end
end