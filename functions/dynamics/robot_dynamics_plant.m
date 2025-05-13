function [dxdt] = robot_dynamics_plant(t, x, u, params)
% ROBOT_DYNAMICS_PLANT Dynamics model for the 7-DOF manipulator
%
% This function implements the state-space model of the manipulator for
% simulation in Simulink. It computes the time derivative of the state
% vector based on the current state and control input.
%
% Inputs:
%   t      - Current time (s)
%   x      - State vector [q; qdot] where:
%             q    - Joint positions [theta1, d_vert, d_horiz, theta4, theta5, theta6, theta7]
%             qdot - Joint velocities
%   u      - Control input (joint torques/forces)
%   params - Structure containing robot parameters
%
% Outputs:
%   dxdt   - Time derivative of state vector [qdot; qddot]
%
% Example:
%   dxdt = robot_dynamics_plant(t, [q; qdot], u, params);

% Check that dynamics functions exist and are on the path
if ~exist('robot_inertia', 'file') || ~exist('robot_coriolis', 'file') || ~exist('robot_gravity', 'file')
    error(['Dynamics functions not found. Generate them first using dynamics.m:\n', ...
           '- robot_inertia.m\n', ...
           '- robot_coriolis.m\n', ...
           '- robot_gravity.m']);
end

% Extract robot parameters
g = params.gravity;          % gravity (m/s^2)
joint_limits = params.joint_limits;  % Joint limits
tau_limits = params.torque_limits;   % Torque/force limits

% Extract state variables
n = 7;  % Number of joints
q = x(1:n);      % Joint positions
qdot = x(n+1:end); % Joint velocities

% Apply joint limits (position)
for i = 1:n
    if q(i) < joint_limits(i, 1)
        q(i) = joint_limits(i, 1);
        if qdot(i) < 0
            qdot(i) = 0;  % Stop at lower limit
        end
    elseif q(i) > joint_limits(i, 2)
        q(i) = joint_limits(i, 2);
        if qdot(i) > 0
            qdot(i) = 0;  % Stop at upper limit
        end
    end
end

% Apply torque/force limits
for i = 1:n
    u(i) = min(max(u(i), -tau_limits(i)), tau_limits(i));
end

% Get dynamic matrices (implemented in generated functions)
% Note: Generated functions expect symbolics, so we need to convert the joint values
q_sym = sym('q', [7, 1]);
qdot_sym = sym('qdot', [7, 1]);

% Create substitution maps for symbolic variables
subs_map_q = struct();
subs_map_qdot = struct();

for i = 1:7
    subs_map_q.(sprintf('q%d', i)) = q(i);
    subs_map_qdot.(sprintf('qdot%d', i)) = qdot(i);
end

% Call the generated dynamics functions with numeric values
M = double(subs(robot_inertia(q_sym), subs_map_q));  % Inertia matrix
C = double(subs(robot_coriolis(q_sym, qdot_sym), [subs_map_q, subs_map_qdot]));  % Coriolis matrix
G = double(subs(robot_gravity(q_sym), subs_map_q));  % Gravity vector

% Compute acceleration (Forward Dynamics)
% M(q) * qddot + C(q, qdot) * qdot + G(q) = tau
qddot = M \ (u - C*qdot - G);

% Return state derivative
dxdt = [qdot; qddot];
end