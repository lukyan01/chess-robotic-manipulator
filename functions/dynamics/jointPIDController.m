function [tau, error_state] = jointPIDController(joint_idx, q, q_dot, q_des, q_dot_des, q_ddot_des, dt, params, prev_error_state)
% JOINTPIDCONTROLLER PID controller for a single robot joint
%
% Inputs:
%   joint_idx        - Joint index (1-7)
%   q                - Current joint position (scalar)
%   q_dot            - Current joint velocity (scalar)
%   q_des            - Desired joint position (scalar)
%   q_dot_des        - Desired joint velocity (scalar)
%   q_ddot_des       - Desired joint acceleration (scalar)
%   dt               - Time step for integral term
%   params           - PID parameters structure with fields:
%                       .Kp - Proportional gain
%                       .Ki - Integral gain
%                       .Kd - Derivative gain
%                       .max_integral - Maximum integral error
%                       .joint_type - 'revolute' or 'prismatic'
%   prev_error_state - Previous error state structure with fields:
%                       .e - Position error
%                       .e_dot - Velocity error
%                       .e_int - Integral error
%
% Outputs:
%   tau         - Computed joint torque/force (scalar)
%   error_state - Updated error state structure

% Initialize error state if not provided
if isempty(prev_error_state)
    prev_error_state = struct('e', 0, 'e_dot', 0, 'e_int', 0);
end

% Calculate position and velocity errors
e = q_des - q;
e_dot = q_dot_des - q_dot;

% Update integral error term with anti-windup
e_int = prev_error_state.e_int + e * dt;

% Apply anti-windup for integral term
if abs(e_int) > params.max_integral
    e_int = sign(e_int) * params.max_integral;
end

% Compute PID terms
p_term = params.Kp * e;
i_term = params.Ki * e_int;
d_term = params.Kd * e_dot;

% Compute control action
tau = p_term + i_term + d_term + q_ddot_des;

% Store updated error state
error_state = struct('e', e, 'e_dot', e_dot, 'e_int', e_int);

% Debug info if requested
if isfield(params, 'debug') && params.debug
    fprintf('Joint %d (%s): P=%.3f, I=%.3f, D=%.3f, Total=%.3f\n', ...
        joint_idx, params.joint_type, p_term, i_term, d_term, tau);
end
end