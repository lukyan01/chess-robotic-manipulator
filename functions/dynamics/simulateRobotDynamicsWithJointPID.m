function [q_actual, q_dot_actual, q_ddot_actual, tau_history] = simulateRobotDynamicsWithJointPID(q_traj, q_dot_traj, q_ddot_traj, time_vec, pid_params, model_params, disturbance)
% SIMULATEROBOTDYNAMICSWITHJOINTPID Simulates robot dynamics with individual joint PID controllers
%
% Inputs:
%   q_traj      - Desired joint position trajectory [7 x n_steps]
%   q_dot_traj  - Desired joint velocity trajectory [7 x n_steps]
%   q_ddot_traj - Desired joint acceleration trajectory [7 x n_steps]
%   time_vec    - Time vector [1 x n_steps]
%   pid_params  - Cell array of PID parameters for each joint
%   model_params - Structure with robot model parameters
%   disturbance - Structure with disturbance parameters (optional)
%                 .enabled: true/false
%                 .time: when to apply disturbance
%                 .joint: which joint to disturb
%                 .magnitude: size of disturbance
%
% Outputs:
%   q_actual      - Actual joint position trajectory [7 x n_steps]
%   q_dot_actual  - Actual joint velocity trajectory [7 x n_steps]
%   q_ddot_actual - Actual joint acceleration trajectory [7 x n_steps]
%   tau_history   - Control torque history [7 x n_steps]

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

% Time step
n_steps = length(time_vec);
dt = mean(diff(time_vec));

% Number of joints
num_joints = 7;

% Initialize state vectors
q_actual = zeros(num_joints, n_steps);
q_dot_actual = zeros(num_joints, n_steps);
q_ddot_actual = zeros(num_joints, n_steps);
tau_history = zeros(num_joints, n_steps);

% Set initial conditions to match trajectory start
q_actual(:,1) = q_traj(:,1);
q_dot_actual(:,1) = q_dot_traj(:,1);

% Initialize error states for each joint
error_states = cell(num_joints, 1);
for i = 1:num_joints
    error_states{i} = struct('e', 0, 'e_dot', 0, 'e_int', 0);
end

% Damping coefficients for each joint
if isfield(model_params, 'damping_factor')
    damping_factor = model_params.damping_factor;
else
    damping_factor = 1.0;
end
B = diag([0.25, 0.5, 0.5, 0.1, 0.1, 0.1, 0.05]) * damping_factor;

% Simulation loop
for i = 1:n_steps-1
    % Current state
    q = q_actual(:,i);
    q_dot = q_dot_actual(:,i);
    
    % Target state from trajectory
    q_des = q_traj(:,i);
    q_dot_des = q_dot_traj(:,i);
    q_ddot_des = q_ddot_traj(:,i);
    
    % Compute control action using PID with inverse dynamics
    [tau, error_states] = multiJointPIDController(q, q_dot, q_des, q_dot_des, q_ddot_des, ...
                                                  dt, pid_params, error_states, model_params);
    
    % Store control action
    tau_history(:,i) = tau;
    
    % Apply disturbance if enabled
    if exist('disturbance', 'var') && ~isempty(disturbance) && isstruct(disturbance) && ...
       isfield(disturbance, 'enabled') && disturbance.enabled
        if i * dt >= disturbance.time && i * dt < (disturbance.time + 0.1)
            % Apply disturbance to specified joint
            tau(disturbance.joint) = tau(disturbance.joint) + disturbance.magnitude;
        end
    end
    
    % Get inverse dynamics components for simulation
    M = robot_inertia(q, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
    C = robot_coriolis(q, q_dot, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
    G = robot_gravity(q, g, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
    
    % Add damping/friction
    friction = B * q_dot;
    
    % Compute acceleration using forward dynamics
    q_ddot = M \ (tau - C * q_dot - G - friction);
    
    % Store acceleration
    q_ddot_actual(:,i) = q_ddot;
    
    % Numerical integration using semi-implicit Euler method
    q_dot_actual(:,i+1) = q_dot + q_ddot * dt;
    q_actual(:,i+1) = q + q_dot_actual(:,i+1) * dt;
    
    % Apply joint limits
    joint_limits = [   
        -pi/4,  pi/4;     % theta1  - Base rotation           (-45°  to 45°)
        0.10,  0.25;      % d_vert  - Vertical prismatic      (10cm  to 25cm)
        0.16,  0.48;      % d_horiz - Horizontal prismatic    (15cm  to 65cm)
        pi/18,  4*pi/9;   % theta4  - Joint 4 rotation        ( 10°  to 80°)
        -8*pi/9,  -pi/9;  % theta5  - Joint 5 rotation        (-160° to -20°)
        -4*pi/9,  -pi/18; % theta6  - Joint 6 rotation        (-80°  to -10°)
        -pi/4,  pi/4      % theta7  - Joint 7 rotation        (-45°  to 45°)
    ];
    
    for j = 1:num_joints
        if q_actual(j,i+1) < joint_limits(j,1)
            q_actual(j,i+1) = joint_limits(j,1);
            q_dot_actual(j,i+1) = 0; % Stop at joint limit
        elseif q_actual(j,i+1) > joint_limits(j,2)
            q_actual(j,i+1) = joint_limits(j,2);
            q_dot_actual(j,i+1) = 0; % Stop at joint limit
        end
    end
end

% Calculate final step acceleration
q = q_actual(:,end);
q_dot = q_dot_actual(:,end);
tau = tau_history(:,end-1); % Use last control action
M = robot_inertia(q, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
C = robot_coriolis(q, q_dot, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
G = robot_gravity(q, g, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
friction = B * q_dot;
q_ddot_actual(:,end) = M \ (tau - C * q_dot - G - friction);
end