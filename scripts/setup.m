%% Chess Robot Simulator - Setup Script
% Initializes all parameters and paths for the robot manipulator simulation

%% Add paths to project folders
addpath(genpath('functions'));
addpath(genpath('controllers'));
addpath(genpath('models'));
addpath(genpath('data'));

%% Robot physical parameters
% Link dimensions
params.L_45 = 0.121851;    % Length of links 1 & 2 (identical) in meters
params.L_6 = 0.3;          % Length of link 3 in meters
params.link_lengths = [params.L_45; params.L_6];

% Masses
params.m_1 = 0.5;          % Base joint (kg)
params.m_2 = 0.8;          % Vertical prismatic joint (kg) 
params.m_3 = 0.8;          % Horizontal prismatic joint (kg)
params.m_4 = 0.5;          % First revolute joint (kg)
params.m_5 = 0.5;          % Second revolute joint (kg)
params.m_6 = 0.5;          % Third revolute joint (kg)
params.m_7 = 0.3;          % End-effector joint (kg)
params.masses = [params.m_1, params.m_2, params.m_3, params.m_4, params.m_5, params.m_6, params.m_7];

% Joint limits
joint_limits = [   
      -pi/4,  pi/4;     % theta1  - Base rotation           (-45°  to 45°)
       0.16,  0.48;     % d_horiz - Horizontal prismatic    (15cm  to 65cm)
       0.10,  0.25;     % d_vert  - Vertical prismatic      (10cm  to 25cm)
      pi/18,  4*pi/9;   % theta4  - Joint 4 rotation        ( 10°  to 80°)
    -8*pi/9,  -pi/9;    % theta5  - Joint 5 rotation        (-160° to -20°)
    -4*pi/9,  -pi/18;   % theta6  - Joint 6 rotation        (-80°  to -10°)
      -pi/4,  pi/4      % theta7  - Joint 7 rotation        (-45°  to 45°)
]; 

% Torque/force limits
params.torque_limits = [10, 100, 100, 5, 5, 5, 2];  % N·m for revolute, N for prismatic

% Chess parameters
params.square_size = 0.06;          % 6 cm
params.chess_piece_height = 0.095;  % Height of tallest piece (king) in meters
params.lift_height = 0.08;          % Additional height to lift piece above board (m)

%% Controller parameters
% PID parameters (initial values, can be tuned)
params.Kp = diag([5, 50, 50, 5, 5, 5, 2]);  % Proportional gains
params.Ki = diag([1, 10, 10, 1, 1, 1, 0.5]); % Integral gains
params.Kd = diag([2, 20, 20, 2, 2, 2, 1]);  % Derivative gains

% Anti-windup parameters
params.windup_limit = 10;  % Limit for integral term

%% Simulation parameters
params.sample_time = 0.001;           % 1 kHz control rate
params.simulation_time = 5;           % Default simulation time
params.gravity = 9.81;                % Gravity acceleration (m/s²)

% Initialize state vector
% [theta1, d_vert, d_horiz, theta4, theta5, theta6, theta7,
%  theta1_dot, d_vert_dot, d_horiz_dot, theta4_dot, theta5_dot, theta6_dot, theta7_dot]
params.initial_state = zeros(14, 1);
params.initial_state(1:7) = [0, 0.1, 0.3, 0, 0, 0, 0];  % Home position

% Disturbance parameters
params.enable_disturbance = false;    % Enable/disable disturbances
params.disturbance_time = 2.5;        % Time to apply disturbance (seconds)
params.disturbance_magnitude = [0, 0, 0, 0, 0, 0, 0]';  % Nm or N

% Logging parameters
params.enable_logging = true;         % Enable data logging
params.log_directory = '../data/simulation_results/';

% Create the log directory if it doesn't exist
if ~exist(params.log_directory, 'dir')
    mkdir(params.log_directory);L_12
end

%% Check if dynamics functions exist
if ~exist('robot_inertia.m', 'file') || ~exist('robot_coriolis.m', 'file') || ~exist('robot_gravity.m', 'file')
    warning(['Dynamics functions not found. Run dynamics.m first to generate these functions:', ...
        '\n  - robot_inertia.m', ...
        '\n  - robot_coriolis.m', ...
        '\n  - robot_gravity.m']);
end

%% Display setup confirmation
fprintf('Setup complete. Robot parameters initialized.\n');
fprintf('Link lengths: L45 = %.6f m, L6 = %.6f m\n', params.L_45, params.L_6);
fprintf('Sample time: %.4f s\n', params.sample_time);

% Return the parameters to the workspace
assignin('base', 'params', params);