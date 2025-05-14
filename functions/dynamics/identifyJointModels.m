function joint_models = identifyJointModels(robot_model, options)
% IDENTIFYJOINTMODELS Identifies linear models for each joint of the robot
%
% This function performs system identification to create linear models
% of each joint that can be used for H-infinity controller design
%
% Inputs:
%   robot_model - Robot model parameters structure
%   options     - Options structure with fields:
%                 .model_order - Order of linear models [default: 2]
%                 .excitation_amplitude - Input signal amplitude [default: varies by joint]
%                 .excitation_freq - Frequency range for PRBS [default: [0.1, 20]]
%                 .simulation_time - Simulation time for identification [default: 10]
%                 .sample_time - Sample time [default: 0.01]
%
% Outputs:
%   joint_models - Cell array of identified linear models for each joint
%
% Example:
%   options = struct('model_order', 3, 'simulation_time', 15);
%   joint_models = identifyJointModels(model_params, options);

% Set default options
if nargin < 2
    options = struct();
end

if ~isfield(options, 'model_order')
    options.model_order = 2;  % Default 2nd order models
end

if ~isfield(options, 'simulation_time')
    options.simulation_time = 10;  % 10 seconds of excitation
end

if ~isfield(options, 'sample_time')
    options.sample_time = 0.01;  % 100 Hz sampling
end

if ~isfield(options, 'excitation_freq')
    options.excitation_freq = [0.1, 20];  % Frequency range in Hz
end

% Number of joints
num_joints = 7;

% Initialize output
joint_models = cell(num_joints, 1);

% Time vector for simulation
t = 0:options.sample_time:options.simulation_time;
n_samples = length(t);

% Generate default excitation amplitudes based on joint type
if ~isfield(options, 'excitation_amplitude')
    options.excitation_amplitude = [
        0.3;    % Base rotation (rad)
        0.05;   % Vertical prismatic (m)
        0.05;   % Horizontal prismatic (m)
        0.3;    % Joint 4 rotation (rad)
        0.3;    % Joint 5 rotation (rad)
        0.3;    % Joint 6 rotation (rad)
        0.3     % Joint 7 rotation (rad)
    ];
end

% Joint types (for printing)
joint_types = {'revolute', 'prismatic', 'prismatic', 'revolute', 'revolute', 'revolute', 'revolute'};

% Starting position (mid-range for each joint)
q_mid = [0; 0.175; 0.32; pi/4; -pi/2; -pi/4; 0];

fprintf('System Identification for Robot Joints\n');
fprintf('--------------------------------------\n');

% Identify model for each joint
for joint_idx = 1:num_joints
    fprintf('Identifying model for joint %d (%s)...\n', joint_idx, joint_types{joint_idx});
    
    % Create excitation signal (PRBS with frequency content)
    amplitude = options.excitation_amplitude(joint_idx);
    excitation = generateExcitationSignal(t, amplitude, options.excitation_freq);
    
    % Initialize data arrays
    u = zeros(n_samples, 1);  % Input torque/force
    y = zeros(n_samples, 1);  % Output position
    
    % Initial conditions
    q = q_mid;
    q_dot = zeros(size(q));
    
    % Extract model parameters
    L_45 = robot_model.L_45;
    L_6 = robot_model.L_6;
    m1 = robot_model.m1;
    m2 = robot_model.m2;
    m3 = robot_model.m3;
    m4 = robot_model.m4;
    m5 = robot_model.m5;
    m6 = robot_model.m6;
    m7 = robot_model.m7;
    g = robot_model.g;
    
    % Damping matrix
    B = diag([0.5, 1.0, 1.0, 0.2, 0.2, 0.2, 0.1]);
    
    % Simulate the joint response to excitation
    for i = 1:n_samples-1
        % Store current position
        y(i) = q(joint_idx);
        
        % Apply torque to the current joint only
        tau = zeros(num_joints, 1);
        tau(joint_idx) = excitation(i);
        u(i) = excitation(i);
        
        % Get inverse dynamics components
        M = robot_inertia(q, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
        C = robot_coriolis(q, q_dot, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
        G = robot_gravity(q, g, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
        
        % Add damping
        friction = B * q_dot;
        
        % Compute acceleration
        q_ddot = M \ (tau - C * q_dot - G - friction);
        
        % Integrate (semi-implicit Euler)
        q_dot = q_dot + q_ddot * options.sample_time;
        q = q + q_dot * options.sample_time;
        
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
            if q(j) < joint_limits(j,1)
                q(j) = joint_limits(j,1);
                q_dot(j) = 0;
            elseif q(j) > joint_limits(j,2)
                q(j) = joint_limits(j,2);
                q_dot(j) = 0;
            end
        end
    end
    
    % Store final position
    y(end) = q(joint_idx);
    
    % Create data object for system identification
    data = iddata(y, u, options.sample_time);
    
    % Remove mean (steady-state offset)
    data = detrend(data);
    
    % Identify state-space model
    try
        sys = ssest(data, options.model_order, ssestOptions('Display', 'off'));
        joint_models{joint_idx} = sys;
        
        % Calculate and display fit percentage
        [~, fit, ~] = compare(data, sys);
        fprintf('  Model order: %d, Fit: %.2f%%\n', options.model_order, fit);
        
        % Plot identification results
        figure('Name', sprintf('Joint %d System Identification', joint_idx));
        
        % Input and output signals
        subplot(2,2,1);
        plot(t, u);
        title('Excitation Input');
        xlabel('Time (s)');
        if joint_idx <= 1 || joint_idx >= 4
            ylabel('Torque (N·m)');
        else
            ylabel('Force (N)');
        end
        grid on;
        
        subplot(2,2,2);
        plot(t, y);
        title('Joint Position Output');
        xlabel('Time (s)');
        if joint_idx <= 1 || joint_idx >= 4
            ylabel('Position (rad)');
        else
            ylabel('Position (m)');
        end
        grid on;
        
        % Compare model output vs measured data
        subplot(2,2,3);
        compare(data, sys);
        title('Model vs Measured Data');
        grid on;
        
        % Bode plot of the identified model
        subplot(2,2,4);
        bodeplot(sys);
        title('Frequency Response');
        grid on;
        
    catch ME
        warning('Failed to identify model for joint %d: %s', joint_idx, ME.message);
        joint_models{joint_idx} = [];
    end
end

fprintf('System identification completed.\n');
end

function excitation = generateExcitationSignal(t, amplitude, freq_range)
% Generates a multi-sine excitation signal with desired frequency content
    
    % Number of frequency components
    n_freqs = 10;
    
    % Create logarithmically spaced frequencies in the desired range
    freqs = logspace(log10(freq_range(1)), log10(freq_range(2)), n_freqs);
    
    % Random phases
    phases = 2*pi*rand(n_freqs, 1);
    
    % Generate multi-sine signal
    excitation = zeros(size(t));
    for i = 1:n_freqs
        excitation = excitation + sin(2*pi*freqs(i)*t + phases(i));
    end
    
    % Normalize and scale to desired amplitude
    excitation = amplitude * excitation / max(abs(excitation));
    
    % Add small amount of white noise for better excitation
    excitation = excitation + 0.05 * amplitude * randn(size(excitation));
end