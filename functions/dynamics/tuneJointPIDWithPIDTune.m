function [optimized_pid_params] = tuneJointPIDWithPIDTune(joint_linear_models, initial_pid_params, options)
% TUNEPIDWITHPIDTUNE Tunes PID controllers using MATLAB's pidtune function
%
% Inputs:
%   joint_linear_models - Cell array of identified linear models for each joint
%   initial_pid_params - Cell array of initial PID parameters for each joint
%   options          - Options structure with fields:
%                      .joints_to_tune - Array of joint indices to tune [default: all]
%                      .bandwidth - Desired closed-loop bandwidth [default: varies by joint]
%                      .phase_margin - Desired phase margin in degrees [default: 60]
%
% Outputs:
%   optimized_pid_params - Cell array of optimized PID parameters for each joint
%
% Example:
%   options = struct('joints_to_tune', [1, 4, 5], 'bandwidth', 10, 'phase_margin', 60);
%   optimized_params = tuneJointPIDWithPIDTune(linear_models, pid_params, options);

% Handle default options
if nargin < 3
    options = struct();
end

% Number of joints
num_joints = length(initial_pid_params);

% Set default options
if ~isfield(options, 'joints_to_tune')
    options.joints_to_tune = 1:num_joints;  % Tune all joints by default
end
if ~isfield(options, 'phase_margin')
    options.phase_margin = 60;  % Default phase margin (degrees)
end

% Default bandwidths per joint type if not specified
if ~isfield(options, 'bandwidth')
    default_bandwidths = [
        10;   % Joint 1 (base rotation)
        5;    % Joint 2 (vertical prismatic)
        5;    % Joint 3 (horizontal prismatic)
        15;   % Joint 4 (revolute)
        15;   % Joint 5 (revolute)
        15;   % Joint 6 (revolute)
        10    % Joint 7 (revolute)
    ];
    options.bandwidth = default_bandwidths;
end

% Initialize result to the initial parameters
optimized_pid_params = initial_pid_params;

% Optimize for each joint in the list
for j = options.joints_to_tune
    fprintf('Tuning joint %d (%s) with pidtune...\n', j, initial_pid_params{j}.joint_type);
    
    % Get linear model for this joint
    P = joint_linear_models{j};
    
    % Check if we have a valid model
    if isempty(P)
        warning('No linear model available for joint %d. Skipping.', j);
        continue;
    end
    
    % Get bandwidth for this joint
    if isscalar(options.bandwidth)
        % Single bandwidth for all joints
        wb = options.bandwidth;
    else
        % Joint-specific bandwidth
        wb = options.bandwidth(j);
    end
    
    % Set pidtune options
    opt = pidtuneOptions('PhaseMargin', options.phase_margin);
    
    try
        % Tune PID controller with pidtune
        C = pidtune(P, 'pid', wb, opt);
        
        % Extract PID parameters
        [Kp, Ki, Kd] = piddata(C);
        
        % Update optimized parameters
        optimized_pid_params{j}.Kp = Kp;
        optimized_pid_params{j}.Ki = Ki;
        optimized_pid_params{j}.Kd = Kd;
        
        fprintf('Joint %d tuned successfully: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', j, Kp, Ki, Kd);
        
        % Analyze the closed-loop system
        CL = feedback(P*C, 1);
        
        % Get step response metrics
        S = stepinfo(CL);
        rise_time = S.RiseTime;
        settling_time = S.SettlingTime;
        overshoot = S.Overshoot;
        
        fprintf('  Rise time: %.3f s, Settling time: %.3f s, Overshoot: %.2f%%\n', ...
            rise_time, settling_time, overshoot);
        
        % Plot results for this joint
        figure('Name', sprintf('Joint %d PID Tuning Results', j));
        
        % Step response
        subplot(2, 1, 1);
        step(CL);
        title(sprintf('Joint %d - Closed-Loop Step Response', j));
        grid on;
        
        % Open-loop Bode plot
        subplot(2, 1, 2);
        margin(P*C);
        title(sprintf('Joint %d - Open-Loop Frequency Response', j));
        grid on;
        
    catch ME
        warning('PID tuning failed for joint %d: %s', j, ME.message);
    end
end

fprintf('PID tuning completed for %d joints.\n', length(options.joints_to_tune));
end