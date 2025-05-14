function [optimized_pid_params] = tuneJointPIDWithHinf(robot_model, joint_linear_models, initial_pid_params, options)
% TUNEPIDCONTROLLERWITHHINF Automatically tunes PID controllers using H∞ optimization
% in the frequency domain
%
% Inputs:
%   robot_model      - Robot model parameters structure
%   joint_linear_models - Cell array of identified linear models for each joint
%                       (state-space or transfer function models)
%   initial_pid_params - Cell array of initial PID parameters for each joint
%   options          - Optimization options structure with fields:
%                      .joints_to_tune - Array of joint indices to tune [default: all]
%                      .w - Frequency vector for analysis [default: logspace(-2, 3, 100)]
%                      .Ms - Maximum sensitivity peak constraint [default: 2.0]
%                      .Mt - Maximum complementary sensitivity peak [default: 1.5]
%                      .Mu - Maximum control sensitivity peak [default: 10]
%                      .wb - Desired bandwidth [default: 10 rad/s]
%                      .PM - Desired phase margin in degrees [default: 45]
%                      .GM - Desired gain margin in dB [default: 6]
%
% Outputs:
%   optimized_pid_params - Cell array of optimized PID parameters for each joint
%
% Example:
%   options = struct('joints_to_tune', [1, 4, 5], 'wb', 15, 'PM', 60);
%   optimized_params = tuneJointPIDWithHinf(model_params, linear_models, pid_params, options);

% Handle default options
if nargin < 4
    options = struct();
end

function pid_params = getPIDParamsFromController(C)
% Alternative function to extract PID parameters from controller
% This is a fallback in case piddata() doesn't work

% Get frequency response at very low frequency (for integral action)
[mag_low, ~] = freqresp(C, 0.0001);
Ki_approx = abs(mag_low) * 0.0001;

% Get frequency response at medium frequency (for proportional action)
[mag_mid, ~] = freqresp(C, 1);
Kp_approx = abs(mag_mid);

% Get frequency response at high frequency (for derivative action)
[mag_high, ~] = freqresp(C, 100);
Kd_approx = abs(mag_high) / 100;

% Return approximate PID parameters
pid_params = struct('Kp', Kp_approx, 'Ki', Ki_approx, 'Kd', Kd_approx);
end

% Number of joints
num_joints = length(initial_pid_params);

% Set default options
if ~isfield(options, 'joints_to_tune')
    options.joints_to_tune = 1:num_joints;  % Tune all joints by default
end
if ~isfield(options, 'w')
    options.w = logspace(-2, 3, 100);  % Frequency range for analysis
end
if ~isfield(options, 'Ms')
    options.Ms = 2.0;  % Maximum sensitivity peak constraint
end
if ~isfield(options, 'Mt')
    options.Mt = 1.5;  % Maximum complementary sensitivity peak
end
if ~isfield(options, 'Mu')
    options.Mu = 10;   % Maximum control sensitivity peak
end
if ~isfield(options, 'wb')
    options.wb = 10;   % Desired bandwidth in rad/s
end
if ~isfield(options, 'PM')
    options.PM = 45;   % Desired phase margin in degrees
end
if ~isfield(options, 'GM')
    options.GM = 6;    % Desired gain margin in dB
end

% Initialize result to the initial parameters
optimized_pid_params = initial_pid_params;

% Optimize for each joint in the list
for j = options.joints_to_tune
    fprintf('Performing H∞ optimization for joint %d (%s)...\n', j, initial_pid_params{j}.joint_type);
    
    % Get linear model for this joint
    P = joint_linear_models{j};
    
    % Check if we have a valid model
    if isempty(P)
        warning('No linear model available for joint %d. Skipping.', j);
        continue;
    end
    
    % Initial gains for this joint
    Kp0 = initial_pid_params{j}.Kp;
    Ki0 = initial_pid_params{j}.Ki;
    Kd0 = initial_pid_params{j}.Kd;
    
    % Create initial PID controller
    C0 = pid(Kp0, Ki0, Kd0);
    
    % Create control design requirements 
    % Create separate margin requirements for gain and phase margins
    req1 = TuningGoal.GainMargin(options.GM);
    req2 = TuningGoal.PhaseMargin(options.PM);
    
    % Initialize requirement array
    req = [req1; req2];
    
    % Add bandwidth requirement if available in your MATLAB version
    try
        req = [req; TuningGoal.Bandwidth(options.wb)];
    catch
        warning('TuningGoal.Bandwidth not available. Using alternative approach.');
        % Alternative: use crossover frequency requirement
        req = [req; TuningGoal.Crossover(options.wb)];
    end
    
    % Add sensitivity constraints
    try
        req = [req; TuningGoal.Sensitivity(options.Ms)];
    catch
        warning('TuningGoal.Sensitivity not available. Using alternative approach.');
        % Alternative: use maximum sensitivity peak constraint
        req = [req; TuningGoal.MaxSensitivity(options.Ms)];
    end
    
    % Add maximum control effort constraint
    try
        req = [req; TuningGoal.MaxLoopGain(options.Mu, options.w(end))];
    catch
        warning('TuningGoal.MaxLoopGain not available. Skipping control effort constraint.');
    end
    
    % Setup systune options
    opt = systuneOptions('RandomStart', 5, 'Display', 'final');
    
    % Perform H∞ optimization
    try
        [C_tuned, fSoft] = systune(P, C0, req, opt);
    catch ME
        warning('systune failed: %s. Falling back to pidtune.', ME.message);
        % Fall back to basic PID tuning
        try
            C_tuned = pidtune(P, 'pid', options.wb);
            fSoft = NaN;
        catch ME2
            warning('pidtune also failed: %s. Keeping original PID values.', ME2.message);
            C_tuned = C0;
            fSoft = Inf;
        end
    end
    
    % Extract optimized PID parameters (with error handling)
    try
        [Kp, Ki, Kd] = piddata(C_tuned);
    catch
        % Alternative method to extract PID data if piddata fails
        try
            pid_params = getPIDParamsFromController(C_tuned);
            Kp = pid_params.Kp;
            Ki = pid_params.Ki;
            Kd = pid_params.Kd;
        catch
            warning('Failed to extract PID parameters. Using original values.');
            Kp = Kp0;
            Ki = Ki0;
            Kd = Kd0;
        end
    end
    
    % Update optimized parameters
    optimized_pid_params{j}.Kp = Kp;
    optimized_pid_params{j}.Ki = Ki;
    optimized_pid_params{j}.Kd = Kd;
    
    fprintf('Joint %d H∞ optimization completed. Soft goal attainment: %.4f\n', j, fSoft);
    fprintf('Optimized gains: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp, Ki, Kd);
    
    % Analysis of the optimized controller
    L = P * C_tuned;  % Open loop
    S = feedback(1, L);  % Sensitivity
    T = 1 - S;  % Complementary sensitivity
    
    % Plot frequency responses
    figure('Name', sprintf('Joint %d H∞ Controller Analysis', j));
    
    % Bode plot
    subplot(2, 2, 1);
    bodeplot(L);
    title('Open Loop Bode Plot');
    grid on;
    
    % Sensitivity
    subplot(2, 2, 2);
    bodemag(S);
    title('Sensitivity Function');
    grid on;
    hold on;
    semilogx(options.w, options.Ms*ones(size(options.w)), 'r--');
    legend('S', 'Constraint');
    
    % Complementary sensitivity
    subplot(2, 2, 3);
    bodemag(T);
    title('Complementary Sensitivity');
    grid on;
    hold on;
    semilogx(options.w, options.Mt*ones(size(options.w)), 'r--');
    legend('T', 'Constraint');
    
    % Step response
    subplot(2, 2, 4);
    step(feedback(P*C_tuned, 1));
    title('Closed-Loop Step Response');
    grid on;
end

% Display final results
fprintf('\nH∞ PID Tuning Completed for %d joints.\n', length(options.joints_to_tune));

% Utility function to analyze the closed-loop performance
analyzeClosedLoopPerformance(joint_linear_models, optimized_pid_params, options);
end

function analyzeClosedLoopPerformance(joint_models, pid_params, options)
% Analyze and display the closed-loop performance metrics

fprintf('\nClosed-Loop Performance Analysis:\n');
fprintf('-------------------------------\n');
fprintf('Joint | Bandwidth | Phase Margin | Gain Margin | Rise Time | Settling Time | Overshoot\n');
fprintf('-------------------------------\n');

for j = options.joints_to_tune
    % Skip if no model available
    if isempty(joint_models{j})
        continue;
    end
    
    % Get plant model
    P = joint_models{j};
    
    % Create controller
    C = pid(pid_params{j}.Kp, pid_params{j}.Ki, pid_params{j}.Kd);
    
    % Open loop
    L = P * C;
    
    % Closed loop
    CL = feedback(P*C, 1);
    
    % Calculate performance metrics
    [Gm, Pm, Wcg, Wcp] = margin(L);
    Pm = Pm;  % Phase margin in degrees
    Gm = 20*log10(Gm);  % Gain margin in dB
    
    % Estimate bandwidth from the -3dB point of the closed-loop system
    [mag, ~, wout] = bode(CL, options.w);
    mag = squeeze(mag);
    idx_bw = find(mag < 0.7071, 1, 'first');  % -3dB point
    if ~isempty(idx_bw) && idx_bw > 1
        bandwidth = wout(idx_bw);
    else
        bandwidth = NaN;
    end
    
    % Get step response metrics
    S = stepinfo(CL);
    rise_time = S.RiseTime;
    settling_time = S.SettlingTime;
    overshoot = S.Overshoot;
    
    % Print results
    fprintf('%5d | %9.2f | %12.2f | %10.2f | %9.3f | %12.3f | %8.2f\n', ...
        j, bandwidth, Pm, Gm, rise_time, settling_time, overshoot);
end

fprintf('-------------------------------\n');
end