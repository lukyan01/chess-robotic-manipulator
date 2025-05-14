%% Test PID Controller with Individual Joint Controllers
% This script tests the PID controller with inverse dynamics on the 7-DOF robot
% Each joint has its own individual PID controller

clear; clc; close all;

% Add paths and load parameters
addpath(genpath('../functions'));
addpath(genpath('./data/generated_functions')); % Path to dynamic model functions

%% Robot Model Parameters
% These should match the parameters used in generate_dynamics.m
model_params = struct();
model_params.L_45 = 0.121851;  % Link 4-5 length (m)
model_params.L_6 = 0.1;        % Link 6 length (m)
model_params.m1 = 0.5;         % Base mass (kg)
model_params.m2 = 0.7;         % Vertical prismatic joint mass (kg)
model_params.m3 = 0.5;         % Horizontal prismatic joint mass (kg)
model_params.m4 = 0.5;         % Link 4 mass (kg)
model_params.m5 = 0.5;         % Link 5 mass (kg)
model_params.m6 = 0.3;         % Link 6 mass (kg)
model_params.m7 = 0.1;         % End effector mass (kg)
model_params.g = 9.81;         % Gravity (m/s^2)

link_lengths = [model_params.L_45; model_params.L_6];

% Simulation parameters
sample_time = 0.01;  % 100 Hz sampling rate

%% Individual PID Controller Parameters
% Create PID parameters for each joint
pid_params = cell(7, 1);

% Base rotation (revolute)
pid_params{1} = struct(...
    'Kp', 100, ...
    'Ki', 5, ...
    'Kd', 10, ...
    'max_integral', 10.0, ...
    'joint_type', 'revolute');

% Vertical prismatic joint
pid_params{2} = struct(...
    'Kp', 400, ...
    'Ki', 20, ...
    'Kd', 40, ...
    'max_integral', 10.0, ...
    'joint_type', 'prismatic');

% Horizontal prismatic joint
pid_params{3} = struct(...
    'Kp', 400, ...
    'Ki', 20, ...
    'Kd', 40, ...
    'max_integral', 10.0, ...
    'joint_type', 'prismatic');

% Joint 4 (revolute)
pid_params{4} = struct(...
    'Kp', 100, ...
    'Ki', 5, ...
    'Kd', 10, ...
    'max_integral', 10.0, ...
    'joint_type', 'revolute');

% Joint 5 (revolute)
pid_params{5} = struct(...
    'Kp', 100, ...
    'Ki', 5, ...
    'Kd', 10, ...
    'max_integral', 10.0, ...
    'joint_type', 'revolute');

% Joint 6 (revolute)
pid_params{6} = struct(...
    'Kp', 100, ...
    'Ki', 5, ...
    'Kd', 10, ...
    'max_integral', 10.0, ...
    'joint_type', 'revolute');

% Joint 7 (revolute)
pid_params{7} = struct(...
    'Kp', 50, ...
    'Ki', 2, ...
    'Kd', 5, ...
    'max_integral', 10.0, ...
    'joint_type', 'revolute');

%% Test 1: Trajectory Tracking - Chess Move
fprintf('Test 1: Trajectory Tracking - Chess Move\n');

% Generate a chess move trajectory
start_square = 'e2';
end_square = 'e4';
move_duration = 2.0;  % seconds

fprintf('Generating trajectory for move: %s to %s\n', start_square, end_square);
[q_traj, qd_traj, qdd_traj, time_vec] = generateChessTraj(start_square, end_square, link_lengths, move_duration, sample_time);

% Simulate robot dynamics with individual joint PID controllers
fprintf('Simulating robot dynamics with individual joint PID controllers...\n');
[q_actual, qd_actual, qdd_actual, tau_history] = simulateRobotDynamicsWithJointPID(q_traj, qd_traj, qdd_traj, time_vec, pid_params, model_params, struct('enabled', false));

% Visualize control performance
visualizeControlPerformance(time_vec, q_traj, qd_traj, qdd_traj, q_actual, qd_actual, qdd_actual, tau_history, link_lengths);
sgtitle('Test 1: Chess Move Trajectory Tracking with Individual Joint PIDs');

%% Test 2: Different PID Parameters for One Joint
fprintf('\nTest 2: Testing Different PID Parameters for Joint 1\n');

% Define different PID parameters to test for Joint 1 (Base rotation)
pid_test_params = {
    struct('Kp', 50, 'Ki', 2, 'Kd', 5, 'max_integral', 10.0, 'joint_type', 'revolute', 'label', 'Low Gains'),
    struct('Kp', 100, 'Ki', 5, 'Kd', 10, 'max_integral', 10.0, 'joint_type', 'revolute', 'label', 'Medium Gains'),
    struct('Kp', 200, 'Ki', 10, 'Kd', 20, 'max_integral', 10.0, 'joint_type', 'revolute', 'label', 'High Gains')
};

% Use the chess trajectory from Test 1
figure('Name', 'Joint 1 PID Parameter Comparison', 'Position', [100, 100, 1200, 600]);
color_map = {'r-', 'g-', 'b-'};

subplot(2,2,1);
hold on; grid on;
plot(time_vec, q_traj(1,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Desired');

for i = 1:length(pid_test_params)
    % Create a copy of the PID parameters
    test_pid_params = pid_params;
    
    % Replace joint 1 parameters with test parameters
    test_pid_params{4} = pid_test_params{i};
    
    % Simulate with these parameters
    [q_test, ~, ~, tau_test] = simulateRobotDynamicsWithJointPID(q_traj, qd_traj, qdd_traj, time_vec, test_pid_params, model_params, struct('enabled', false));
    
    % Plot results
    plot(time_vec, q_test(4,:), color_map{i}, 'LineWidth', 1.5, 'DisplayName', pid_test_params{4}.label);
end
title('Joint 1 Position Tracking');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Location', 'best');

subplot(2,2,2);
hold on; grid on;
for i = 1:length(pid_test_params)
    test_pid_params = pid_params;
    test_pid_params{1} = pid_test_params{i};
    [q_test, ~, ~, tau_test] = simulateRobotDynamicsWithJointPID(q_traj, qd_traj, qdd_traj, time_vec, test_pid_params, model_params, struct('enabled', false));
    error = q_traj(1,:) - q_test(1,:);
    plot(time_vec, error, color_map{i}, 'LineWidth', 1.5, 'DisplayName', pid_test_params{i}.label);
end
title('Joint 1 Position Error');
xlabel('Time (s)');
ylabel('Error (rad)');
legend('Location', 'best');

subplot(2,2,3);
hold on; grid on;
for i = 1:length(pid_test_params)
    test_pid_params = pid_params;
    test_pid_params{1} = pid_test_params{i};
    [q_test, ~, ~, tau_test] = simulateRobotDynamicsWithJointPID(q_traj, qd_traj, qdd_traj, time_vec, test_pid_params, model_params, struct('enabled', false));
    plot(time_vec, tau_test(1,:), color_map{i}, 'LineWidth', 1.5, 'DisplayName', pid_test_params{i}.label);
end
title('Joint 1 Control Torque');
xlabel('Time (s)');
ylabel('Torque (N·m)');
legend('Location', 'best');

subplot(2,2,4);
hold on; grid on;
for i = 1:length(pid_test_params)
    % Calculate metrics for display
    test_pid_params = pid_params;
    test_pid_params{1} = pid_test_params{i};
    [q_test, ~, ~, ~] = simulateRobotDynamicsWithJointPID(q_traj, qd_traj, qdd_traj, time_vec, test_pid_params, model_params, struct('enabled', false));
    error = q_traj(1,:) - q_test(1,:);
    
    % Calculate metrics
    max_error = max(abs(error));
    rms_error = sqrt(mean(error.^2));
    
    % Plot as bar chart
    bar(i, max_error, 'FaceColor', color_map{i}(1));
    text(i, max_error + 0.002, sprintf('Max: %.4f', max_error), 'HorizontalAlignment', 'center');
    
    % Add RMS error as text
    text(i, max_error/2, sprintf('RMS: %.4f', rms_error), 'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold');
end
title('Joint 1 Error Metrics');
ylabel('Max Error (rad)');
xticks(1:length(pid_test_params));
xticklabels({pid_test_params{1}.label, pid_test_params{2}.label, pid_test_params{3}.label});
grid on;

%% Test 3: Step Response with Individual Joint Controllers
fprintf('\nTest 3: Step Response with Individual Joint Controllers\n');

% Generate a step input for each joint
duration = 3.0;  % seconds
time_vec_step = 0:sample_time:duration;
n_steps = length(time_vec_step);

% Initial and target joint positions
q_init = [0; 0.15; 0.3; pi/6; -pi/3; -pi/4; 0];
q_target = [pi/6; 0.2; 0.4; pi/4; -pi/2; -pi/6; pi/12];

% Create step trajectory
q_step = zeros(7, n_steps);
qd_step = zeros(7, n_steps);
qdd_step = zeros(7, n_steps);

for i = 1:n_steps
    if time_vec_step(i) < 0.5
        q_step(:,i) = q_init;
    else
        q_step(:,i) = q_target;
    end
end

% Simulate robot dynamics with step input
fprintf('Simulating step response...\n');
[q_step_actual, qd_step_actual, qdd_step_actual, tau_step] = simulateRobotDynamicsWithJointPID(q_step, qd_step, qdd_step, time_vec_step, pid_params, model_params, struct('enabled', false));

% Visualize step response
visualizeControlPerformance(time_vec_step, q_step, qd_step, qdd_step, q_step_actual, qd_step_actual, qdd_step_actual, tau_step, link_lengths);
sgtitle('Test 3: Step Response with Individual Joint PIDs');

%% Test 4: Disturbance Rejection with Individual Joint Controllers
fprintf('\nTest 4: Disturbance Rejection with Individual Joint Controllers\n');

% Use the chess trajectory from Test 1
% Apply a disturbance at t = 1.0s to the base joint

% Define disturbance
disturbance = struct();
disturbance.enabled = true;
disturbance.time = 1.0;     % Apply at 1 second
disturbance.joint = 1;      % Apply to base joint
disturbance.magnitude = 10; % Large torque disturbance

% Simulate with disturbance
fprintf('Simulating disturbance rejection...\n');
[q_dist, qd_dist, qdd_dist, tau_dist] = simulateRobotDynamicsWithJointPID(q_traj, qd_traj, qdd_traj, time_vec, pid_params, model_params, disturbance);

% Visualize disturbance rejection
visualizeControlPerformance(time_vec, q_traj, qd_traj, qdd_traj, q_dist, qd_dist, qdd_dist, tau_dist, link_lengths);
sgtitle('Test 4: Disturbance Rejection with Individual Joint PIDs');

% Plot specific joint with disturbance highlighted
figure('Name', 'Disturbance Rejection - Joint 1', 'Position', [200, 200, 800, 400]);
subplot(2,1,1);
plot(time_vec, q_traj(1,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Desired');
hold on;
plot(time_vec, q_dist(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
xline(disturbance.time, 'g--', 'LineWidth', 2, 'DisplayName', 'Disturbance');
xlabel('Time (s)');
ylabel('Position (rad)');
title('Joint 1 Position with Disturbance');
legend('Location', 'best');
grid on;

subplot(2,1,2);
plot(time_vec, tau_dist(1,:), 'k-', 'LineWidth', 1.5);
hold on;
xline(disturbance.time, 'g--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Torque (N·m)');
title('Joint 1 Control Torque');
grid on;

fprintf('\nPID Controller Tests Completed with Individual Joint Controllers\n');