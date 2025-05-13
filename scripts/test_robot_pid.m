%% Chess Robot PID Control and Simulation
% This script demonstrates how to create, tune, and simulate the 7-DOF
% chess robot with PID control for trajectory tracking.

clear;
close all;
clc;

% Add paths to your functions
addpath(genpath('functions'));
addpath(genpath('models'));
addpath(genpath('controllers'));
addpath(fullfile('data','generated_functions'));  % Add path to generated dynamics functions

%% Create Robot Model and Controller
disp('Creating robot model and controller...');

% Create robot model
robot = ChessRobotModel();

% Create PID controller
controller = RobotPIDController(robot);

%% PID Tuning Options
% Uncomment the following line to open the PID Tuner for a specific joint
controller.pidTuner(1);  % Tune the first joint (base rotation)

%% Run Multi-Joint Step Response Test - Testing All Joints
disp('Running step response test for all joints...');

% Simulation parameters
sim_time = 2.0;     % Simulation time (s)
dt = 0.001;         % Time step (s)
time = 0:dt:sim_time;
n_steps = length(time);

% Define joint test limits - step value for each joint
joint_steps = [pi/4, ...  % theta_1: 45 degrees rotation
               0.1,  ...  % d_2: 10 cm vertical
               0.1,  ...  % d_3: 10 cm horizontal
               pi/6, ...  % theta_4: 30 degrees 
               -pi/6,...  % theta_5: -30 degrees
               pi/4, ...  % theta_6: 45 degrees
               pi/3];     % theta_7: 60 degrees

% Initialize arrays for all joints
q_traj_all = cell(7, 1);
qd_traj_all = cell(7, 1);
u_traj_all = cell(7, 1);

% Test each joint separately
for joint_idx = 1:7
    disp(['Testing joint ', num2str(joint_idx), '...']);
    
    % Initial and target joint positions
    q_init = zeros(7, 1);
    q_target = q_init;
    q_target(joint_idx) = joint_steps(joint_idx);  % Step input
    
    % Initialize simulation variables
    q_traj = zeros(n_steps, 7);
    qd_traj = zeros(n_steps, 7);
    u_traj = zeros(n_steps, 7);
    
    % Set initial state
    robot.q = q_init;
    robot.qd = zeros(7, 1);
    
    % Reset controller
    controller.resetIntegrators();
    
    % Run simulation
    for i = 1:n_steps
        % Get current state
        q_current = robot.q;
        qd_current = robot.qd;
        
        % Compute control input
        u = controller.computeControl(q_target, q_current, qd_current, dt);
        
        % Apply control and step forward
        [x_next, ~] = robot.step(u, dt);
        
        % Record trajectory
        q_traj(i, :) = q_current';
        qd_traj(i, :) = qd_current';
        u_traj(i, :) = u';
    end
    
    % Store results
    q_traj_all{joint_idx} = q_traj;
    qd_traj_all{joint_idx} = qd_traj;
    u_traj_all{joint_idx} = u_traj;
end

% Plot results for all joints
figure('Name', 'Multi-Joint Step Response', 'Position', [100, 100, 1200, 800]);

joint_names = {'θ₁ (Base)', 'd₂ (Vertical)', 'd₃ (Horizontal)', 'θ₄ (Elbow)', 'θ₅ (Elbow)', 'θ₆ (Wrist)', 'θ₇ (End)'};
colors = [
    0.8500, 0.3250, 0.0980;  % Red-orange
    0.0000, 0.4470, 0.7410;  % Blue
    0.4660, 0.6740, 0.1880;  % Green
    0.4940, 0.1840, 0.5560;  % Purple
    0.9290, 0.6940, 0.1250;  % Yellow
    0.3010, 0.7450, 0.9330;  % Cyan
    0.6350, 0.0780, 0.1840;  % Dark red
];

for joint_idx = 1:7
    subplot(4, 2, joint_idx);
    hold on;
    
    % Get data for this joint
    q_traj = q_traj_all{joint_idx};
    q_target_val = zeros(size(time));
    q_target_val(:) = joint_steps(joint_idx);
    
    % Plot step response
    plot(time, q_traj(:, joint_idx), 'Color', colors(joint_idx,:), 'LineWidth', 2);
    plot(time, q_target_val, 'r--', 'LineWidth', 1.5);
    
    grid on;
    title(sprintf('Joint %s Response', joint_names{joint_idx}));
    xlabel('Time (s)');
    
    if joint_idx <= 3
        ylabel('Position (rad or m)');
    else
        ylabel('Position (rad)');
    end
    
    legend('Actual', 'Target');
    
    % Calculate performance metrics
    settling_time = findSettlingTime(time, q_traj(:, joint_idx), q_target_val, 0.05);
    overshoot = findOvershoot(q_traj(:, joint_idx), q_target_val(1));
    
    % Add performance metrics to the plot
    text(0.6*sim_time, 0.2*joint_steps(joint_idx), sprintf('Settling: %.2fs\nOvershoot: %.1f%%', settling_time, overshoot*100), 'FontSize', 8);
end

% Function to calculate settling time (5% criterion)
function ts = findSettlingTime(time, response, target, threshold)
    % Calculate error signal
    error = abs(response - target) / abs(target(end));
    
    % Find settling time (when error stays below threshold)
    settled = error < threshold;
    idx = find(settled, 1, 'first');
    
    if isempty(idx)
        ts = inf;  % Not settled
    else
        ts = time(idx);
    end
end

% Function to calculate overshoot
function os = findOvershoot(response, initial)
    if initial == 0
        % Avoid division by zero
        peak = max(abs(response));
        final = response(end);
        os = (peak - abs(final)) / abs(final);
    else
        % Standard overshoot calculation
        peak = max(abs(response));
        final = abs(response(end));
        os = (peak - final) / (final - initial);
    end
    
    % Limit to non-negative
    os = max(0, os);
end

%% Run Chess Move Trajectory Tracking Test - Using generateChessTraj
disp('Running chess move trajectory tracking test...');

% Define a series of chess moves to test
chess_moves = {
    'e2', 'e4',  % Classic opening
    'a1', 'h8',  % Diagonal move (most complex)
    'h1', 'a8',  % Opposite diagonal
    'd4', 'd5'   % Short vertical move
};

move_duration = 1.0;  % seconds
sample_time = 0.01;   % trajectory sampling time

% Loop through moves
for move_idx = 1:2:length(chess_moves)
    % Get start and end squares
    start_square = chess_moves{move_idx};
    end_square = chess_moves{move_idx+1};
    
    disp(['Testing chess move: ', start_square, ' to ', end_square]);
    
    % Generate trajectory for the chess move
    [q_traj_target, qd_traj_target, qdd_traj_target, traj_time] = generateChessTraj(start_square, end_square, [robot.L_45; robot.L_6], move_duration, sample_time);

    % Convert to row-wise for visualization
    q_traj_target = q_traj_target';
    qd_traj_target = qd_traj_target';
    qdd_traj_target = qdd_traj_target';

    % Resample trajectory for control simulation (if needed)
    if sample_time ~= dt
        t_control = 0:dt:move_duration;
        q_target_resampled = zeros(length(t_control), 7);
        qd_target_resampled = zeros(length(t_control), 7);
        
        for j = 1:7
            q_target_resampled(:, j) = interp1(traj_time, q_traj_target(:, j), t_control, 'pchip');
            qd_target_resampled(:, j) = interp1(traj_time, qd_traj_target(:, j), t_control, 'pchip');
        end
    else
        t_control = traj_time;
        q_target_resampled = q_traj_target;
        qd_target_resampled = qd_traj_target;
    end

    % Initialize simulation variables
    n_steps = length(t_control);
    q_sim = zeros(n_steps, 7);
    qd_sim = zeros(n_steps, 7);
    u_sim = zeros(n_steps, 7);

    % Set initial state to trajectory start
    robot.q = q_target_resampled(1, :)';
    robot.qd = zeros(7, 1);

    % Reset controller
    controller.resetIntegrators();

    % Run simulation
    for i = 1:n_steps
        % Get current state
        q_current = robot.q;
        qd_current = robot.qd;
        
        % Get target state at this time
        q_target = q_target_resampled(i, :)';
        qd_target = qd_target_resampled(i, :)';
        
        % Compute control input (with feed-forward from trajectory)
        u = controller.computeControl(q_target, q_current, qd_current, dt);
        
        % Add feed-forward term from trajectory acceleration (optional)
        if i < length(t_control)
            % Get mass matrix at current configuration
            [M, C, G] = robot.getDynamicsMatrices(q_current, qd_current);
            
            % Add feed-forward acceleration term (τ_ff = M * q̈_desired + C * q̇_desired + G)
            if i <= size(qdd_traj_target, 1)
                u_ff = M * qdd_traj_target(i, :)' + C * qd_target + G;
                u = u + 0.8 * u_ff;  % Scale factor to blend with feedback
            end
        end
        
        % Apply control and step forward
        [x_next, ~] = robot.step(u, dt);
        
        % Record trajectory
        q_sim(i, :) = q_current';
        qd_sim(i, :) = qd_current';
        u_sim(i, :) = u';
    end

    % Calculate end-effector positions
    ee_pos_target = zeros(n_steps, 3);
    ee_pos_actual = zeros(n_steps, 3);

    for i = 1:n_steps
        [pos_target, ~] = robot.getEndEffectorPose(q_target_resampled(i, :)');
        [pos_actual, ~] = robot.getEndEffectorPose(q_sim(i, :)');
        
        ee_pos_target(i, :) = pos_target';
        ee_pos_actual(i, :) = pos_actual';
    end

    % Calculate tracking error
    tracking_error = zeros(n_steps, 7);
    for i = 1:7
        tracking_error(:, i) = q_sim(:, i) - q_target_resampled(:, i);
    end

    position_error = zeros(n_steps, 3);
    for i = 1:3
        position_error(:, i) = ee_pos_actual(:, i) - ee_pos_target(:, i);
    end

    % Plot joint trajectory tracking
    figure('Name', ['Joint Trajectory Tracking: ', start_square, ' to ', end_square], 'Position', [100, 100, 1200, 800]);

    joint_names = {'θ₁', 'd₂', 'd₃', 'θ₄', 'θ₅', 'θ₆', 'θ₇'};
    colors = lines(7);

    for i = 1:7
        subplot(4, 2, i);
        hold on;
        plot(t_control, q_target_resampled(:, i), 'r--', 'LineWidth', 1.5);
        plot(t_control, q_sim(:, i), 'b-', 'LineWidth', 1.5);
        grid on;
        title(sprintf('Joint %s Trajectory', joint_names{i}));
        xlabel('Time (s)');
        if i <= 3
            ylabel('Position (rad or m)');
        else
            ylabel('Position (rad)');
        end
        legend('Target', 'Actual');
        
        % Add RMSE to the plot
        rmse = sqrt(mean((q_sim(:, i) - q_target_resampled(:, i)).^2));
        text(0.1, min(q_target_resampled(:, i)) + 0.8*(max(q_target_resampled(:, i))-min(q_target_resampled(:, i))), ...
             sprintf('RMSE: %.4f', rmse), 'FontSize', 8);
    end

    % Plot end-effector tracking
    figure('Name', ['End-Effector Tracking: ', start_square, ' to ', end_square], 'Position', [100, 100, 1200, 600]);

    % 3D trajectory
    subplot(2, 2, [1, 3]);
    plot3(ee_pos_target(:, 1), ee_pos_target(:, 2), ee_pos_target(:, 3), 'r--', 'LineWidth', 2);
    hold on;
    plot3(ee_pos_actual(:, 1), ee_pos_actual(:, 2), ee_pos_actual(:, 3), 'b-', 'LineWidth', 2);
    scatter3(ee_pos_target(1, 1), ee_pos_target(1, 2), ee_pos_target(1, 3), 100, 'r', 'filled');
    scatter3(ee_pos_target(end, 1), ee_pos_target(end, 2), ee_pos_target(end, 3), 100, 'r', 'filled');
    grid on;
    axis equal;
    title('End-Effector Trajectory');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    legend('Target', 'Actual', 'Start', 'End');

    % Position error
    subplot(2, 2, 2);
    plot(t_control, position_error, 'LineWidth', 1.5);
    grid on;
    title('End-Effector Position Error');
    xlabel('Time (s)');
    ylabel('Error (m)');
    legend('X', 'Y', 'Z');

    % Error norm
    subplot(2, 2, 4);
    error_norm = sqrt(sum(position_error.^2, 2));
    plot(t_control, error_norm, 'k-', 'LineWidth', 2);
    grid on;
    title('End-Effector Position Error Norm');
    xlabel('Time (s)');
    ylabel('Error (m)');
    
    % Display metrics for this move
    disp(['Move: ', start_square, ' to ', end_square]);
    disp(['Maximum joint tracking error: ', num2str(max(abs(tracking_error(:))), '%.4f'), ' rad/m']);
    disp(['RMS joint tracking error: ', num2str(sqrt(mean(mean(tracking_error.^2))), '%.4f'), ' rad/m']);
    disp(['Maximum end-effector position error: ', num2str(max(error_norm), '%.4f'), ' m']);
    disp(['RMS end-effector position error: ', num2str(sqrt(mean(error_norm.^2)), '%.4f'), ' m']);
    disp('------------------------------');
    
    % Visualize in 3D using the visualizeTrajectory function
    chess_params.show_board = true;
    chess_params.square_size = 0.06;
    chess_params.board_offset = [0.18; 0.24; 0];
    
    visualizeTrajectory(q_sim', qd_sim', [], t_control, [robot.L_45; robot.L_6], chess_params);
    title(sprintf('Chess Move: %s to %s', start_square, end_square));
    
    % Pause to view results
    disp('Press any key to continue to next move...');
    pause;
end