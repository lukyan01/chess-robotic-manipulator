%% Chess Robot Trajectory Test Script
% This script demonstrates the trajectory generation for a chess piece move
% and validates it using forward kinematics

%% Initialize
clear;
close all;
clc;

% Add paths and load parameters
addpath(genpath('../functions'));

% Robot parameters
link_lengths = [0.121851; 0.3];  % [L45, L6]
sample_time = 0.01;              % 100 Hz sampling

% Chess parameters
chess_params = struct('show_board', true, ...
                     'square_size', 0.06, ...
                     'board_offset', [0.18; 0.24; 0]);

%% Test 1: e2 to e4 - Classic chess opening move
start_square = 'a1';
end_square = 'h1';
move_duration = 2.0;  % seconds

fprintf('Generating trajectory for move: %s to %s\n', start_square, end_square);
[q_traj, qd_traj, qdd_traj, time_vec] = generateChessTraj(start_square, end_square, link_lengths, move_duration, sample_time);

% Visualize trajectory
visualizeTrajectory(q_traj, time_vec, link_lengths, chess_params);
title(sprintf('Chess Move: %s to %s', start_square, end_square));

%% Test 2: Check joint velocities and accelerations
figure('Name', 'Joint Velocities and Accelerations');

% Joint velocities
subplot(2,1,1);
plot(time_vec, qd_traj', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Joint Velocity');
title('Joint Velocities');
legend('θ₁', 'd₂', 'd₃', 'θ₄', 'θ₅', 'θ₆', 'θ₇', 'Location', 'best');

% Joint accelerations
subplot(2,1,2);
plot(time_vec, qdd_traj', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Joint Acceleration');
title('Joint Accelerations');
legend('θ₁', 'd₂', 'd₃', 'θ₄', 'θ₅', 'θ₆', 'θ₇', 'Location', 'best');

%% Test 3: Validate trajectory with forward kinematics
% Calculate end-effector position throughout trajectory
n_samples = size(q_traj, 2);
ee_pos = zeros(3, n_samples);
ee_orient = zeros(3, n_samples);

for i = 1:n_samples
    [pos, orient] = forwardKinematics(q_traj(:,i), link_lengths);
    ee_pos(:,i) = pos;
    ee_orient(:,i) = orient;
end

% Plot end-effector position
figure('Name', 'End-Effector Position');
plot(time_vec, ee_pos', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
title('End-Effector Position');
legend('X', 'Y', 'Z', 'Location', 'best');

% Plot 3D trajectory
figure('Name', 'End-Effector 3D Trajectory');
plot3(ee_pos(1,:), ee_pos(2,:), ee_pos(3,:), 'b-', 'LineWidth', 2);
grid on;
hold on;

% Draw start and end points
plot3(ee_pos(1,1), ee_pos(2,1), ee_pos(3,1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(ee_pos(1,end), ee_pos(2,end), ee_pos(3,end), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% Draw chess board
drawChessboard(gca, chess_params.square_size, chess_params.board_offset);

% Label axes
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(sprintf('End-Effector Trajectory: %s to %s', start_square, end_square));
view(3);
axis equal;

%% Test 4: Multiple chess piece moves
moves = {
    'a1', 'a8';  % Rook - longest file move
    'a1', 'h1';  % Longest rank move
    'a1', 'h8';  % Longest diagonal
    'e2', 'e4';  % Classic pawn opening
    'd7', 'd5';  % Classic pawn response
    'g1', 'f3';  % Knight move
};

% Create a figure to plot trajectories for all moves
figure('Name', 'Multiple Chess Moves', 'Position', [100, 100, 1200, 800]);
for i = 1:size(moves, 1)
    start_sq = moves{i,1};
    end_sq = moves{i,2};
    
    fprintf('Generating trajectory for move %d: %s to %s\n', i, start_sq, end_sq);
    [q_traj, ~, ~, time_vec] = generateChessTraj(start_sq, end_sq, link_lengths, move_duration, sample_time);
    
    % Calculate end-effector position
    n_samples = size(q_traj, 2);
    ee_pos = zeros(3, n_samples);
    for j = 1:n_samples
        [pos, ~] = forwardKinematics(q_traj(:,j), link_lengths);
        ee_pos(:,j) = pos;
    end
    
    % Plot in 3D space
    subplot(2, 3, i);
    plot3(ee_pos(1,:), ee_pos(2,:), ee_pos(3,:), 'LineWidth', 2);
    hold on;
    
    % Draw start and end points
    plot3(ee_pos(1,1), ee_pos(2,1), ee_pos(3,1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot3(ee_pos(1,end), ee_pos(2,end), ee_pos(3,end), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    
    % Draw simplified chess board
    drawChessboard(gca, chess_params.square_size, chess_params.board_offset);
    
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(sprintf('%s to %s', start_sq, end_sq));
    view(3);
    axis equal;
end

%% Helper function to draw chessboard
function drawChessboard(ax, square_size, offset)
% Draw a chess board in the 3D plot
board_size = 8 * square_size;

% Create the board base
x = [0, board_size, board_size, 0, 0] - offset(1);
y = [0, 0, board_size, board_size, 0] - offset(2);
z = zeros(size(x)) + offset(3);

fill3(ax, x, y, z, [0.8 0.8 0.8], 'FaceAlpha', 0.5);

% Draw chess squares
for row = 1:8
    for col = 1:8
        % Determine square color (alternating pattern)
        if mod(row+col, 2) == 0
            color = [1 1 1]; % White
        else
            color = [0.3 0.3 0.3]; % Black
        end
        
        % Calculate square corners
        x0 = (col-1) * square_size - offset(1);
        y0 = (row-1) * square_size - offset(2);
        z0 = offset(3) + 0.001; % Slight offset to avoid z-fighting
        
        % Draw square
        x = [x0, x0+square_size, x0+square_size, x0, x0];
        y = [y0, y0, y0+square_size, y0+square_size, y0];
        z = z0 * ones(size(x));
        
        fill3(ax, x, y, z, color, 'FaceAlpha', 0.5);
    end
end

% Draw board labels (files a-h, ranks 1-8)
for i = 1:8
    % File labels (a-h)
    text(ax, (i-0.5)*square_size - offset(1), -0.02 - offset(2), offset(3), ...
        char('a' + i - 1), 'HorizontalAlignment', 'center');
    
    % Rank labels (1-8)
    text(ax, -0.02 - offset(1), (i-0.5)*square_size - offset(2), offset(3), ...
        num2str(i), 'HorizontalAlignment', 'center');
end
end