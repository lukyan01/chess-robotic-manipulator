%% Chess Robot Demonstration - Chess Move Trajectory
% This script demonstrates a realistic chess piece move with visualization

%% Setup
clear;
close all;
clc;

% Add paths
addpath(genpath('../functions'));

% Robot parameters
link_lengths = [0.121851; 0.3];  % [L12, L3]
sample_time = 0.01;              % 100 Hz sampling

% Chess parameters
chess_params = struct('show_board', true, ...
                     'square_size', 0.06, ...
                     'board_offset', [0.18; 0.24; 0]);

%% Select a chess move
% Classic chess openings
openings = {
    'e2', 'e4', 'King''s Pawn Opening';
    'd2', 'd4', 'Queen''s Pawn Opening';
    'c2', 'c4', 'English Opening';
    'g1', 'f3', 'Réti Opening';
    'e2', 'e4', 'e7', 'e5', 'King''s Pawn Game';
    'e2', 'e4', 'e7', 'e6', 'French Defense';
    'e2', 'e4', 'c7', 'c5', 'Sicilian Defense';
    'd2', 'd4', 'd7', 'd5', 'Queen''s Pawn Game';
    'd2', 'd4', 'g8', 'f6', 'Indian Defense';
};

% Choose an opening
opening_idx = 1;  % King's Pawn Opening
move_duration = 4.0;  % seconds

start_square = openings{opening_idx, 1};
end_square = openings{opening_idx, 2};
opening_name = openings{opening_idx, 3};

fprintf('Demonstrating: %s (%s to %s)\n', opening_name, start_square, end_square);

%% Generate the trajectory
[q_traj, qd_traj, qdd_traj, time_vec] = generateChessTraj(start_square, end_square, link_lengths, move_duration, sample_time);

%% Visualize the trajectory
visualizeTrajectory(q_traj, time_vec, link_lengths, chess_params);
title(sprintf('Chess Move: %s to %s (%s)', start_square, end_square, opening_name));

%% Create animation
fprintf('Creating animation...\n');

% Setup animation figure
figure('Position', [100, 100, 1000, 750], 'Name', sprintf('Chess Move Animation: %s to %s', start_square, end_square));
ax = axes();
view(3);
axis equal;
grid on;
hold on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(sprintf('Chess Move: %s to %s (%s)', start_square, end_square, opening_name));

% Draw chess board
drawChessboard(ax, chess_params.square_size, chess_params.board_offset);

% Highlight source and destination squares
highlightChessSquare(ax, start_square, chess_params, [1, 0.8, 0.8]);  % Light red
highlightChessSquare(ax, end_square, chess_params, [0.8, 1, 0.8]);    % Light green

% Setup for animation
num_frames = length(time_vec);
play_rate = 1;  % 1 = real-time, >1 = faster

% Calculate end-effector positions for the full trajectory
ee_positions = zeros(3, num_frames);
for i = 1:num_frames
    [pos, ~] = forwardKinematics(q_traj(:, i), link_lengths);
    ee_positions(:, i) = pos;
end

% Create a trace of the end-effector path
plot3(ee_positions(1, :), ee_positions(2, :), ee_positions(3, :), 'b-', 'LineWidth', 1, 'Color', [0.5, 0.5, 1, 0.3]);

% Draw chess piece at first position
piece_height = 0.095;  % meters
[piece_handle, piece_coords] = drawChessPiece(ax, start_square, chess_params, 'p', 'w');

% Initial robot visualization
robot_links_handle = [];
robot_joints_handle = [];
[robot_links_handle, robot_joints_handle] = updateRobotVisualization(ax, q_traj(:, 1), link_lengths, [], []);

% Text display for time and joint values
time_text = text(0.1, 0.5, 0.6, sprintf('Time: %.2f / %.2f s', 0, time_vec(end)), 'FontSize', 12);

% Animation loop
fprintf('Animation starting...\n');
tic;
for i = 1:play_rate:num_frames
    % Update robot visualization
    [robot_links_handle, robot_joints_handle] = updateRobotVisualization(ax, q_traj(:, i), link_lengths, robot_links_handle, robot_joints_handle);
    
    % Move chess piece with the end-effector (simplified)
    if i > 1  % Don't move on first frame
        piece_coords = ee_positions(:, i);
        updateChessPiece(piece_handle, piece_coords);
    end
    
    % Update time display
    set(time_text, 'String', sprintf('Time: %.2f / %.2f s', time_vec(i), time_vec(end)));
    
    % Draw and pause
    drawnow;
    
    % Wait to maintain correct timing (adjusted for computation time)
    elapsed = toc;
    expected = time_vec(i) / play_rate;
    pause_time = max(0, expected - elapsed);
    pause(pause_time);
end

fprintf('Animation complete.\n');

%% Helper Functions

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
        
        fill3(ax, x, y, z, color, 'FaceAlpha', 0.8);
        
        % Add rank/file labels for the outermost squares
        if col == 1
            text(ax, x0-0.01, y0+square_size/2, z0, num2str(row), 'FontSize', 8);
        end
        if row == 1
            text(ax, x0+square_size/2, y0-0.01, z0, char('a' + col - 1), 'FontSize', 8);
        end
    end
end
end

function highlightChessSquare(ax, square, chess_params, color)
% Highlight a specific chess square
file = square(1);
rank = str2double(square(2));

col_idx = double(lower(file)) - double('a') + 1;
row_idx = rank;

square_size = chess_params.square_size;
offset = chess_params.board_offset;

% Calculate square corners
x0 = (col_idx-1) * square_size - offset(1);
y0 = (row_idx-1) * square_size - offset(2);
z0 = offset(3) + 0.002; % Slightly above the board

% Draw highlighted square
x = [x0, x0+square_size, x0+square_size, x0];
y = [y0, y0, y0+square_size, y0+square_size];
z = z0 * ones(size(x));

fill3(ax, x, y, z, color, 'FaceAlpha', 0.5, 'EdgeColor', 'r', 'LineWidth', 2);
end

function [piece_handle, coords] = drawChessPiece(ax, square, chess_params, piece_type, color)
% Draw a simplified chess piece
% piece_type: 'p'=pawn, 'r'=rook, 'n'=knight, 'b'=bishop, 'q'=queen, 'k'=king
% color: 'w'=white, 'b'=black

[x, y] = getSquareCoord(square(1), str2double(square(2)));
z = chess_params.board_offset(3);

% Piece dimensions based on type
piece_height = 0.095;  % Default height (meters)
base_radius = 0.02;    % Base radius (meters)
top_radius = 0.015;    % Top radius (meters)

% Adjust height based on piece type
switch lower(piece_type)
    case 'p' % Pawn
        piece_height = 0.075;
        top_radius = 0.010;
    case 'r' % Rook
        piece_height = 0.085;
    case 'n' % Knight
        piece_height = 0.085;
    case 'b' % Bishop
        piece_height = 0.090;
    case 'q' % Queen
        piece_height = 0.095;
    case 'k' % King
        piece_height = 0.095;
end

% Set piece color
if strcmpi(color, 'w')
    piece_color = [0.9 0.9 0.9]; % White
else
    piece_color = [0.2 0.2 0.2]; % Black
end

% Create a simple piece (cylinder with spherical top)
[x_cyl, y_cyl, z_cyl] = cylinder([base_radius, top_radius], 20);
z_cyl = z_cyl * piece_height;

% Translate to proper position
x_cyl = x_cyl + x;
y_cyl = y_cyl + y;
z_cyl = z_cyl + z;

% Draw the piece
piece_handle = surf(ax, x_cyl, y_cyl, z_cyl, 'FaceColor', piece_color, 'EdgeColor', 'none');

% Add a spherical top
[x_sphere, y_sphere, z_sphere] = sphere(15);
x_sphere = x_sphere * top_radius + x;
y_sphere = y_sphere * top_radius + y;
z_sphere = z_sphere * top_radius + z + piece_height;

top_handle = surf(ax, x_sphere, y_sphere, z_sphere, 'FaceColor', piece_color, 'EdgeColor', 'none');

% Combine handles and return
piece_handle = [piece_handle; top_handle];
coords = [x; y; z + piece_height];
end

function updateChessPiece(piece_handle, coords)
% Update the position of a chess piece
x = coords(1);
y = coords(2);
z = coords(3) - 0.095;  % Adjust for piece height

% Update cylinder position
x_data = get(piece_handle(1), 'XData');
y_data = get(piece_handle(1), 'YData');
z_data = get(piece_handle(1), 'ZData');

x_offset = x - mean(x_data(:));
y_offset = y - mean(y_data(:));
z_offset = z - min(z_data(:));

set(piece_handle(1), 'XData', x_data + x_offset);
set(piece_handle(1), 'YData', y_data + y_offset);
set(piece_handle(1), 'ZData', z_data + z_offset);

% Update sphere position
x_data = get(piece_handle(2), 'XData');
y_data = get(piece_handle(2), 'YData');
z_data = get(piece_handle(2), 'ZData');

x_offset = x - mean(x_data(:));
y_offset = y - mean(y_data(:));
z_offset = z - min(z_data(:)) + 0.095;  % Position at top of cylinder

set(piece_handle(2), 'XData', x_data + x_offset);
set(piece_handle(2), 'YData', y_data + y_offset);
set(piece_handle(2), 'ZData', z_data + z_offset);
end

function [links_handle, joints_handle] = updateRobotVisualization(ax, q, link_lengths, links_handle, joints_handle)
% Update or create visualization of robot configuration
% q - joint values [theta1, d_vert, d_horiz, theta4, theta5, theta6, theta7]
% link_lengths - [L_45, L_6]

% Extract joint values
theta1 = q(1);
d_vert = q(2);
d_horiz = q(3);
theta4 = q(4);
theta5 = q(5);
theta6 = q(6);
theta7 = q(7);

% Extract link lengths
L_45 = link_lengths(1);
L_6 = link_lengths(2);

% Rotation matrices
rotz = @(t) [cos(t), -sin(t), 0; sin(t), cos(t), 0; 0, 0, 1];
rotx = @(t) [1, 0, 0; 0, cos(t), -sin(t); 0, sin(t), cos(t)];

% Transform matrices (homogeneous coordinates)
H1to0 = [rotz(theta1) [0;0;0]; 0 0 0 1];
H2to1 = [rotz(-pi/2)* rotx(-pi/2) [0; 0; d_vert]; 0 0 0 1];
H3to2 = [rotx(pi/2) * rotz(pi/2) * rotx(pi/2) [0; 0; d_horiz]; 0 0 0 1];
H4to3 = [rotz(theta4) [L_45*cos(theta4); L_45*sin(theta4); 0]; 0 0 0 1];
H5to4 = [rotz(theta5) [L_45*cos(theta5); L_45*sin(theta5); 0]; 0 0 0 1];
H6to5 = [rotz(theta6) [L_6*cos(theta6); L_6*sin(theta6); 0]; 0 0 0 1];
H7to6 = [rotz(pi/2) * rotx(pi/2) * rotz(theta7) [0; 0; 0;]; 0 0 0 1];

% Calculate transformations to base frame
T0 = eye(4);
T1 = T0 * H1to0;
T2 = T1 * H2to1;
T3 = T2 * H3to2;
T4 = T3 * H4to3;
T5 = T4 * H5to4;
T6 = T5 * H6to5;
T7 = T6 * H7to6;

% Extract joint positions
joints = zeros(3,8);
joints(:,1) = T0(1:3,4);  % Base (frame 0)
joints(:,2) = T1(1:3,4);  % Joint 1
joints(:,3) = T2(1:3,4);  % Joint 2
joints(:,4) = T3(1:3,4);  % Joint 3
joints(:,5) = T4(1:3,4);  % Joint 4
joints(:,6) = T5(1:3,4);  % Joint 5
joints(:,7) = T6(1:3,4);  % Joint 6
joints(:,8) = T7(1:3,4);  % End effector

% Update or create robot links visualization
if isempty(links_handle)
    links_handle = plot3(ax, joints(1,:), joints(2,:), joints(3,:), ...
        'LineWidth', 3, 'Color', [0.2 0.2 0.8]);
else
    set(links_handle, 'XData', joints(1,:), 'YData', joints(2,:), 'ZData', joints(3,:));
end

% Update or create joint markers
joint_colors = {[0.8 0.2 0.2], [0.2 0.8 0.2], [0.2 0.8 0.2], ...
                [0.8 0.2 0.2], [0.8 0.2 0.2], [0.8 0.2 0.2], ...
                [0.8 0.2 0.2], [0.2 0.2 0.8]};
joint_sizes = [10, 10, 10, 10, 10, 10, 10, 12];

if isempty(joints_handle)
    joints_handle = gobjects(1, size(joints, 2));
    for i = 1:size(joints, 2)
        joints_handle(i) = plot3(ax, joints(1,i), joints(2,i), joints(3,i), 'o', ...
            'MarkerSize', joint_sizes(i), 'MarkerFaceColor', joint_colors{i}, ...
            'MarkerEdgeColor', 'k');
    end
else
    for i = 1:size(joints, 2)
        set(joints_handle(i), 'XData', joints(1,i), 'YData', joints(2,i), 'ZData', joints(3,i));
    end
end
end