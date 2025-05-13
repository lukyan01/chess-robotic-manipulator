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
end_square = 'h8';
move_duration = 2.0;  % seconds

fprintf('Generating trajectory for move: %s to %s\n', start_square, end_square);
[q_traj, qd_traj, qdd_traj, time_vec] = generateChessTraj(start_square, end_square, link_lengths, move_duration, sample_time);

% Visualize trajectory
visualizeTrajectory(q_traj, qd_traj, qdd_traj, time_vec, link_lengths, chess_params);
title(sprintf('Chess Move: %s to %s', start_square, end_square));

clear q_traj  qd_traj qdd_traj time_vec
%% Test 2: Check joint velocities and accelerations
moves = {
    'a1', 'a8', ...  
    'a8', 'h8', ...  
    'h8', 'h1', ...  
    'h1', 'a1' 
};
[q_traj, qd_traj, qdd_traj, time_vec] = generateChessSequence(moves, link_lengths, 1);
visualizeTrajectory(q_traj, qd_traj, qdd_traj, time_vec, link_lengths, chess_params);
title('Chess Sequence');