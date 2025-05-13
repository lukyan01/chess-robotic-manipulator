function [q_traj, qd_traj, qdd_traj, time_vec] = generateChessSequence(moves, link_lengths, duration_per_move, sample_time)
% GENERATECHESSSEQUENCE Generates trajectory for a sequence of chess moves
%
% Inputs:
%   moves            - Cell array of move pairs, e.g., {'e2','e4', 'd7','d5', ...}
%                      Format: alternating start and end squares
%   link_lengths     - Link lengths [L_45, L_6]
%   duration_per_move - Duration of each move in seconds (default: 5)
%   sample_time      - Sample time for trajectory points (default: 0.05)
%
% Outputs:
%   q_traj           - Joint position trajectory, one column per time step
%   qd_traj          - Joint velocity trajectory, one column per time step
%   qdd_traj         - Joint acceleration trajectory, one column per time step
%   time_vec         - Time vector for the entire sequence
%
% Example:
%   [q, qd, qdd, t] = generateChessSequence({'e2','e4','d7','d5','e4','d5'}, [0.121851; 0.3], 5, 0.05);

% Default parameters
if nargin < 3
    duration_per_move = 1;  % 5 seconds per move
end
if nargin < 4
    sample_time = 0.01;    % 50ms sample time (20Hz)
end

% Validate inputs
if mod(length(moves), 2) ~= 0
    error('Moves array must contain pairs of squares (start and end for each move)');
end

num_moves = length(moves) / 2;
fprintf('Generating trajectories for %d chess moves...\n', num_moves);

% Initialize trajectory arrays
q_traj = [];
qd_traj = [];
qdd_traj = [];
time_vec = [];

% Process each move
total_duration = 0;
for i = 1:num_moves
    start_square = moves{2*i-1};
    end_square = moves{2*i};
    
    fprintf('Move %d: %s to %s\n', i, start_square, end_square);
    
    % Generate trajectory for this move
    [q_move, qd_move, qdd_move, t_move] = generateChessTraj(start_square, end_square, link_lengths, duration_per_move, sample_time);
    
    % Adjust time vector to be continuous
    if ~isempty(time_vec)
        t_move = t_move + time_vec(end);
    end
    
    % Concatenate with full trajectory
    q_traj = [q_traj, q_move];
    qd_traj = [qd_traj, qd_move];
    qdd_traj = [qdd_traj, qdd_move];
    time_vec = [time_vec, t_move];
    
    % Update total duration
    total_duration = total_duration + duration_per_move;
end

fprintf('Total trajectory duration: %.2f seconds\n', total_duration);
fprintf('Trajectory size: %d time steps, %d joints\n', size(q_traj, 2), size(q_traj, 1));
end