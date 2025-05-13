function [q_traj, qd_traj, qdd_traj, time_vec] = generateChessTraj(start_square, end_square, link_lengths, duration, sample_time)
% GENERATECHESSTRAJ Generates a trajectory for chess piece movement
%
% Inputs:
%   start_square - Starting chess square (e.g., 'e2')
%   end_square   - Ending chess square (e.g., 'e4')
%   link_lengths - Link lengths [L_45, L_6]
%   duration     - Total duration of movement (seconds)
%   sample_time  - Sample time for trajectory points (seconds)
%
% Outputs:
%   q_traj       - Joint position trajectory, one column per time step
%   qd_traj      - Joint velocity trajectory, one column per time step
%   qdd_traj     - Joint acceleration trajectory, one column per time step
%   time_vec     - Time vector
%
% Example:
%   [q, qd, qdd, t] = generateChessTraj('e2', 'e4', [0.121851; 0.3], 5, 0.01);

% Chess piece dimensions
CHESS_PIECE_HEIGHT = 0.095;  % Height of chess piece (m)
LIFT_HEIGHT = 0.15;          % Additional height to lift piece above board (m)

% Validate inputs
validateattributes(duration, {'numeric'}, {'scalar', 'positive'}, 'generateChessTraj', 'duration');
validateattributes(sample_time, {'numeric'}, {'scalar', 'positive'}, 'generateChessTraj', 'sample_time');

% Parse chess squares
if ~ischar(start_square) || length(start_square) ~= 2
    error('Start square must be a string of length 2 (e.g., "e2")');
end
if ~ischar(end_square) || length(end_square) ~= 2
    error('End square must be a string of length 2 (e.g., "e4")');
end

start_file = start_square(1);
start_rank = str2double(start_square(2));
end_file = end_square(1);
end_rank = str2double(end_square(2));

% Verify valid chess square
valid_files = 'abcdefgh';
valid_ranks = 1:8;

if ~any(start_file == valid_files) || ~any(start_rank == valid_ranks)
    error('Invalid start square. File must be a-h, rank must be 1-8.');
end
if ~any(end_file == valid_files) || ~any(end_rank == valid_ranks)
    error('Invalid end square. File must be a-h, rank must be 1-8.');
end

% Compute robot frame coordinates for each square
[start_x, start_y] = getSquareCoord(start_file, start_rank);
[end_x, end_y] = getSquareCoord(end_file, end_rank);

% Define waypoints for chess piece movement
% 1. Start position (on source square)
% 2. Lifted position above source
% 3. Lifted position above destination
% 4. Final position (on destination square)

% Define the 4 waypoints in Cartesian space
waypoints = [
    % Position [x, y, z]                % Orientation [gx, gy, gz]
    start_x, start_y, CHESS_PIECE_HEIGHT,               0, 0, 0;    % At starting square
    start_x, start_y, CHESS_PIECE_HEIGHT + LIFT_HEIGHT, 0, 0, 0;    % Lifted above starting square
    end_x,   end_y,   CHESS_PIECE_HEIGHT + LIFT_HEIGHT, 0, 0, 0;    % Lifted above ending square
    end_x,   end_y,   CHESS_PIECE_HEIGHT,               0, 0, 0     % At ending square
];
% Define timing for each segment (fractions of total duration)
segment_durations = [0.2, 0.5, 0.3] * duration;  % 20% lift, 50% move, 30% place
segment_times = [0, cumsum(segment_durations)];  % Start times for each segment

% Compute IK for each waypoint
num_waypoints = size(waypoints, 1);
q_waypoints = zeros(7, num_waypoints);  % 7 joint values for each waypoint

for i = 1:num_waypoints
    target_pose = waypoints(i, :)';
    try
        % Calculate inverse kinematics
        [joint_vals, ~] = inverseKinematics(target_pose, link_lengths);
        q_waypoints(:, i) = joint_vals';
    catch ME
        error('Inverse kinematics failed at waypoint %d: %s', i, ME.message);
    end
end

% Generate full trajectory
full_traj_time = 0:sample_time:duration;
num_samples = length(full_traj_time);
q_traj = zeros(7, num_samples);
qd_traj = zeros(7, num_samples);
qdd_traj = zeros(7, num_samples);

% Generate trajectory for each segment
for i = 1:length(segment_durations)
    % Find time indices for this segment
    segment_start_time = segment_times(i);
    segment_end_time = segment_times(i+1);
    segment_indices = full_traj_time >= segment_start_time & full_traj_time <= segment_end_time;
    
    % Adjust time to be 0-based for this segment
    segment_time = full_traj_time(segment_indices) - segment_start_time;
    
    % Generate quintic trajectory for this segment
    [q_seg, qd_seg, qdd_seg, ~] = generateQuinticTraj(q_waypoints(:, i), q_waypoints(:, i+1), ...
        segment_durations(i), sample_time);
    
    % Store in the full trajectory
    q_traj(:, segment_indices) = q_seg;
    qd_traj(:, segment_indices) = qd_seg;
    qdd_traj(:, segment_indices) = qdd_seg;
end

time_vec = full_traj_time;

% Validate trajectory with forward kinematics
validateChessTraj(q_traj, waypoints, link_lengths);
end

function validateChessTraj(q_traj, waypoints, links)
% Validate the trajectory by checking key points with forward kinematics
% Check first, last, and middle points

checkpoints = [1, floor(size(q_traj, 2)/2), size(q_traj, 2)];

for i = 1:length(checkpoints)
    idx = checkpoints(i);
    waypoint_idx = min(size(waypoints, 1), max(1, round(idx / size(q_traj, 2) * size(waypoints, 1))));
    
    q = q_traj(:, idx);
    [pos, orient] = forwardKinematics(q, links);
    
    target_pos = waypoints(waypoint_idx, 1:3)';
    target_orient = waypoints(waypoint_idx, 4:6)';
    
    pos_error = norm(pos - target_pos);
    orient_error = norm(angleDiff(orient, target_orient));
    
    if pos_error > 1e-2 || orient_error > 1e-2
        warning('Large error at checkpoint %d: pos_error=%.4f m, orient_error=%.4f rad', ...
            idx, pos_error, orient_error);
    end
end
end

function diff = angleDiff(angle1, angle2)
% Returns the difference between two angles accounting for periodicity
diff = mod(angle1 - angle2 + pi, 2*pi) - pi;
end