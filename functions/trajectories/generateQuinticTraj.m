function [q_traj, qd_traj, qdd_traj, time_vec] = generateQuinticTraj(q_start, q_end, duration, sample_time)
% GENERATEQUINTICTRAJ Generates a quintic polynomial trajectory
%
% Inputs:
%   q_start    - Starting position vector
%   q_end      - Ending position vector
%   duration   - Duration of the trajectory (seconds)
%   sample_time - Sample time for trajectory points (seconds)
%
% Outputs:
%   q_traj     - Position trajectory, one column per time step
%   qd_traj    - Velocity trajectory, one column per time step
%   qdd_traj   - Acceleration trajectory, one column per time step
%   time_vec   - Time vector
%
% Example:
%   [q, qd, qdd, t] = generateQuinticTraj([0;0;0], [1;1;1], 2, 0.01);

% Validate inputs
validateattributes(q_start, {'numeric'}, {'vector'}, 'generateQuinticTraj', 'q_start');
validateattributes(q_end, {'numeric'}, {'vector'}, 'generateQuinticTraj', 'q_end');
validateattributes(duration, {'numeric'}, {'scalar', 'positive'}, 'generateQuinticTraj', 'duration');
validateattributes(sample_time, {'numeric'}, {'scalar', 'positive'}, 'generateQuinticTraj', 'sample_time');

% Ensure column vectors
q_start = q_start(:);
q_end = q_end(:);

% Check that vectors are the same length
if length(q_start) ~= length(q_end)
    error('Start and end position vectors must have the same dimensions');
end

% Create time vector
time_vec = 0:sample_time:duration;
n_samples = length(time_vec);
n_joints = length(q_start);

% Initialize output arrays
q_traj = zeros(n_joints, n_samples);
qd_traj = zeros(n_joints, n_samples);
qdd_traj = zeros(n_joints, n_samples);

% Calculate quintic trajectory for each joint
for j = 1:n_joints
    q0 = q_start(j);
    qf = q_end(j);
    
    % Boundary conditions: zero velocity and acceleration at both endpoints
    qd0 = 0;
    qdf = 0;
    qdd0 = 0;
    qddf = 0;
    
    % Solve for quintic polynomial coefficients
    % s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    
    % Initial position, velocity, acceleration
    a0 = q0;
    a1 = qd0;
    a2 = qdd0/2;
    
    % Final position, velocity, acceleration constraints
    T = duration;
    A = [  T^3,   T^4,    T^5;
         3*T^2, 4*T^3,  5*T^4;
         6*T,  12*T^2, 20*T^3];
    
    b = [qf - q0 - qd0*T - qdd0*T^2/2;
         qdf - qd0 - qdd0*T;
         qddf - qdd0];
    
    % Solve for a3, a4, a5
    x = A\b;
    a3 = x(1);
    a4 = x(2);
    a5 = x(3);
    
    % Generate trajectory points
    for i = 1:n_samples
        t = time_vec(i);
        
        % Position
        q_traj(j,i) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
        
        % Velocity
        qd_traj(j,i) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
        
        % Acceleration
        qdd_traj(j,i) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
    end
end
end