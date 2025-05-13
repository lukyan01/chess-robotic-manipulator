function [q_traj, qd_traj, qdd_traj, time_vec] = generateLinearTraj(start_pos, end_pos, duration, sample_time, link_lengths)
% GENERATELINEARTRAJ Generates a straight-line trajectory in Cartesian space
%
% Generates a trajectory where the end-effector follows a straight line
% between two points in Cartesian space. The trajectory is parameterized
% with a quintic polynomial to ensure smooth acceleration profiles.
%
% Inputs:
%   start_pos   - Starting position [x; y; z; gx; gy; gz]
%   end_pos     - Ending position [x; y; z; gx; gy; gz]
%   duration    - Duration of the trajectory (seconds)
%   sample_time - Sample time for trajectory points (seconds)
%   link_lengths - Link lengths [L_12, L_3]
%
% Outputs:
%   q_traj     - Joint position trajectory, one column per time step
%   qd_traj    - Joint velocity trajectory, one column per time step
%   qdd_traj   - Joint acceleration trajectory, one column per time step
%   time_vec   - Time vector
%
% Example:
%   [q, qd, qdd, t] = generateLinearTraj([0.3;0.2;0.1;0;-pi/2;0], 
%                                        [0.4;0.3;0.2;0;-pi/2;0], 
%                                        2, 0.01, [0.121851; 0.3]);

% Validate inputs
validateattributes(start_pos, {'numeric'}, {'vector', 'numel', 6}, 'generateLinearTraj', 'start_pos');
validateattributes(end_pos, {'numeric'}, {'vector', 'numel', 6}, 'generateLinearTraj', 'end_pos');
validateattributes(duration, {'numeric'}, {'scalar', 'positive'}, 'generateLinearTraj', 'duration');
validateattributes(sample_time, {'numeric'}, {'scalar', 'positive'}, 'generateLinearTraj', 'sample_time');
validateattributes(link_lengths, {'numeric'}, {'vector', 'numel', 2}, 'generateLinearTraj', 'link_lengths');

% Ensure column vectors
start_pos = start_pos(:);
end_pos = end_pos(:);

% Create time vector
time_vec = 0:sample_time:duration;
n_samples = length(time_vec);

% Interpolate in Cartesian space using quintic polynomial for smooth motion
% Extract position and orientation components
start_p = start_pos(1:3);  % Position
start_o = start_pos(4:6);  % Orientation

end_p = end_pos(1:3);      % Position
end_o = end_pos(4:6);      % Orientation

% Generate quintic trajectory for position
[p_traj, pd_traj, pdd_traj] = quinticPolynomial(start_p, end_p, duration, time_vec);

% Generate quintic trajectory for orientation
[o_traj, od_traj, odd_traj] = quinticPolynomial(start_o, end_o, duration, time_vec);

% Compute inverse kinematics for each point along the trajectory
q_traj = zeros(7, n_samples);
qd_traj = zeros(7, n_samples);
qdd_traj = zeros(7, n_samples);

% Define small time delta for numerical derivatives if needed
dt = sample_time;

% Calculate IK for each point in the trajectory
for i = 1:n_samples
    % Current Cartesian pose
    current_pose = [p_traj(:,i); o_traj(:,i)];
    
    % Solve inverse kinematics
    try
        [joint_vals, ~] = inverseKinematics(current_pose, link_lengths);
        q_traj(:,i) = joint_vals';
    catch ME
        warning('Inverse kinematics failed at point %d: %s', i, ME.message);
        
        % If first point fails, we can't continue
        if i == 1
            error('Inverse kinematics failed at starting point. Trajectory cannot be generated.');
        end
        
        % Otherwise use previous valid value (not ideal but keeps simulation running)
        q_traj(:,i) = q_traj(:,i-1);
    end
    
    % Calculate joint velocities and accelerations
    if i > 1
        % Calculate numerical joint velocity
        qd_traj(:,i-1) = (q_traj(:,i) - q_traj(:,i-1)) / dt;
        
        % Calculate numerical joint acceleration for middle points
        if i > 2
            qdd_traj(:,i-2) = (qd_traj(:,i-1) - qd_traj(:,i-2)) / dt;
        end
    end
end

% Fill in the last velocity and acceleration values
if n_samples > 1
    qd_traj(:,end) = qd_traj(:,end-1);
    qdd_traj(:,end-1) = qdd_traj(:,end-2);
    qdd_traj(:,end) = qdd_traj(:,end-1);
end

% Filter velocities and accelerations to smooth out numerical noise
if n_samples > 5
    for j = 1:7
        qd_traj(j,:) = smoothdata(qd_traj(j,:), 'gaussian', 5);
        qdd_traj(j,:) = smoothdata(qdd_traj(j,:), 'gaussian', 5);
    end
end

% Validate trajectory with forward kinematics
validateTrajectory(q_traj, time_vec, p_traj, o_traj, link_lengths);
end

function [p_traj, pd_traj, pdd_traj] = quinticPolynomial(p0, pf, duration, time_vec)
% Generates quintic polynomial trajectories for each component

% Ensure column vectors
p0 = p0(:);
pf = pf(:);

n_dims = length(p0);
n_samples = length(time_vec);

% Initialize trajectories
p_traj = zeros(n_dims, n_samples);
pd_traj = zeros(n_dims, n_samples);
pdd_traj = zeros(n_dims, n_samples);

% Generate quintic polynomial for each dimension
for d = 1:n_dims
    % Initial and final values
    x0 = p0(d);
    xf = pf(d);
    
    % Boundary conditions (zero velocity and acceleration at endpoints)
    xd0 = 0; xdf = 0;
    xdd0 = 0; xddf = 0;
    
    % Solve for polynomial coefficients
    T = duration;
    a0 = x0;
    a1 = xd0;
    a2 = xdd0/2;
    
    A = [  T^3,   T^4,   T^5;
         3*T^2, 4*T^3, 5*T^4;
         6*T,  12*T^2, 20*T^3];
    
    b = [xf - x0 - xd0*T - xdd0*T^2/2;
         xdf - xd0 - xdd0*T;
         xddf - xdd0];
    
    x = A\b;
    a3 = x(1);
    a4 = x(2);
    a5 = x(3);
    
    % Evaluate the polynomial at each time point
    for i = 1:n_samples
        t = time_vec(i);
        
        % Position
        p_traj(d,i) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
        
        % Velocity
        pd_traj(d,i) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
        
        % Acceleration
        pdd_traj(d,i) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
    end
end
end

function validateTrajectory(q_traj, time_vec, p_traj, o_traj, link_lengths)
% Validate trajectory by checking forward kinematics against desired cartesian path

% Calculate forward kinematics at several points
check_indices = [1, round(length(time_vec)/2), length(time_vec)];
max_pos_error = 0;
max_orient_error = 0;

for i = 1:length(check_indices)
    idx = check_indices(i);
    
    % Calculate forward kinematics
    [pos, orient] = forwardKinematics(q_traj(:,idx), link_lengths);
    
    % Compare with desired Cartesian position
    pos_error = norm(pos - p_traj(:,idx));
    orient_error = norm(angleDiff(orient, o_traj(:,idx)));
    
    max_pos_error = max(max_pos_error, pos_error);
    max_orient_error = max(max_orient_error, orient_error);
    
    % Output warning if error is significant
    if pos_error > 1e-3
        warning('Position error at t=%.2f: %.4f m', time_vec(idx), pos_error);
    end
    if orient_error > 1e-3
        warning('Orientation error at t=%.2f: %.4f rad', time_vec(idx), orient_error);
    end
end

fprintf('Maximum position error: %.4f m\n', max_pos_error);
fprintf('Maximum orientation error: %.4f rad\n', max_orient_error);
end

function diff = angleDiff(angle1, angle2)
% Returns the difference between two angles accounting for periodicity
diff = mod(angle1 - angle2 + pi, 2*pi) - pi;
end