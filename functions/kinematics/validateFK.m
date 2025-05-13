function error = validateFK(q_vals, target_pose, links)
% VALIDATEFK Validates inverse kinematics solution using forward kinematics
%
% Inputs:
%   q_vals     - Joint values from IK [theta_1, d_2, d_3, theta4, theta5, theta6, theta7]
%   target_pose - Target pose [x, y, z, gx, gy, gz]
%   links      - Link lengths [L_45, L_6]
%
% Outputs:
%   error - Structure containing position and orientation errors
%
% Example:
%   error = validateFK([pi/4, 0.3, 0.2, pi/6, -pi/3, pi/4, 0], [0.5, 0.5, 0.1, 0, -pi/2, 0], [0.121851; 0.3])

% Calculate forward kinematics
[fk_pos, fk_orient] = forwardKinematics(q_vals, links);

% Target position and orientation
target_pos = target_pose(1:3);
target_orient = target_pose(4:6);

% Calculate position error
pos_error = norm(fk_pos - target_pos);

% Calculate orientation error (angle between rotation matrices)
orient_error = norm(angleDiff(fk_orient, target_orient));

% Return error metrics
error = struct('position', pos_error, 'orientation', orient_error);

% Display error if it's significant
threshold = 1e-4;
if pos_error > threshold || orient_error > threshold
    fprintf('Position error: %.6f m\n', pos_error);
    fprintf('Orientation error: %.6f rad\n', orient_error);
    
    % Component-wise position errors
    fprintf('Position errors (x,y,z): [%.6f, %.6f, %.6f] m\n', ...
        fk_pos(1) - target_pos(1), ...
        fk_pos(2) - target_pos(2), ...
        fk_pos(3) - target_pos(3));
    
    % Component-wise orientation errors
    fprintf('Orientation errors (x,y,z): [%.6f, %.6f, %.6f] rad\n', ...
        angleDiff(fk_orient(1), target_orient(1)), ...
        angleDiff(fk_orient(2), target_orient(2)), ...
        angleDiff(fk_orient(3), target_orient(3)));
end
end

function diff = angleDiff(angle1, angle2)
% Returns the difference between two angles accounting for periodicity
diff = mod(angle1 - angle2 + pi, 2*pi) - pi;
end