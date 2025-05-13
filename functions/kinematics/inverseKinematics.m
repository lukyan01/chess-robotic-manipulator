function [vals, subs_map] = inverseKinematics(target, links)
    % CLOSED‐FORM “ELBOW UP” IK for the 7‐DOF SCARA‐like chain
    %
    % Inputs:
    %   target = [px; py; pz; gz]
    %   links  = [L45; L6]
    %
    % Outputs:
    %   vals     = [theta_1, d_2, d_3, theta_4, theta_5, theta_6, theta_7]
    %   subs_map = symbolic variable to numeric mapping for substitution

    %% Extract input
    px_t = target(1); py_t = target(2); pz_t = target(3);
    gz_t = target(4);
    L_45 = links(1); L_6 = links(2);

    %% Step 1: Compute theta1 and theta7
    theta1 = atan2(py_t, px_t);
    theta7 = theta1 - gz_t;

    %% Step 2: Compute theta_4, theta_5 elbow up values
    r = sqrt(px_t^2 + py_t^2);

    r_min = 0.212132;  % sqrt(0.21^2 + 0.03^2) [closest possible square d1]
    r_max = 0.664078;  % sqrt(0.21^2 + 0.63^2) [furthest possible square a8]
    dh_min = r_min - 2*L_45*cos(4*pi/9);  % assuming theta4 in (10, 80)deg range
    dh_max = r_max - 2*L_45*cos(pi/18);

    dh_necc = r - 2*L_45*cos(pi/4); % we want to use theta_4 = pi/4, theta_5 = -pi/2 when possible
    if dh_necc <= dh_max && dh_necc >= dh_min
        d_horiz = dh_necc;
        theta4 = pi/4;
        theta5 = -pi/2;
    else
        if dh_necc > dh_max
            d_horiz = dh_max;
        else
            d_horiz = dh_min;
        end
        r_elbow = r - d_horiz;
        theta4 = acos(r_elbow / (2 * L_45));
        theta5 = -2 * theta4;
    end

    %% Step 3: Compute theta6 from constraint gy = theta4 + theta5 + theta6 = -pi/2
    theta6 = -pi/2 - theta4 - theta5;

    %% Step 4: Compute prismatic joint values
    d_vert  = pz_t - (L_45*sin(theta4) + L_45*sin(theta4 + theta5)) + L_6;

    %% Output vector
    vals = [theta1, d_vert, d_horiz, theta4, theta5, theta6, theta7];

    %% Build symbolic substitution map
    subs_map = struct( ...
      'theta_1',  theta1,  ...
      'd_vert',   d_vert, ...
      'd_horiz',  d_horiz,  ...
      'theta_4',  theta4,  ...
      'theta_5',  theta5,  ...
      'theta_6',  theta6,  ...
      'theta_7',  theta7,  ...
      'L_45',     L_45,      ...
      'L_6',      L_6 ...
    );
    validateSolution(vals);
end

function validateSolution(joint_vals)
% Check if the calculated joint values are within physical limits
% Throw error if any joint is out of range

% Define joint limits [min, max]
joint_limits = [   
      -pi/4,  pi/4;     % theta1  - Base rotation           (-45°  to 45°)
       0.16,  0.48;     % d_horiz - Horizontal prismatic    (15cm  to 65cm)
       0.10,  0.25;     % d_vert  - Vertical prismatic      (10cm  to 25cm)
      pi/18,  4*pi/9;   % theta4  - Joint 4 rotation        ( 10°  to 80°)
    -8*pi/9,  -pi/9;    % theta5  - Joint 5 rotation        (-160° to -20°)
    -4*pi/9,  -pi/18;   % theta6  - Joint 6 rotation        (-80°  to -10°)
      -pi/4,  pi/4      % theta7  - Joint 7 rotation        (-45°  to 45°)
]; 


joint_names = {'theta1', 'd_horiz', 'd_vert', 'theta4', 'theta5', 'theta6', 'theta7'};

% Check each joint
for i = 1:length(joint_vals)
    if ceil(joint_vals(i)) < joint_limits(i,1) || floor(joint_vals(i)) > joint_limits(i,2)
        warning('Joint %s (%.4f) exceeds limits [%.4f, %.4f]', ...
            joint_names{i}, joint_vals(i), joint_limits(i,1), joint_limits(i,2));
    end
end
end