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
    LE = links(1); L3 = links(2);

    %% Step 1: Compute theta1 and theta7
    theta1 = atan2(py_t, px_t);
    theta7 = theta1 - gz_t;

    %% Step 2: Compute theta_4, theta_5 elbow up values
    r = sqrt(px_t^2 + py_t^2);

    r_min = 0.212132;  % sqrt(0.21^2 + 0.03^2) [closest possible square d1]
    r_max = 0.664078;  % sqrt(0.21^2 + 0.63^2) [furthest possible square a8]
    dh_min = r_min - 2*LE*cos(4*pi/9);  % assuming theta4 in (10, 80)deg range
    dh_max = r_max - 2*LE*cos(pi/18);

    dh_necc = r - 2*LE*cos(pi/4); % we want to use theta_4 = pi/4, theta_5 = -pi/2 when possible
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
        theta4 = acos(r_elbow / (2 * LE));
        theta5 = -2 * theta4;
    end

    %% Step 3: Compute theta6 from constraint gy = theta4 + theta5 + theta6 = -pi/2
    theta6 = -pi/2 - theta4 - theta5;

    %% Step 4: Compute prismatic joint values
    d_vert  = pz_t - (LE*sin(theta4) + LE*sin(theta4 + theta5)) + L3;

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
      'L_12',     LE,      ...
      'L_3',      L3 ...
    );
end
