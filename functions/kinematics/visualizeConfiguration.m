function visualizeConfiguration(joint_vals, link_lengths, square_label)
% VISUALIZE_FRAMES  Draws the six joint frames of your manipulator and the chessboard

theta_1 = joint_vals(1);
d_2  = joint_vals(2);
d_3 = joint_vals(3);
theta_4 = joint_vals(4);
theta_5 = joint_vals(5);
theta_6 = joint_vals(6);
theta_7 = joint_vals(7);
L_45 = link_lengths(1);
L_3 = link_lengths(2);

% Base frame
T0 = eye(4);

T1 = T0 * [rotz(theta_1)                            [0;0;0];                                    0 0 0 1];
T2 = T1 * [rotz(-pi/2)* rotx(-pi/2)                 [0; 0; d_2];                             0 0 0 1];
T3 = T2 * [rotx(pi/2) * rotz(pi/2) * rotx(pi/2)     [0;                0; d_3];             0 0 0 1];
T4 = T3 * [rotz(theta_4)                            [L_45*cos(theta_4); L_45*sin(theta_4); 0];    0 0 0 1];
T5 = T4 * [rotz(theta_5)                            [L_45*cos(theta_5); L_45*sin(theta_5); 0];    0 0 0 1];
T6 = T5 * [rotz(theta_6)                            [L_3*cos(theta_6); L_3*sin(theta_6); 0];    0 0 0 1];
T7 = T6 * [rotz(pi/2) * rotx(pi/2) * rotz(theta_7)  [0; 0; 0;]; 0 0 0 1];

% collect all transforms
Ts = cat(3, T0, T1, T2, T3, T4, T5, T6, T7);

% plot them
figure;
hold on; grid on; axis equal;
colors = lines(size(Ts,3));

% Draw the chessboard
square_size = 0.06; % 6 cm squares
for row = 0:7
    for col = 0:7
        [x, y] = getSquareCoord(char('a' + col), row + 1);
        if mod(row + col, 2) == 0
            color = [0.2, 0.2, 0.2];  % dark square
        else
            color = [0.8, 0.8, 0.8];  % ligh square
        end
        rectangle('Position', [x - square_size/2, y - square_size/2, square_size, square_size], ...
                  'FaceColor', color, 'EdgeColor', 'k');
    end
end

% Plot robot frames
for i = 1:size(Ts,3)
    R = Ts(1:3,1:3,i);
    p = Ts(1:3,4,i)';
    q = rotm2quat(R);
    if i==1 || i==8
        plotTransforms(p, q, 'FrameSize', L_45/2);
        text(p(1),p(2),p(3), sprintf('F%d',i-1), 'FontSize',12, 'FontWeight','bold');
    end
end

% Set axes
xlabel('X'); ylabel('Y'); zlabel('Z');
view(0,90)  % YZ
title(sprintf('Configuration %s', square_label));
hold off;
end

%% Helper rotation matrices (you can also use the Robotics System Toolbox ones)
function R = rotz(t)
  R = [ cos(t) -sin(t) 0;
        sin(t)  cos(t) 0;
           0       0   1 ];
end

function R = rotx(t)
  R = [ 1    0       0;
        0  cos(t) -sin(t)  ;
        0  sin(t)  cos(t)];
end