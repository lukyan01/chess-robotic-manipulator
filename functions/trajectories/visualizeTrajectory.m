function visualizeTrajectory(q_traj, qd_traj, qdd_traj, time_vec, link_lengths, chess_params)
% VISUALIZETRAJECTORY Interactive visualization of robot trajectory with velocity colormap
%
% Inputs:
%   q_traj       - Joint position trajectory, 7×N matrix
%   qd_traj      - Joint velocity trajectory, 7×N matrix (optional)
%   qdd_traj     - Joint acceleration trajectory, 7×N matrix (optional)
%   time_vec     - Time vector, 1×N
%   link_lengths - Link lengths [L_45, L_6]
%   chess_params - (Optional) Structure with chess visualization parameters
%
% Example:
%   visualizeTrajectory(q_traj, qd_traj, qdd_traj, time_vec, [0.121851; 0.3]);
%   visualizeTrajectory(q_traj, [], [], time_vec, [0.121851; 0.3]); % Without vel/accel

% Handle optional velocity and acceleration inputs
if nargin < 6
    chess_params = struct('show_board', true, ...
                          'square_size', 0.06, ...
                          'board_offset', [0.18; 0.24; 0]);
end

if isempty(qd_traj) || isempty(qdd_traj)
    % Calculate velocity and acceleration using central differences if not provided
    n_steps = size(q_traj, 2);
    dt = mean(diff(time_vec));
    
    if isempty(qd_traj)
        qd_traj = zeros(size(q_traj));
        for i = 2:n_steps-1
            qd_traj(:,i) = (q_traj(:,i+1) - q_traj(:,i-1)) / (2*dt);
        end
        qd_traj(:,1) = qd_traj(:,2);
        qd_traj(:,end) = qd_traj(:,end-1);
    end
    
    if isempty(qdd_traj)
        qdd_traj = zeros(size(q_traj));
        for i = 2:n_steps-1
            qdd_traj(:,i) = (qd_traj(:,i+1) - qd_traj(:,i-1)) / (2*dt);
        end
        qdd_traj(:,1) = qdd_traj(:,2);
        qdd_traj(:,end) = qdd_traj(:,end-1);
    end
end

dt_real = mean(diff(time_vec));
n_steps = size(q_traj, 2);

%% Create figure and layout
fig = figure('Name', 'Robot Trajectory Viewer', ...
             'Position', [50, 50, 1400, 900], 'WindowState', 'maximized');

% Create a layout with 3D view on left, joint plots on right
% 3D axes for robot visualization
ax1 = subplot(2, 3, [1,4], 'Parent', fig);
hold(ax1, 'on'); 
grid(ax1, 'on');
xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)'); zlabel(ax1, 'Z (m)');
title(ax1, 'Robot Configuration and Trajectory');
view(ax1, 3);
daspect(ax1, [1 1 1]);  % Set data aspect ratio to 1:1:1 (cube)
axis(ax1, 'equal');     % Ensure equal scaling on all axes

% Joint position plot
ax_pos = subplot(2, 3, 2, 'Parent', fig);
hold(ax_pos, 'on'); grid(ax_pos, 'on');
xlabel(ax_pos, 'Time (s)');
ylabel(ax_pos, 'Joint Position');
title(ax_pos, 'Joint Positions');

% Joint velocity plot
ax_vel = subplot(2, 3, 3, 'Parent', fig);
hold(ax_vel, 'on'); grid(ax_vel, 'on');
xlabel(ax_vel, 'Time (s)');
ylabel(ax_vel, 'Joint Velocity');
title(ax_vel, 'Joint Velocities');

% Joint acceleration plot
ax_acc = subplot(2, 3, 5, 'Parent', fig);
hold(ax_acc, 'on'); grid(ax_acc, 'on');
xlabel(ax_acc, 'Time (s)');
ylabel(ax_acc, 'Joint Acceleration');
title(ax_acc, 'Joint Accelerations');

% End-effector position plot
ax_ee = subplot(2, 3, 6, 'Parent', fig);
hold(ax_ee, 'on'); grid(ax_ee, 'on');
xlabel(ax_ee, 'Time (s)');
ylabel(ax_ee, 'Position (m)');
title(ax_ee, 'End-Effector Position');

%% Calculate end-effector trajectory and velocity
ee_traj = zeros(3, n_steps);
ee_orient = zeros(3, n_steps);
for i = 1:n_steps
    [pos, orient] = forwardKinematics(q_traj(:, i), link_lengths);
    ee_traj(:, i) = pos;
    ee_orient(:, i) = orient;
end

% Calculate end-effector velocity using gradient
ee_vel = zeros(3, n_steps);
for i = 1:3
    ee_vel(i,:) = gradient(ee_traj(i,:), time_vec);
end

% Calculate end-effector speed (magnitude of velocity)
ee_speed = sqrt(sum(ee_vel.^2, 1));

%% Draw static elements

% Draw chessboard
if chess_params.show_board
    drawChessboard(ax1, chess_params.square_size, chess_params.board_offset);
end

% Set up colors for each joint
colors = [
    0.8500, 0.3250, 0.0980;  % Red-orange
    0.0000, 0.4470, 0.7410;  % Blue
    0.4660, 0.6740, 0.1880;  % Green
    0.4940, 0.1840, 0.5560;  % Purple
    0.9290, 0.6940, 0.1250;  % Yellow
    0.3010, 0.7450, 0.9330;  % Cyan
    0.6350, 0.0780, 0.1840;  % Dark red
];
joint_names = {'θ₁', 'd₂', 'd₃', 'θ₄', 'θ₅', 'θ₆', 'θ₇'};

% Plot joint positions
for i = 1:7
    plot(ax_pos, time_vec, q_traj(i,:), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', joint_names{i});
end
legend(ax_pos, 'Location', 'best');

% Plot joint velocities
for i = 1:7
    plot(ax_vel, time_vec, qd_traj(i,:), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', joint_names{i});
end
legend(ax_vel, 'Location', 'best');

% Plot joint accelerations
for i = 1:7
    plot(ax_acc, time_vec, qdd_traj(i,:), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', joint_names{i});
end
legend(ax_acc, 'Location', 'best');

% Plot end-effector position
plot(ax_ee, time_vec, ee_traj(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
plot(ax_ee, time_vec, ee_traj(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
plot(ax_ee, time_vec, ee_traj(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
plot(ax_ee, time_vec, ee_speed, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Speed');
legend(ax_ee, 'Location', 'best');

% Plot 3D trajectory with velocity coloring
surface([ee_traj(1,:); ee_traj(1,:)], ...
        [ee_traj(2,:); ee_traj(2,:)], ...
        [ee_traj(3,:); ee_traj(3,:)], ...
        [ee_speed; ee_speed], ...
        'Parent', ax1, ...
        'FaceColor', 'none', ...
        'EdgeColor', 'interp', ...
        'LineWidth', 2);

% Add colorbar for velocity
cb = colorbar(ax1);
cb.Label.String = 'End-Effector Speed (m/s)';
colormap(ax1, jet);

%% Dynamic elements and animation controls

% Place holders for dynamic elements
robot_plot = gobjects(1);           % Robot configuration
current_pos_plot = gobjects(1);     % Current position marker
traj_plot = gobjects(1);            % Partial trajectory up to current time
time_markers = gobjects(4);         % Time markers on all plots

% Initial configuration
updateRobotConfig(1);

% Add slider for waypoint control
uicontrol(fig, 'Style', 'text', 'String', 'Trajectory Position:', ...
          'Units', 'normalized', 'Position', [0.1, 0.1, 0.15, 0.03], 'FontSize', 10);
      
slider = uicontrol('Style', 'slider', ...
    'Min', 1, 'Max', n_steps, 'Value', 1, ...
    'SliderStep', [1/(n_steps-1), max(10/(n_steps-1), 0.1)], ...
    'Units', 'normalized', 'Position', [0.1, 0.06, 0.25, 0.03], ...
    'Callback', @(src,~) updateRobotConfig(round(src.Value)));

time_label = uicontrol('Style', 'text', ...
    'Units', 'normalized', 'Position', [0.36, 0.06, 0.15, 0.03], ...
    'FontSize', 11, 'HorizontalAlignment', 'left', ...
    'String', sprintf('t = %.2f s', time_vec(1)));

% Add play/pause control
play_button = uicontrol('Style', 'pushbutton', ...
    'String', 'Play', ...
    'Units', 'normalized', 'Position', [0.1, 0.01, 0.1, 0.04], ...
    'Callback', @playAnimation);

% Animation control variables
anim_timer = timer;
anim_timer.TimerFcn = @animateRobot;
anim_timer.StopFcn = @stopAnimation;
anim_timer.Period = dt_real;  % Default animation speed
anim_timer.ExecutionMode = 'fixedRate';
current_idx = 1;
is_playing = false;

% Speed control slider
uicontrol(fig, 'Style', 'text', 'String', 'Animation Speed:', ...
          'Units', 'normalized', 'Position', [0.25, 0.01, 0.15, 0.03], 'FontSize', 10);
      
speed_slider = uicontrol('Style', 'slider', ...
    'Min', 0.25, 'Max', 3.0, 'Value', 1.0, ...          % ✅ Real-time default
    'SliderStep', [0.05, 0.2], ...
    'Units', 'normalized', ...
    'Position', [0.4, 0.01, 0.1, 0.03], ...
    'Callback', @updateSpeed);

%% --- Callback function to update the robot config
    function updateRobotConfig(idx)
        % Delete previous dynamic elements
        delete(robot_plot);
        delete(current_pos_plot);
        
        % Clear previous time markers
        for i = 1:length(time_markers)
            if ishghandle(time_markers(i))
                delete(time_markers(i));
            end
        end
        
        % Update current index
        current_idx = max(1, min(idx, n_steps));
        
        % Draw robot configuration at this time step
        hold(ax1, 'on');
        robot_plot = plotRobotConfig(ax1, q_traj(:,current_idx), link_lengths, 1.0);
        
        % Draw end-effector position marker
        current_pos_plot = scatter3(ax1, ee_traj(1,current_idx), ee_traj(2,current_idx), ee_traj(3,current_idx), ...
                              100, 'w', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
        
        % Add time markers to all the plots
        time_markers(1) = xline(ax_pos, time_vec(current_idx), 'r-', 'LineWidth', 2);
        time_markers(2) = xline(ax_vel, time_vec(current_idx), 'r-', 'LineWidth', 2);
        time_markers(3) = xline(ax_acc, time_vec(current_idx), 'r-', 'LineWidth', 2);
        time_markers(4) = xline(ax_ee, time_vec(current_idx), 'r-', 'LineWidth', 2);
        
        % Update slider and label
        slider.Value = current_idx;
        time_label.String = sprintf('t = %.2f s (Speed: %.2f m/s)', time_vec(current_idx), ee_speed(current_idx));
        
        drawnow;
    end

%% --- Animation functions
    function playAnimation(~, ~)
        if ~is_playing
            if current_idx >= n_steps
                current_idx = 1;  % Reset to beginning if at end
                updateRobotConfig(current_idx);
            end
            is_playing = true;
            play_button.String = 'Pause';
            start(anim_timer);
        else
            is_playing = false;
            play_button.String = 'Play';
            stop(anim_timer);
        end
    end

    function animateRobot(~, ~)
        current_idx = current_idx + 1;
        if current_idx > n_steps
            current_idx = 1;  % Loop back to beginning
        end
        updateRobotConfig(current_idx);
    end

    function stopAnimation(~, ~)
        is_playing = false;
        play_button.String = 'Play';
    end

    function updateSpeed(src, ~)
        speed_factor = src.Value;
        stop(anim_timer);
        anim_timer.Period = dt_real / speed_factor;
        if is_playing
            start(anim_timer);
        end
    end

%% Clean up timer when figure is closed
    set(fig, 'CloseRequestFcn', @closeFigure);
    
    function closeFigure(~, ~)
        try
            stop(anim_timer);
            delete(anim_timer);
        catch
            % Timer might already be deleted
        end
        delete(fig);
    end
end

function drawChessboard(ax, square_size, offset)
% Draw a chess board in the 3D plot using getSquareCoord function

% Draw chess squares
for rank = 1:8
    for file = 1:8
        % Convert to chess notation
        file_char = char('a' + file - 1);
        
        % Get square center coordinates
        [x, y] = getSquareCoord(file_char, rank);
        z = offset(3) + 0.001; % Slight offset to avoid z-fighting
        
        % Determine square color (alternating pattern)
        if mod(rank + file, 2) == 0
            color = [0.2, 0.2, 0.2]; % Dark square
        else
            color = [0.9, 0.9, 0.9]; % Light square
        end
        
        % Create square vertices
        square_x = [x-square_size/2, x+square_size/2, x+square_size/2, x-square_size/2, x-square_size/2];
        square_y = [y-square_size/2, y-square_size/2, y+square_size/2, y+square_size/2, y-square_size/2];
        square_z = z * ones(size(square_x));
        
        % Draw square
        fill3(ax, square_x, square_y, square_z, color, 'FaceAlpha', 0.8, 'EdgeColor', [0.5 0.5 0.5]);
        
        % Add labels only on the edges
        if file == 1  % a-file (left side)
            text(ax, x, y+square_size/2 + .005, z, num2str(rank), 'FontSize', 8, 'HorizontalAlignment', 'right');
        end
        if rank == 1  % 1st rank (bottom side)
            text(ax, x-square_size/2, y, z, file_char, 'FontSize', 8, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
        end
    end
end

% Add a small marker at the robot base for reference
plot3(ax, 0, 0, offset(3)+0.003, 'r.', 'MarkerSize', 15);
end



function h = plotRobotConfig(ax, q, link_lengths, alpha)
% Plot the robot configuration
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

% Plot robot links
h1 = plot3(ax, joints(1,:), joints(2,:), joints(3,:), 'k-', 'LineWidth', 2.5, 'Color', [0.2 0.2 0.8 alpha]);

% Plot joint markers
h_points = [];
for i = 1:size(joints,2)
    switch i
        case {1, 5, 6, 7, 8}  % Revolute joints and end effector
            h_tmp = scatter3(ax, joints(1,i), joints(2,i), joints(3,i), 100, ...
    [0.8 0.2 0.2], 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', alpha);
        case {3, 4}  % Prismatic joints
            h_tmp = scatter3(ax, joints(1,i), joints(2,i), joints(3,i), 100, ...
    [0.2 0.8 0.2], 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', alpha);
        otherwise
            h_tmp = scatter3(ax, joints(1,i), joints(2,i), joints(3,i), 50, ...
    [0 0 0], 'filled', 'MarkerFaceAlpha', alpha);
    end
    h_points = [h_points, h_tmp];
end

% Optionally plot the end-effector coordinate frame axes
axis_length = 0.05;
origin = T7(1:3,4);
x_axis = origin + axis_length * T7(1:3,1);
y_axis = origin + axis_length * T7(1:3,2);
z_axis = origin + axis_length * T7(1:3,3);

h2 = plot3(ax, [origin(1), x_axis(1)], [origin(2), x_axis(2)], [origin(3), x_axis(3)], 'r-', 'LineWidth', 2, 'Color', [1 0 0 alpha]);
h3 = plot3(ax, [origin(1), y_axis(1)], [origin(2), y_axis(2)], [origin(3), y_axis(3)], 'g-', 'LineWidth', 2, 'Color', [0 1 0 alpha]);
h4 = plot3(ax, [origin(1), z_axis(1)], [origin(2), z_axis(2)], [origin(3), z_axis(3)], 'b-', 'LineWidth', 2, 'Color', [0 0 1 alpha]);

% Return all plot handles
h = [h1; h_points(:); h2; h3; h4];
end