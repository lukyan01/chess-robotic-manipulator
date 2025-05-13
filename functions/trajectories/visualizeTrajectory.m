function visualizeTrajectory(q_traj, time_vec, link_lengths, chess_params)
% VISUALIZETRAJECTORY Interactive visualization of robot trajectory
%
% Inputs:
%   q_traj       - Joint trajectory, 7×N matrix
%   time_vec     - Time vector, 1×N
%   link_lengths - Link lengths [L_45, L_6]
%   chess_params - (Optional) Structure with chess visualization parameters
%
% Example:
%   visualizeTrajectory(q_traj, time_vec, [0.121851; 0.3]);

% Default chess parameters if not provided
if nargin < 4
    chess_params = struct('show_board', true, ...
                          'square_size', 0.06, ...
                          'board_offset', [0.18; 0.24; 0]);
end

n_steps = size(q_traj, 2);

%% Create figure and layout
fig = figure('Name', 'Robot Trajectory Viewer', ...
             'Position', [100, 100, 1200, 800]);

% 3D axes for robot visualization
ax1 = axes('Parent', fig, 'Position', [0.05, 0.25, 0.55, 0.7]);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)'); zlabel(ax1, 'Z (m)');
title(ax1, 'Robot Configuration and Trajectory');
view(ax1, 3);

% Joint angles plot
ax2 = axes('Parent', fig, 'Position', [0.7, 0.55, 0.25, 0.4]);
hold(ax2, 'on'); grid(ax2, 'on');
xlabel(ax2, 'Time (s)');
ylabel(ax2, 'Joint Value');
title(ax2, 'Joint Positions');

% End-effector position plot
ax3 = axes('Parent', fig, 'Position', [0.7, 0.05, 0.25, 0.4]);
hold(ax3, 'on'); grid(ax3, 'on');
xlabel(ax3, 'Time (s)');
ylabel(ax3, 'Position (m)');
title(ax3, 'End-effector Position');

% Pre-calculate end-effector trajectory for plotting
ee_traj = zeros(3, n_steps);
ee_orient = zeros(3, n_steps);
for i = 1:n_steps
    [pos, orient] = forwardKinematics(q_traj(:, i), link_lengths);
    ee_traj(:, i) = pos;
    ee_orient(:, i) = orient;
end

% Plot full trajectory (static/dashed)
plot3(ax1, ee_traj(1, :), ee_traj(2, :), ee_traj(3, :), 'b--', 'LineWidth', 1);

% Plot joint trajectories (static)
colors = lines(7);
joint_names = {'θ₁', 'd₂', 'd₃', 'θ₄', 'θ₅', 'θ₆', 'θ₇'};
for i = 1:7
    plot(ax2, time_vec, q_traj(i,:), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', joint_names{i});
end
legend(ax2, 'Location', 'eastoutside');

% Plot end-effector position (static)
plot(ax3, time_vec, ee_traj(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
plot(ax3, time_vec, ee_traj(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
plot(ax3, time_vec, ee_traj(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
legend(ax3, 'Location', 'eastoutside');

% Draw chessboard
if chess_params.show_board
    drawChessboard(ax1, chess_params.square_size, chess_params.board_offset);
end

% Place holders for dynamic elements
robot_plot = gobjects(1);           % Robot configuration
current_pos_plot = gobjects(1);     % Current position marker
traj_plot = gobjects(1);            % Partial trajectory up to current time
time_marker1 = gobjects(1);         % Time marker on joint plot
time_marker2 = gobjects(1);         % Time marker on EE plot

% Initial configuration
updateRobotConfig(1);

% Add slider for waypoint control
uicontrol(fig, 'Style', 'text', 'String', 'Trajectory Position:', ...
          'Units', 'normalized', 'Position', [0.05, 0.15, 0.15, 0.03], 'FontSize', 10);
      
slider = uicontrol('Style', 'slider', ...
    'Min', 1, 'Max', n_steps, 'Value', 1, ...
    'SliderStep', [1/(n_steps-1), max(10/(n_steps-1), 0.1)], ...
    'Units', 'normalized', 'Position', [0.05, 0.1, 0.4, 0.03], ...
    'Callback', @(src,~) updateRobotConfig(round(src.Value)));

time_label = uicontrol('Style', 'text', ...
    'Units', 'normalized', 'Position', [0.45, 0.1, 0.15, 0.03], ...
    'FontSize', 11, 'HorizontalAlignment', 'left', ...
    'String', sprintf('t = %.2f s', time_vec(1)));

% Add play/pause control
play_button = uicontrol('Style', 'pushbutton', ...
    'String', 'Play', ...
    'Units', 'normalized', 'Position', [0.05, 0.05, 0.1, 0.04], ...
    'Callback', @playAnimation);

% Animation control variables
anim_timer = timer;
anim_timer.TimerFcn = @animateRobot;
anim_timer.StopFcn = @stopAnimation;
anim_timer.Period = 0.01;  % Default animation speed
anim_timer.ExecutionMode = 'fixedRate';
current_idx = 1;
is_playing = false;

% Speed control slider
uicontrol(fig, 'Style', 'text', 'String', 'Animation Speed:', ...
          'Units', 'normalized', 'Position', [0.2, 0.05, 0.15, 0.03], 'FontSize', 10);
      
speed_slider = uicontrol('Style', 'slider', ...
    'Min', 0.5, 'Max', 5, 'Value', 1, ...
    'SliderStep', [0.05, 0.2], ...
    'Units', 'normalized', 'Position', [0.35, 0.05, 0.1, 0.03], ...
    'Callback', @updateSpeed);

%% --- Callback function to update the robot config
    function updateRobotConfig(idx)
        % Delete previous elements
        delete(robot_plot);
        delete(current_pos_plot);
        delete(traj_plot);
        delete(time_marker1);
        delete(time_marker2);
        
        % Update current index
        current_idx = max(1, min(idx, n_steps));
        
        % Draw robot configuration at this time step
        hold(ax1, 'on');
        robot_plot = plotRobotConfig(ax1, q_traj(:,current_idx), link_lengths, 1.0);
        
        % Draw end-effector position marker
        current_pos_plot = scatter3(ax1, ee_traj(1,current_idx), ee_traj(2,current_idx), ee_traj(3,current_idx), ...
                              100, 'r', 'filled', 'MarkerEdgeColor', 'k');
        
        % Plot trajectory up to current point
        traj_plot = plot3(ax1, ee_traj(1,1:current_idx), ee_traj(2,1:current_idx), ee_traj(3,1:current_idx), ...
                         'g-', 'LineWidth', 2);
        
        % Add time markers to the graphs
        time_marker1 = xline(ax2, time_vec(current_idx), 'r-', 'LineWidth', 2);
        time_marker2 = xline(ax3, time_vec(current_idx), 'r-', 'LineWidth', 2);
        
        % Update slider and label
        slider.Value = current_idx;
        time_label.String = sprintf('t = %.2f s', time_vec(current_idx));
        
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
        anim_timer.Period = 0.05 / speed_factor;
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
% Draw a chess board in the 3D plot
board_size = 8 * square_size;

% Create the board base at z=0
x = [0, board_size, board_size, 0, 0];
y = [0, 0, board_size, board_size, 0];
z = zeros(size(x));

% Apply the offset
x = x - offset(1);
y = y - offset(2);
z = z + offset(3);

% Draw the board base
fill3(ax, x, y, z, [0.8 0.8 0.8], 'FaceAlpha', 0.5);

% Draw chess squares
for row = 1:8
    for col = 1:8
        % Determine square color (alternating pattern)
        if mod(row+col, 2) == 0
            color = [1 1 1]; % White
        else
            color = [0.3 0.3 0.3]; % Black
        end
        
        % Calculate square corners at z=0
        x0 = (col-1) * square_size - offset(1);
        y0 = (row-1) * square_size - offset(2);
        z0 = offset(3) + 0.001; % Slight offset to avoid z-fighting
        
        % Draw square
        x = [x0, x0+square_size, x0+square_size, x0, x0];
        y = [y0, y0, y0+square_size, y0+square_size, y0];
        z = z0 * ones(size(x));
        
        fill3(ax, x, y, z, color, 'FaceAlpha', 0.8);
        
        % Add rank/file labels for the outermost squares
        if col == 1
            text(ax, x0-0.01, y0+square_size/2, z0, num2str(row), 'FontSize', 8);
        end
        if row == 1
            text(ax, x0+square_size/2, y0-0.01, z0, char('a' + col - 1), 'FontSize', 8);
        end
    end
end

% Draw a thin line around the board edges for clarity
x = [0, board_size, board_size, 0, 0] - offset(1);
y = [0, 0, board_size, board_size, 0] - offset(2);
z = zeros(size(x)) + offset(3) + 0.002; % Slightly above the squares
plot3(ax, x, y, z, 'k-', 'LineWidth', 1);
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