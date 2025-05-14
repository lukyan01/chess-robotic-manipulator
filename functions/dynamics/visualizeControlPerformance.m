function visualizeControlPerformance(time_vec, q_des, q_dot_des, q_ddot_des, q_actual, q_dot_actual, q_ddot_actual, tau_history, link_lengths)
% VISUALIZECONTROLPERFORMANCE Visualizes controller performance
%
% Inputs:
%   time_vec     - Time vector
%   q_des        - Desired joint positions
%   q_dot_des    - Desired joint velocities
%   q_ddot_des   - Desired joint accelerations
%   q_actual     - Actual joint positions
%   q_dot_actual - Actual joint velocities
%   q_ddot_actual - Actual joint accelerations
%   tau_history  - Control torques
%   link_lengths - Robot link lengths

% Create figure
fig = figure('Name', 'PID Controller Performance', 'Position', [50, 50, 1200, 800]);

% Define joint names and colors
joint_names = {'θ₁', 'd₂', 'd₃', 'θ₄', 'θ₅', 'θ₆', 'θ₇'};
colors = [
    0.8500, 0.3250, 0.0980;  % Red-orange
    0.0000, 0.4470, 0.7410;  % Blue
    0.4660, 0.6740, 0.1880;  % Green
    0.4940, 0.1840, 0.5560;  % Purple
    0.9290, 0.6940, 0.1250;  % Yellow
    0.3010, 0.7450, 0.9330;  % Cyan
    0.6350, 0.0780, 0.1840;  % Dark red
];

% Position error subplot
subplot(3, 2, 1);
hold on; grid on;
position_error = q_des - q_actual;
for i = 1:7
    plot(time_vec, position_error(i,:), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', joint_names{i});
end
title('Position Error');
xlabel('Time (s)');
ylabel('Error');
legend('Location', 'best');

% Velocity error subplot
subplot(3, 2, 2);
hold on; grid on;
velocity_error = q_dot_des - q_dot_actual;
for i = 1:7
    plot(time_vec, velocity_error(i,:), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', joint_names{i});
end
title('Velocity Error');
xlabel('Time (s)');
ylabel('Error');
legend('Location', 'best');

% Control torque subplot
subplot(3, 2, 3);
hold on; grid on;
for i = 1:7
    plot(time_vec, tau_history(i,:), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', joint_names{i});
end
title('Control Torque/Force');
xlabel('Time (s)');
ylabel('Torque/Force');
legend('Location', 'best');

% Position tracking for selected joint
joint_to_show = 1; % Change to visualize different joint
subplot(3, 2, 4);
hold on; grid on;
plot(time_vec, q_des(joint_to_show,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Desired');
plot(time_vec, q_actual(joint_to_show,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Actual');
title(['Position Tracking - Joint ', joint_names{joint_to_show}]);
xlabel('Time (s)');
ylabel('Position');
legend('Location', 'best');

% End-effector position tracking
subplot(3, 2, 5:6);
hold on; grid on;

% Calculate end-effector position for desired and actual trajectories
ee_pos_des = zeros(3, length(time_vec));
ee_pos_actual = zeros(3, length(time_vec));

for i = 1:length(time_vec)
    [pos_des, ~] = forwardKinematics(q_des(:,i), link_lengths);
    [pos_actual, ~] = forwardKinematics(q_actual(:,i), link_lengths);
    
    ee_pos_des(:,i) = pos_des;
    ee_pos_actual(:,i) = pos_actual;
end

% Calculate end-effector tracking error
ee_error = sqrt(sum((ee_pos_des - ee_pos_actual).^2, 1));

% Plot 3D end-effector paths
plot3(ee_pos_des(1,:), ee_pos_des(2,:), ee_pos_des(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Desired Path');
plot3(ee_pos_actual(1,:), ee_pos_actual(2,:), ee_pos_actual(3,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Actual Path');
% Add markers for start and end positions
plot3(ee_pos_des(1,1), ee_pos_des(2,1), ee_pos_des(3,1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Start');
plot3(ee_pos_des(1,end), ee_pos_des(2,end), ee_pos_des(3,end), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'End');

title('End-Effector Trajectory Tracking');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(3); % 3D view
legend('Location', 'best');
grid on;
axis equal;

% Create a second figure for end-effector error
figure('Name', 'End-Effector Tracking Error', 'Position', [100, 100, 800, 400]);

% Plot end-effector tracking error over time
subplot(2,1,1);
plot(time_vec, ee_error, 'k-', 'LineWidth', 2);
title('End-Effector Position Error');
xlabel('Time (s)');
ylabel('Error (m)');
grid on;

% Plot end-effector position components
subplot(2,1,2);
hold on;
plot(time_vec, ee_pos_des(1,:) - ee_pos_actual(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'X Error');
plot(time_vec, ee_pos_des(2,:) - ee_pos_actual(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y Error');
plot(time_vec, ee_pos_des(3,:) - ee_pos_actual(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z Error');
title('End-Effector Component Errors');
xlabel('Time (s)');
ylabel('Error (m)');
legend('Location', 'best');
grid on;

% Display performance metrics
fprintf('Controller Performance Metrics:\n');
fprintf('Max End-Effector Error: %.4f m\n', max(ee_error));
fprintf('Mean End-Effector Error: %.4f m\n', mean(ee_error));
fprintf('RMS End-Effector Error: %.4f m\n', sqrt(mean(ee_error.^2)));

% Calculate and display maximum joint errors
max_pos_error = max(abs(position_error), [], 2);
fprintf('\nMaximum Position Errors:\n');
for i = 1:7
    if i == 1 || i >= 4
        % Angular joints (radians to degrees)
        fprintf('Joint %s: %.2f deg\n', joint_names{i}, rad2deg(max_pos_error(i)));
    else
        % Prismatic joints (meters)
        fprintf('Joint %s: %.4f m\n', joint_names{i}, max_pos_error(i));
    end
end
end