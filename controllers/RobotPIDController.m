classdef RobotPIDController < handle
    % ROBOTPIDCONTROLLER PID controller for the 7-DOF chess robot
    %
    % This class implements a configurable PID controller for the robot,
    % compatible with MATLAB's PID tuning tools. It can be used for:
    % - Joint space control
    % - Task space control
    % - Trajectory tracking
    
    properties
        % Controller configuration
        control_mode = 'joint';  % 'joint' or 'task'
        
        % PID gains for each joint
        Kp = zeros(7, 1);  % Proportional gains
        Ki = zeros(7, 1);  % Integral gains
        Kd = zeros(7, 1);  % Derivative gains
        
        % Task space PID gains (position)
        Kp_pos = diag([10, 10, 10]);  % Position gains
        Ki_pos = diag([0.1, 0.1, 0.1]);  % Position integral gains
        Kd_pos = diag([1, 1, 1]);  % Position derivative gains
        
        % Task space PID gains (orientation)
        Kp_orient = diag([5, 5, 5]);  % Orientation gains
        Ki_orient = diag([0.05, 0.05, 0.05]);  % Orientation integral gains
        Kd_orient = diag([0.5, 0.5, 0.5]);  % Orientation derivative gains
        
        % Control parameters
        dt = 0.001;            % Control sample time (s)
        max_integral = 10.0;   % Anti-windup limit
        
        % Controller state
        error_prev = zeros(7, 1);      % Previous error
        error_sum = zeros(7, 1);       % Integrated error
        
        % Task space error state
        pos_error_prev = zeros(3, 1);    % Previous position error
        pos_error_sum = zeros(3, 1);     % Integrated position error
        orient_error_prev = zeros(3, 1); % Previous orientation error
        orient_error_sum = zeros(3, 1);  % Integrated orientation error
        
        % Robot model
        robot = [];  % Reference to the robot model
    end
    
    methods
        function obj = RobotPIDController(robot_model)
            % Constructor
            % Input:
            %   robot_model - Reference to the robot model (ChessRobotModel)
            
            obj.robot = robot_model;
            
            % Initialize gains with default values for this particular robot
            obj.Kp = [10; 200; 200; 5; 5; 2; 1];  % Proportional gains
            obj.Ki = [1; 20; 20; 0.5; 0.5; 0.2; 0.1];  % Integral gains
            obj.Kd = [2; 40; 40; 1; 1; 0.4; 0.2];  % Derivative gains
        end
        
        function setGains(obj, Kp, Ki, Kd)
            % Set PID gains for joint space control
            % Inputs:
            %   Kp - Proportional gains (7x1)
            %   Ki - Integral gains (7x1)
            %   Kd - Derivative gains (7x1)
            
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            
            % Reset error states
            obj.error_prev = zeros(7, 1);
            obj.error_sum = zeros(7, 1);
        end
        
        function setTaskSpaceGains(obj, Kp_pos, Ki_pos, Kd_pos, Kp_orient, Ki_orient, Kd_orient)
            % Set PID gains for task space control
            % Inputs:
            %   Kp_pos, Ki_pos, Kd_pos - Position gains (3x3)
            %   Kp_orient, Ki_orient, Kd_orient - Orientation gains (3x3)
            
            if nargin > 1 && ~isempty(Kp_pos)
                obj.Kp_pos = Kp_pos;
            end
            
            if nargin > 2 && ~isempty(Ki_pos)
                obj.Ki_pos = Ki_pos;
            end
            
            if nargin > 3 && ~isempty(Kd_pos)
                obj.Kd_pos = Kd_pos;
            end
            
            if nargin > 4 && ~isempty(Kp_orient)
                obj.Kp_orient = Kp_orient;
            end
            
            if nargin > 5 && ~isempty(Ki_orient)
                obj.Ki_orient = Ki_orient;
            end
            
            if nargin > 6 && ~isempty(Kd_orient)
                obj.Kd_orient = Kd_orient;
            end
            
            % Reset task space error states
            obj.pos_error_prev = zeros(3, 1);
            obj.pos_error_sum = zeros(3, 1);
            obj.orient_error_prev = zeros(3, 1);
            obj.orient_error_sum = zeros(3, 1);
        end
        
        function setControlMode(obj, mode)
            % Set control mode ('joint' or 'task')
            % Input:
            %   mode - Control mode string
            
            valid_modes = {'joint', 'task'};
            
            if ~ismember(mode, valid_modes)
                error('Invalid control mode. Must be one of: %s', strjoin(valid_modes, ', '));
            end
            
            obj.control_mode = mode;
            
            % Reset error states
            obj.error_prev = zeros(7, 1);
            obj.error_sum = zeros(7, 1);
            obj.pos_error_prev = zeros(3, 1);
            obj.pos_error_sum = zeros(3, 1);
            obj.orient_error_prev = zeros(3, 1);
            obj.orient_error_sum = zeros(3, 1);
        end
        
        function u = computeControl(obj, q_target, q_current, qd_current, dt)
            % Compute control input based on current state and target
            % Inputs:
            %   q_target - Target joint positions (7x1) or task position/orientation [px;py;pz;ox;oy;oz]
            %   q_current - Current joint positions (7x1)
            %   qd_current - Current joint velocities (7x1)
            %   dt - Time step (optional, uses obj.dt if not provided)
            %
            % Outputs:
            %   u - Control input (torques/forces) (7x1)
            
            if nargin < 5
                dt = obj.dt;
            end
            
            if strcmp(obj.control_mode, 'joint')
                % Joint space PID control
                u = obj.computeJointSpaceControl(q_target, q_current, qd_current, dt);
            else
                % Task space PID control
                u = obj.computeTaskSpaceControl(q_target, q_current, qd_current, dt);
            end
        end
        
        function u = computeJointSpaceControl(obj, q_target, q_current, qd_current, dt)
            % Compute joint space PID control
            % Inputs:
            %   q_target - Target joint positions (7x1)
            %   q_current - Current joint positions (7x1)
            %   qd_current - Current joint velocities (7x1)
            %   dt - Time step
            %
            % Outputs:
            %   u - Control input (torques/forces) (7x1)
            
            % Calculate joint error
            error = q_target - q_current;
            
            % Update integral term with anti-windup
            obj.error_sum = obj.error_sum + error * dt;
            
            % Apply anti-windup limits
            obj.error_sum = min(max(obj.error_sum, -obj.max_integral), obj.max_integral);
            
            % Calculate derivative term (using measured velocity instead of error derivative)
            d_error = -qd_current;  % Velocity error (target is zero velocity at setpoint)
            
            % Calculate PID control law
            u_p = obj.Kp .* error;              % Proportional term
            u_i = obj.Ki .* obj.error_sum;      % Integral term
            u_d = obj.Kd .* d_error;            % Derivative term
            
            % Compute control input
            u = u_p + u_i + u_d;
            
            % Update previous error
            obj.error_prev = error;
            
            % Add gravity compensation
            [~, ~, G] = obj.robot.getDynamicsMatrices(q_current, qd_current);
            u = u + G;
        end
        
        function u = computeTaskSpaceControl(obj, target_pose, q_current, qd_current, dt)
            % Compute task space PID control using the Jacobian
            % Inputs:
            %   target_pose - Target position and orientation [px;py;pz;ox;oy;oz]
            %   q_current - Current joint positions (7x1)
            %   qd_current - Current joint velocities (7x1)
            %   dt - Time step
            %
            % Outputs:
            %   u - Control input (torques/forces) (7x1)
            
            % Get current end-effector pose
            [current_pos, current_orient] = obj.robot.getEndEffectorPose(q_current);
            
            % Split target into position and orientation
            target_pos = target_pose(1:3);
            target_orient = target_pose(4:6);
            
            % Calculate task space errors
            pos_error = target_pos - current_pos;
            orient_error = angleDiff(target_orient, current_orient);
            
            % Update integral terms with anti-windup
            obj.pos_error_sum = obj.pos_error_sum + pos_error * dt;
            obj.orient_error_sum = obj.orient_error_sum + orient_error * dt;
            
            % Apply anti-windup limits
            max_int = obj.max_integral;
            obj.pos_error_sum = min(max(obj.pos_error_sum, -max_int), max_int);
            obj.orient_error_sum = min(max(obj.orient_error_sum, -max_int), max_int);
            
            % Calculate task space PID control
            % Use current joint velocities transformed to task space for damping
            
            % Position control
            pos_p = obj.Kp_pos * pos_error;
            pos_i = obj.Ki_pos * obj.pos_error_sum;
            pos_d = -obj.Kd_pos * (current_pos - (current_pos - obj.pos_error_prev)/dt);
            
            % Orientation control
            orient_p = obj.Kp_orient * orient_error;
            orient_i = obj.Ki_orient * obj.orient_error_sum;
            orient_d = -obj.Kd_orient * (current_orient - (current_orient - obj.orient_error_prev)/dt);
            
            % Combine position and orientation control
            F_task = [pos_p + pos_i + pos_d; orient_p + orient_i + orient_d];
            
            % Compute Jacobian - analytical or numerical
            J = computeJacobian(obj.robot, q_current);
            
            % Map task forces to joint torques: τ = J^T * F_task
            u = J' * F_task;
            
            % Add gravity compensation
            [~, ~, G] = obj.robot.getDynamicsMatrices(q_current, qd_current);
            u = u + G;
            
            % Update previous errors
            obj.pos_error_prev = pos_error;
            obj.orient_error_prev = orient_error;
        end
        
        function pidTuner(obj, joint_index)
            % Open PID Tuner app for the specified joint
            % Inputs:
            %   joint_index - Index of joint to tune (1-7)
            
            if nargin < 2
                joint_index = 1;  % Default to first joint
            end
            
            % Get linearized model
            ss_model = obj.robot.getLinearModel();
            
            % Extract single-input, single-output model for the joint
            plant = ss_model(joint_index, joint_index);
            
            % Create PID controller for tuning
            pid_ctrl = pid(obj.Kp(joint_index), obj.Ki(joint_index), obj.Kd(joint_index));
            
            % Open PID Tuner
            pidTuner(plant, pid_ctrl);
        end
        
        function resetIntegrators(obj)
            % Reset all integrator states
            obj.error_sum = zeros(7, 1);
            obj.pos_error_sum = zeros(3, 1);
            obj.orient_error_sum = zeros(3, 1);
        end
    end
end

function diff = angleDiff(angle1, angle2)
    % Returns the difference between two angles accounting for periodicity
    diff = mod(angle1 - angle2 + pi, 2*pi) - pi;
end

function J = computeJacobian(robot, q)
    % Compute the robot Jacobian numerically
    % This maps joint velocities to end-effector velocities
    % J = [Jv; Jw] such that [v; w] = J * q_dot
    %
    % Inputs:
    %   robot - Robot model (ChessRobotModel)
    %   q - Joint positions (7x1)
    %
    % Outputs:
    %   J - Jacobian matrix (6x7)
    
    delta = 1e-6;  % Perturbation size
    
    % Initialize Jacobian
    J = zeros(6, 7);
    
    % Get baseline end-effector pose
    [pos0, orient0] = robot.getEndEffectorPose(q);
    x0 = [pos0; orient0];
    
    % Compute Jacobian columns by finite differences
    for i = 1:7
        q_perturb = q;
        q_perturb(i) = q_perturb(i) + delta;
        
        % Get perturbed end-effector pose
        [pos_i, orient_i] = robot.getEndEffectorPose(q_perturb);
        x_i = [pos_i; orient_i];
        
        % Compute partial derivatives
        J(:, i) = (x_i - x0) / delta;
    end
end