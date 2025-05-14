classdef ChessRobotModel < handle
    % CHESSROBOTMODEL Dynamic model of 7-DOF chess robot
    %
    % This class implements the full dynamic model of the 7-DOF chess robot
    % manipulator for simulation and control. It can be used with MATLAB's
    % control system design tools for PID tuning and other control methods.
    
    properties
        % Robot dimensions
        L_45 = 0.121851;  % Length of links 4 and 5 (m)
        L_6 = 0.3;        % Length of link 6 (m)
        
        % Physical parameters
        m = [0, 1.2, 1.0, 0.8, 0.8, 0.5, 0.2];  % Masses (kg)
        g = 9.81;                                  % Gravity (m/s^2)
        
        % Joint limits [min, max]
        joint_limits = [   
              -pi/4,  pi/4;     % theta1  - Base rotation           (-45°  to 45°)
               0.16,  0.48;     % d_horiz - Horizontal prismatic    (15cm  to 65cm)
               0.10,  0.25;     % d_vert  - Vertical prismatic      (10cm  to 25cm)
              pi/18,  4*pi/9;   % theta4  - Joint 4 rotation        ( 10°  to 80°)
            -8*pi/9,  -pi/9;    % theta5  - Joint 5 rotation        (-160° to -20°)
            -4*pi/9,  -pi/18;   % theta6  - Joint 6 rotation        (-80°  to -10°)
              -pi/4,  pi/4      % theta7  - Joint 7 rotation        (-45°  to 45°)
        ]; 
                     
        % Torque/force limits
        torque_limits = [10.0;   % theta_1 (Nm)
                         100.0;  % d_2 (N)
                         100.0;  % d_3 (N)
                         5.0;    % theta_4 (Nm)
                         5.0;    % theta_5 (Nm)
                         2.0;    % theta_6 (Nm)
                         1.0];   % theta_7 (Nm)
                     
        % State space dimensions
        n_joints = 7;      % Number of joints
        n_states = 14;     % 7 positions + 7 velocities
        n_inputs = 7;      % 7 joint torques/forces
        n_outputs = 14;    % Full state feedback
        
        % Current state
        q = zeros(7, 1);   % Joint positions
        qd = zeros(7, 1);  % Joint velocities
        
        % Simulation parameters
        dt = 0.001;        % Default time step for simulation (s)
    end
    
    methods
        function obj = ChessRobotModel()
            % Constructor
            % Initialize robot state to default home position
            obj.q = [0; 0.2; 0.2; pi/4; -pi/2; 0; 0];
            obj.qd = zeros(7, 1);
        end
        
        function [M, C, G] = getDynamicsMatrices(obj, q, qd)
            % Get dynamic matrices at current state
            % Inputs:
            %   q  - Joint positions (7x1)
            %   qd - Joint velocities (7x1)
            % Outputs:
            %   M - Mass matrix (7x7)
            %   C - Coriolis matrix (7x7)
            %   G - Gravity vector (7x1)
            
            % Call the generated functions
            try
                 % Expand mass arguments
                m = num2cell(obj.m);
        
                % Call generated functions with all required parameters
                M = robot_inertia(q, obj.L_45, obj.L_6, m{:});
                C = robot_coriolis(q, qd, obj.L_45, obj.L_6, m{:});
                G = robot_gravity(q, obj.g, obj.L_45, obj.L_6, m{:});
                
                % Substitute numeric values for symbolic parameters
                % Example: substitute links lengths, masses, etc.
                % [Implementation depends on how your generated functions handle parameters]
                
            catch ME
                warning('Error computing dynamics: %s', ME.message);
                M = eye(7);
                C = zeros(7);
                G = zeros(7, 1);
            end
        end
        
        function dxdt = dynamics(obj, t, x, u)
            % Continuous-time dynamics function for ODE solver
            % Implements: M(q)q̈ + C(q,q̇)q̇ + G(q) = τ
            %
            % Inputs:
            %   t - Current time
            %   x - State vector [q; qd] (14x1)
            %   u - Control input (torques/forces) (7x1)
            %
            % Outputs:
            %   dxdt - State derivatives [qd; qdd] (14x1)
            
            % Extract state components
            q = x(1:obj.n_joints);
            qd = x(obj.n_joints+1:end);
            
            % Apply joint limits
            for i = 1:obj.n_joints
                if q(i) < obj.joint_limits(i, 1)
                    q(i) = obj.joint_limits(i, 1);
                    if qd(i) < 0
                        qd(i) = 0;  % Stop at lower limit
                    end
                elseif q(i) > obj.joint_limits(i, 2)
                    q(i) = obj.joint_limits(i, 2);
                    if qd(i) > 0
                        qd(i) = 0;  % Stop at upper limit
                    end
                end
            end
            
            % Apply torque/force limits
            for i = 1:obj.n_joints
                u(i) = min(max(u(i), -obj.torque_limits(i)), obj.torque_limits(i));
            end
            
            % Get dynamics matrices
            [M, C, G] = obj.getDynamicsMatrices(q, qd);
            
            % Compute joint accelerations: qdd = M^-1 * (τ - C*qd - G)
            qdd = M \ (u - C*qd - G);
            
            % Return state derivatives
            dxdt = [qd; qdd];
        end
        
        function [x_next, y] = step(obj, u, dt)
            % Simulate one time step with given control input
            % Inputs:
            %   u  - Control input (torques/forces) (7x1)
            %   dt - Time step (s), optional
            %
            % Outputs:
            %   x_next - Next state [q; qd] (14x1)
            %   y      - Measured outputs (14x1, full state)
            
            if nargin < 3
                dt = obj.dt;  % Use default time step
            end
            
            % Current state
            x = [obj.q; obj.qd];
            
            % Integrate dynamics using RK4 method
            k1 = obj.dynamics(0, x, u);
            k2 = obj.dynamics(dt/2, x + dt*k1/2, u);
            k3 = obj.dynamics(dt/2, x + dt*k2/2, u);
            k4 = obj.dynamics(dt, x + dt*k3, u);
            
            x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
            
            % Update object state
            obj.q = x_next(1:obj.n_joints);
            obj.qd = x_next(obj.n_joints+1:end);
            
            % Full state observation
            y = x_next;
        end
        
        function [q, t] = simulate(obj, t_span, u_func)
            % Simulate the robot over a time span with a control function
            % Inputs:
            %   t_span - Time span [t_start, t_end] or time vector
            %   u_func - Control function handle: u = u_func(t, x)
            %
            % Outputs:
            %   q - Joint trajectory, one row per time step (time x 7)
            %   t - Time vector
            
            % Initialize
            if length(t_span) == 2
                t = t_span(1):obj.dt:t_span(2);
            else
                t = t_span;
            end
            
            n_steps = length(t);
            q = zeros(n_steps, obj.n_joints);
            qd = zeros(n_steps, obj.n_joints);
            
            % Set initial state
            q(1,:) = obj.q';
            qd(1,:) = obj.qd';
            
            % Simulate
            for i = 1:n_steps-1
                % Get control input
                x = [q(i,:)'; qd(i,:)'];
                u = u_func(t(i), x);
                
                % Step forward
                [x_next, ~] = obj.step(u, t(i+1) - t(i));
                
                % Store results
                q(i+1,:) = x_next(1:obj.n_joints)';
                qd(i+1,:) = x_next(obj.n_joints+1:end)';
            end
            
            % Reset robot state to initial
            obj.q = q(1,:)';
            obj.qd = qd(1,:)';
        end
        
        function ss_model = getLinearModel(obj, q_op, u_op)
            % Create a linearized state-space model around an operating point
            % Useful for control design and PID tuning
            % Inputs:
            %   q_op - Operating point joint positions (7x1)
            %   u_op - Operating point control inputs (7x1)
            %
            % Outputs:
            %   ss_model - State-space model object
            
            if nargin < 2
                q_op = obj.q;
            end
            if nargin < 3
                % Compute equilibrium input (gravity compensation)
                [~, ~, G] = obj.getDynamicsMatrices(q_op, zeros(7, 1));
                u_op = G;
            end
            
            % Linearize around operating point
            x_op = [q_op; zeros(7, 1)];  % Equilibrium at zero velocity
            
            % Define state perturbation for numerical linearization
            delta = 1e-6;
            
            % Compute A matrix (df/dx)
            A = zeros(obj.n_states, obj.n_states);
            for i = 1:obj.n_states
                x_perturb = x_op;
                x_perturb(i) = x_perturb(i) + delta;
                
                f_nominal = obj.dynamics(0, x_op, u_op);
                f_perturb = obj.dynamics(0, x_perturb, u_op);
                
                A(:, i) = (f_perturb - f_nominal) / delta;
            end
            
            % Compute B matrix (df/du)
            B = zeros(obj.n_states, obj.n_inputs);
            for i = 1:obj.n_inputs
                u_perturb = u_op;
                u_perturb(i) = u_perturb(i) + delta;
                
                f_nominal = obj.dynamics(0, x_op, u_op);
                f_perturb = obj.dynamics(0, x_op, u_perturb);
                
                B(:, i) = (f_perturb - f_nominal) / delta;
            end
            
            % Define C and D matrices for full state output
            C = eye(obj.n_states);
            D = zeros(obj.n_outputs, obj.n_inputs);
            
            % Create state-space model
            ss_model = ss(A, B, C, D);
        end
        
        function [pos, orient] = getEndEffectorPose(obj, q)
            % Compute end-effector position and orientation
            % Inputs:
            %   q - Joint positions (7x1), optional
            %
            % Outputs:
            %   pos    - End-effector position [x; y; z]
            %   orient - End-effector orientation (Euler angles)
            
            if nargin < 2
                q = obj.q;
            end
            
            % Call forwardKinematics function
            [pos, orient] = forwardKinematics(q, [obj.L_45; obj.L_6]);
        end
    end
    
    methods (Static)
        function params = getDefaultParams()
            % Return default parameter values for the model
            params.L_45 = 0.121851;  % Length of links 4 and 5 (m)
            params.L_6 = 0.1;        % Length of link 6 (m)
            params.m = [0, 1.2, 1.0, 0.8, 0.8, 0.5, 0.2];  % Masses (kg)
            params.g = 9.81;         % Gravity (m/s^2)
            
            % Joint friction coefficients (damping)
            params.b = [0.5, 5.0, 5.0, 0.2, 0.2, 0.1, 0.1];
        end
    end
end