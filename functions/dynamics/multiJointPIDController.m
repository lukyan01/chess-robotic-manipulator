function [tau, error_states] = multiJointPIDController(q, q_dot, q_des, q_dot_des, q_ddot_des, dt, pid_params, prev_error_states, model_params)
    % Extract model parameters
    L_45 = model_params.L_45;
    L_6 = model_params.L_6;
    m1 = model_params.m1;
    m2 = model_params.m2;
    m3 = model_params.m3;
    m4 = model_params.m4;
    m5 = model_params.m5;
    m6 = model_params.m6;
    m7 = model_params.m7;
    g = model_params.g;
    
    % Number of joints
    num_joints = 7;
    
    % Initialize outputs
    error_states = cell(num_joints, 1);
    
    % IMPORTANT CHANGE: First calculate inverse dynamics components
    M = robot_inertia(q, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
    C = robot_coriolis(q, q_dot, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
    G = robot_gravity(q, g, L_45, L_6, m1, m2, m3, m4, m5, m6, m7);
    
    % Calculate position and velocity errors
    e = q_des - q;
    e_dot = q_dot_des - q_dot;
    
    % Initialize feedback vector
    feedback = zeros(num_joints, 1);
    
    % Calculate gains as matrices to account for coupling
    Kp = zeros(num_joints, num_joints);
    Ki = zeros(num_joints, num_joints);
    Kd = zeros(num_joints, num_joints);
    
    % Extract gains and calculate error states for each joint
    for i = 1:num_joints
        % Get gains for this joint
        Kp(i,i) = pid_params{i}.Kp;
        Ki(i,i) = pid_params{i}.Ki;
        Kd(i,i) = pid_params{i}.Kd;
        
        % Update error state
        % Calculate integral error
        if isempty(prev_error_states{i})
            % Initialize if first iteration
            error_states{i} = struct('e', e(i), 'e_dot', e_dot(i), 'e_int', e(i) * dt);
        else
            % Update integral term with anti-windup
            e_int = prev_error_states{i}.e_int + e(i) * dt;
            
            % Apply anti-windup
            max_integral = pid_params{i}.max_integral;
            if abs(e_int) > max_integral
                e_int = sign(e_int) * max_integral;
            end
            
            error_states{i} = struct('e', e(i), 'e_dot', e_dot(i), 'e_int', e_int);
        end
    end
    
    % Assemble integral error vector
    e_int = zeros(num_joints, 1);
    for i = 1:num_joints
        e_int(i) = error_states{i}.e_int;
    end
    
    % Calculate feedback term using matrix gains for better coupling handling
    feedback = Kp * e + Kd * e_dot + Ki * e_int;
    
    % IMPORTANT: Apply computed torque control with feed-forward AND account for coupling
    % This uses matrix operations so cross-joint effects are properly handled
    tau = M * (q_ddot_des + feedback) + C * q_dot_des + G;
    
    % Apply actuator constraints if needed
    max_torque = [10.0; 100.0; 100.0; 5.0; 5.0; 5.0; 1.0];
    for i = 1:num_joints
        if abs(tau(i)) > max_torque(i)
            tau(i) = sign(tau(i)) * max_torque(i);
        end
    end
end