%% Robot Dynamics Generator
% This script creates symbolic dynamic models for the 7-DOF robot
% and exports optimized MATLAB functions for simulation

clear; clc; close all;

%% Rotation Matrix Helper Functions
rotz = @(t) [ cos(t) -sin(t) 0;
              sin(t)  cos(t) 0;
                 0       0   1 ];

rotx = @(t) [ 1    0       0;
              0  cos(t) -sin(t);
              0  sin(t)  cos(t)];

skew = @(p) [    0   , -p(3),  p(2);
                p(3)   ,    0 , -p(1);
               -p(2)   ,  p(1),    0];

%% Define Symbolic Variables
% Joint Variables
syms theta_1 d_2 d_3 theta_4 theta_5 theta_6 theta_7 real
q = [theta_1; d_2; d_3; theta_4; theta_5; theta_6; theta_7];

% Joint Velocity Variables
syms dtheta_1 dd_2 dd_3 dtheta_4 dtheta_5 dtheta_6 dtheta_7 real
dq = [dtheta_1; dd_2; dd_3; dtheta_4; dtheta_5; dtheta_6; dtheta_7];

% Joint Acceleration Variables
syms ddtheta_1 ddd_2 ddd_3 ddtheta_4 ddtheta_5 ddtheta_6 ddtheta_7 real
ddq = [ddtheta_1; ddd_2; ddd_3; ddtheta_4; ddtheta_5; ddtheta_6; ddtheta_7];

% Link Parameters
syms L_45 L_6 real

% Dynamic Parameters
syms m1 m2 m3 m4 m5 m6 m7 real  % Link masses
syms g real                     % Gravity

%% DH Transform Matrices (Frame i to i-1)
H1 = [rotz(theta_1) [0;0;0]; 0 0 0 1];
H2 = [rotz(-sym(pi)/2)* rotx(-sym(pi)/2) [0; 0; d_2]; 0 0 0 1];
H3 = [rotx(sym(pi)/2) * rotz(sym(pi)/2) * rotx(sym(pi)/2) [0; 0; d_3]; 0 0 0 1];
H4 = [rotz(theta_4) [L_45*cos(theta_4); L_45*sin(theta_4); 0]; 0 0 0 1];
H5 = [rotz(theta_5) [L_45*cos(theta_5); L_45*sin(theta_5); 0]; 0 0 0 1];
H6 = [rotz(theta_6) [L_6*cos(theta_6); L_6*sin(theta_6); 0]; 0 0 0 1];
H7 = [rotz(sym(pi)/2) * rotx(sym(pi)/2) * rotz(theta_7) [0; 0; 0;]; 0 0 0 1];

% Frame Rotation Matrices (Frame i to i-1)
Rot = cat(3, H1(1:3, 1:3).', H2(1:3, 1:3).', H3(1:3, 1:3).', H4(1:3, 1:3).', H5(1:3, 1:3).', H6(1:3, 1:3).', H7(1:3, 1:3).');

%% Frame Transforms (Frame i to 0)
T0 = eye(4);
T1 = T0 * H1;
T2 = T1 * H2;
T3 = T2 * H3;
T4 = T3 * H4;
T5 = T4 * H5;
T6 = T5 * H6;
T7 = T6 * H7;
Transf = cat(3, T1, T2, T3, T4, T5, T6, T7);

%% Link Vector Definitions
% Link vector definitions in local frame
r1 = [0; 0; 0];
r2 = [0; -d_2; 0];
r3 = [0; 0; d_3];
r4 = [L_45; 0; 0];
r5 = [L_45; 0; 0];
r6 = [L_6; 0; 0];
r7 = [0; 0; 0];
rs = [r1, r2, r3, r4, r5, r6, r7];

% Center of mass vector definitions in local frame
lc1 = 0;      % Base joint - negligible
lc2 = d_2/2;  % Prismatic joint - center
lc3 = d_3/2;  % Prismatic joint - center
lc4 = L_45/2; % Link 4 - center
lc5 = L_45/2; % Link 5 - center
lc6 = L_6/2;  % Link 6 - center
lc7 = 0;      % End effector - negligible

rc1 = [0; 0; 0]; 
rc2 = [0; lc2; 0]; 
rc3 = [0; 0; -lc3];
rc4 = [-lc4*cos(theta_4); -lc4*sin(theta_4); 0];
rc5 = [-lc5*cos(theta_5); -lc5*sin(theta_5); 0];
rc6 = [-lc6*cos(theta_6); -lc6*sin(theta_6); 0];
rc7 = [0; 0; 0];
rcs = [rc1, rc2, rc3, rc4, rc5, rc6, rc7];

%% Joint Types and Velocities
% Joint types: 0 = prismatic, 1 = revolute
jointTypes = [1 0 0 1 1 1 1];

omega = cell(1,8); % angular velocity in frame i
v = cell(1,8);     % linear velocity in frame i
vC = cell(1,8);    % velocity of center of mass

omega{1} = zeros(3,1); % omega0 in frame 0
v{1} = zeros(3,1);     % v0 in frame 0

for i = 1:7
    Ri = Rot(:,:,i);        % R_i_i-1
    z_im1 = [0; 0; 1];      % axis of motion in i-1
    qdot_i = dq(i);     
    
    % Angular velocity
    if jointTypes(i) == 1  % Revolute
        omega{i+1} = Ri * (omega{i} + z_im1 * qdot_i);
    else                   % Prismatic
        omega{i+1} = Ri * omega{i};
    end

    % Linear velocity
    if jointTypes(i) == 1  % Revolute
        v{i+1} = Ri * v{i} + cross(omega{i + 1}, rs(:,i));
    else                   % Prismatic
        v{i+1} = Ri * (v{i} + z_im1 * qdot_i) + cross(omega{i + 1}, rs(:,i));
    end

    % Center of mass velocity
    rCi = rcs(:,i);
    vC{i+1} = v{i+1} + cross(omega{i + 1}, rCi);
end

%% Inertia Matrices and Energy
% Mass definitions (placeholders - will be set in simulation)
link_masses = [m1, m2, m3, m4, m5, m6, m7];

% Inertia tensors (simple models for each link)
% Using parallel axis theorem: I = I_cm + m*d^2
% For simplicity, using cylindrical approximations
I1 = zeros(3,3);                                    % Base - negligible for dynamics
I2 = diag([0.001, 0.001, m2*(d_2/2)^2]);            % Vertical prismatic joint
I3 = diag([0.001, 0.001, m3*(d_3/2)^2]);            % Horizontal prismatic joint
I4 = diag([m4*(L_45/2)^2, m4*(L_45/2)^2, 0.001]);   % Link 4
I5 = diag([m5*(L_45/2)^2, m5*(L_45/2)^2, 0.001]);   % Link 5
I6 = diag([m6*(L_6/2)^2, m6*(L_6/2)^2, 0.001]);     % Link 6
I7 = diag([0.001, 0.001, 0.001]);                   % End effector - negligible

link_inertias = {I1, I2, I3, I4, I5, I6, I7};

% Calculate kinetic and potential energies
KE = 0;
PE = 0;

for i = 1:7
    % Kinetic Energy: 0.5 * m * v^2 + 0.5 * ω^T * I * ω
    KE = KE + (1/2) * link_masses(i) * (vC{i+1}.' * vC{i+1}) + ...
             (1/2) * (omega{i+1}.' * link_inertias{i} * omega{i+1});
    
    % Potential Energy: m*g*h (height in base frame)
    p_i = Transf(1:3, 4, i);                        % origin of frame {i} in base
    rCi_local = rcs(:,i);                           % COM offset in local frame {i}
    Ri_0 = Transf(1:3, 1:3, i);                     % Rotation from {i} to {0}
    pCi = p_i + Ri_0 * rCi_local;                   % COM position in base frame
    PE = PE + link_masses(i) * g * pCi(3);          % PE = m*g*h
end

% Simplify energies
KE = simplify(KE);
PE = simplify(PE);

% Lagrangian
L = KE - PE;
disp('Lagrangian calculated successfully')

%% Derive Equations of Motion
fprintf('Deriving equations of motion...\n');

% Initialize tau vector (generalized forces)
tau = sym('tau', [7,1]);

% Calculate symbolic M, C, G matrices for dynamics: M(q)q̈ + C(q,q̇)q̇ + G(q) = τ
fprintf('Calculating mass matrix M(q)...\n');
M = sym(zeros(7,7));
for i = 1:7
    for j = 1:7
        % Calculate the coefficient of ddq(j) in the equation for joint i
        M(i,j) = diff(diff(KE, dq(i)), dq(j));
    end
end
M = simplify(M);

fprintf('Calculating gravity vector G(q)...\n');
G = sym(zeros(7,1));
for i = 1:7
    G(i) = diff(PE, q(i));
end
G = simplify(G);

fprintf('Calculating Coriolis and centrifugal terms C(q,q̇)q̇...\n');
% First calculate the Christoffel symbols
C = sym(zeros(7,7));
for i = 1:7
    for j = 1:7
        for k = 1:7
            % Calculate the Christoffel symbols of the first kind
            C(i,j) = C(i,j) + 0.5 * (diff(M(i,j), q(k)) + ...
                               diff(M(i,k), q(j)) - ...
                               diff(M(j,k), q(i))) * dq(k);
        end
    end
end
C = simplify(C);

%% Generate MATLAB Functions
fprintf('Generating optimized MATLAB functions...\n');
output_dir = fullfile('data','generated_functions');

% Create a directory for the generated functions
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% Generate optimized functions for M, C, G
fprintf('Generating inertia matrix function...\n');
matlabFunction(M, 'File', fullfile('data','generated_functions','robot_inertia.m'), ...
               'Vars', {q, L_45, L_6, m1, m2, m3, m4, m5, m6, m7}, 'Optimize', true);

fprintf('Generating Coriolis matrix function...\n');
matlabFunction(C, 'File', fullfile('data','generated_functions','robot_coriolis.m'), ...
               'Vars', {q, dq, L_45, L_6, m1, m2, m3, m4, m5, m6, m7}, 'Optimize', true);
           
fprintf('Generating gravity vector function...\n');
matlabFunction(G, 'File', fullfile('data','generated_functions','robot_gravity.m'), ...
               'Vars', {q, g, L_45, L_6, m1, m2, m3, m4, m5, m6, m7}, 'Optimize', true);

fprintf('Dynamics model generation complete!\n');