function [pos, orient] = forwardKinematics(q, links)
% FORWARDKINEMATICS Computes the forward kinematics for the 7-DOF manipulator
%
% Inputs:
%   q     = [theta_1, d_2, d_3, theta_4, theta_5, theta_6, theta_7]
%           Joint values in rad (angles) and m (prismatic)
%   links = [L_45; L_6] - Link lengths in meters
%
% Outputs:
%   pos    = [x; y; z] - End effector position in base frame
%   orient = [gx; gy; gz] - End effector orientation (Euler angles)
%
% Example:
%   [pos, orient] = forwardKinematics([pi/4, 0.3, 0.1, pi/6, -pi/3, pi/4, 0], [0.121851; 0.3]);

% Extract joint values
theta1 = q(1);  % Base rotation
d_vert = q(2);  % Vertical prismatic joint
d_horiz = q(3); % Horizontal prismatic joint
theta4 = q(4);
theta5 = q(5);
theta6 = q(6);
theta7 = q(7);

% Extract link lengths
L_45 = links(1);
L_6 = links(2);

% Rotation matrix helper functions
rotz = @(t) [cos(t), -sin(t), 0; sin(t), cos(t), 0; 0, 0, 1];
rotx = @(t) [1, 0, 0; 0, cos(t), -sin(t); 0, sin(t), cos(t)];

% Transformation matrices from frame i to i-1
H1to0 = [rotz(theta1) [0;0;0]; 0 0 0 1];
H2to1 = [rotz(-pi/2)* rotx(-pi/2) [0; 0; d_vert]; 0 0 0 1];
H3to2 = [rotx(pi/2) * rotz(pi/2) * rotx(pi/2) [0; 0; d_horiz]; 0 0 0 1];
H4to3 = [rotz(theta4) [L_45*cos(theta4); L_45*sin(theta4); 0]; 0 0 0 1];
H5to4 = [rotz(theta5) [L_45*cos(theta5); L_45*sin(theta5); 0]; 0 0 0 1];
H6to5 = [rotz(theta6) [L_6*cos(theta6); L_6*sin(theta6); 0]; 0 0 0 1];
H7to6 = [rotz(pi/2) * rotx(pi/2) * rotz(theta7) [0; 0; 0;]; 0 0 0 1];

% Calculate transformation from end-effector to base
T0 = eye(4);
T1 = T0 * H1to0;
T2 = T1 * H2to1;
T3 = T2 * H3to2;
T4 = T3 * H4to3;
T5 = T4 * H5to4;
T6 = T5 * H6to5;
T7 = T6 * H7to6;

% Extract position from transformation matrix
pos = T7(1:3, 4);

% Extract orientation as quaternion
R = T7(1:3, 1:3);
quat = rotm2quat(R);
orient = quat2eul(quat, 'XYZ')';
end