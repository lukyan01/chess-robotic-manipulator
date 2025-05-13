function [x_robot, y_robot] = getSquareCoord(file_col, rank_row)
% GETSQUARECOORD Converts chessboard square to robot frame coordinates
%
% Inputs:
%   file_col - char 'a' to 'h' (column on chessboard)
%   rank_row - int 1 to 8 (row on chessboard)
%
% Outputs:
%   x_robot, y_robot - coordinates in robot's 2D XY frame
%
% Example:
%   [x, y] = getSquareCoord('e', 4);

% Chess square size in meters
square_size = 0.06;  

% Compute board frame (x, y) — bottom-left of a1 is (0, 0)
col_idx = double(lower(file_col)) - double('a');   % 0 to 7
row_idx = rank_row - 1;                            % 0 to 7

% Calculate center position of the square in board frame
target_b = [(col_idx + 0.5) * square_size; (row_idx + 0.5) * square_size];

% Board to robot frame transformation
% The robot is positioned at the base of the board, rotated 90 degrees
R = [cos(pi/2), -sin(pi/2);
     sin(pi/2),  cos(pi/2)];  % rotation of +π/2 
T = [0.18; 0.24];             % translation offset between board and robot

% Transform from board coordinates to robot coordinates
robot_pos = R.'*target_b + T;

x_robot = robot_pos(1);
y_robot = robot_pos(2);
end