clear;
close all;
clc;

% Add paths and load parameters
addpath(genpath('../functions'));

test_chessboard_kinematics()

function test_chessboard_kinematics()
    % Set up chessboard parameters
    chessPiece_h = 0.095;        % height of chess piece
    link_lengths = [0.121851; 0.1]; % L_45, L_6

    % Initialize error counters
    total_tests = 0;
    failed_tests = 0;

    % Loop over all 8x8 squares
    for row = 1:1
        for col = 1:1
            % Compute center of square in board coordinates
            square_label = sprintf('%c%d', 'a' + col - 1, row);
            [square_x, square_y] = getSquareCoord(char('a' + col - 1), row);

            % Define pose target
            pos_target = [square_x; square_y; chessPiece_h; 0; 0; 0];

            try
                % Run inverse kinematics
                [vals, ~] = inverseKinematics(pos_target, link_lengths);

                % Run forward kinematics for validation
                [pos_fk, orient_fk] = forwardKinematics(vals, link_lengths);

                % Calculate axis-wise position errors
                pos_error = pos_fk - pos_target(1:3);
                pos_magnitude = norm(pos_error);

                % Calculate axis-wise orientation errors
                gz_target = pos_target(4);
                gz_actual = orient_fk(3);
                gz_error = angleDiff(gz_actual, gz_target);
                orient_magnitude = abs(gz_error);

                % Check if the errors are within acceptable thresholds
                pos_threshold = 1e-3;  % 1 mm
                orient_threshold = 1e-2;  % ~0.57 degrees
                if pos_magnitude > pos_threshold || orient_magnitude > orient_threshold
                    fprintf('\n--- Square [%s] ---\n', square_label);
                    fprintf('Target: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', pos_target);
                    fprintf('Actual: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n', pos_fk, orient_fk);
                    fprintf('Position error: [%.6f, %.6f, %.6f] m (Magnitude: %.6f m)\n', ...
                        pos_error(1), pos_error(2), pos_error(3), pos_magnitude);
                    fprintf('Orientation error (gz): [%.6f] rad (Magnitude: %.6f rad)\n', ...
                        gz_error, orient_magnitude);
                    failed_tests = failed_tests + 1;
                end

                % Print results for debugging
                fprintf('theta_1  = %6.2f°\n', rad2deg(vals(1)));
                fprintf('d_2 = %.4f m\n', vals(2));
                fprintf('d_3  = %.4f m\n', vals(3));
                fprintf('theta_4  = %6.2f°\n', rad2deg(vals(4)));
                fprintf('theta_5  = %6.2f°\n', rad2deg(vals(5)));
                fprintf('theta_6  = %6.2f°\n', rad2deg(vals(6)));
                fprintf('theta_7  = %6.2f°\n', rad2deg(vals(7)));
                fprintf('\n-----------------\n');
                
                visualizeConfiguration(vals, link_lengths, square_label)

            catch ME
                fprintf('\n--- Square [%s] ---\n', square_label);
                fprintf('IK failed: %s\n', ME.message);
                failed_tests = failed_tests + 1;
            end
            
            total_tests = total_tests + 1;
        end
    end

    % Summary
    fprintf('\nTest Summary:\n');
    fprintf('Total Tests: %d\n', total_tests);
    fprintf('Passed Tests: %d\n', total_tests - failed_tests);
    fprintf('Failed Tests: %d\n', failed_tests);
    fprintf('Pass Rate: %.2f%%\n', 100 * (total_tests - failed_tests) / total_tests);
end

function diff = angleDiff(angle1, angle2)
    % Returns the difference between two angles accounting for periodicity
    diff = mod(angle1 - angle2 + pi, 2*pi) - pi;
end
