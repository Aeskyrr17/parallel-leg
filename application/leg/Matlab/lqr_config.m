function [Q, R] = lqr_config()
% LQR tuning parameters.
% Edit only these two matrices, then build the project with CMake.

% State:
% [position, velocity, yaw, yaw_rate, left_angle, left_angle_rate, ...
%  right_angle, right_angle_rate, pitch, pitch_rate]
Q = diag([358, 35, 555, 55, 1000, 4, 1000, 4, 6888, 12]);

% Input:
% [left_wheel_torque, right_wheel_torque, left_leg_torque, right_leg_torque]
R = diag([3.85, 3.85, 1.25, 1.25]);
end
