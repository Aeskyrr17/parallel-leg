function [Q, R] = lqr_config()
% LQR tuning parameters.
% Edit only these two matrices, then build the project with CMake.

% State:
% [position, velocity, yaw, yaw_rate, left_angle, left_angle_rate, ...
%  right_angle, right_angle_rate, pitch, pitch_rate]
Q = diag([80, 8, 400, 80, 1000, 2, 1000, 2, 10000, 4]);

% Input:
% [left_wheel_torque, right_wheel_torque, left_leg_torque, right_leg_torque]
R = diag([1.5, 1.5, 0.5, 0.5]);
end
