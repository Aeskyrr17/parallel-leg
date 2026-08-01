function [Q, R] = lqr_config()
% LQR tuning parameters.
% Edit only these two matrices, then build the project with CMake.

% State:
% [position, velocity, yaw, yaw_rate, left_angle, left_angle_rate, ...
%  right_angle, right_angle_rate, pitch, pitch_rate]
Q = diag([175, 40, 450, 90, 1000, 2, 1000, 2, 6688, 4]);

% Input:
% [left_wheel_torque, right_wheel_torque, left_leg_torque, right_leg_torque]
R = diag([2.25, 2.25, 3.0, 3.0]);
end
