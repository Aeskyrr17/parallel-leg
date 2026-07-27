function gain_coefficients = lqr_tuner(write_lqr_hpp, lqr_path)
% CMake calls this function to regenerate coefficients_ in lqr.hpp.
% Run lqr_tuner(false) manually to preview without writing the header.
if nargin < 1
    write_lqr_hpp = true;
end

matlab_dir = fileparts(mfilename('fullpath'));
addpath(matlab_dir);
leg_dir = fileparts(matlab_dir);
config_path = fullfile(leg_dir, 'Inc', 'leg_config.hpp');
if nargin < 2
    lqr_path = fullfile(leg_dir, 'Inc', 'lqr.hpp');
end
config_source = fileread(config_path);

% The controller uses u = gain * (state - reference), so gain = -lqr(...).
% Requires MATLAB Control System Toolbox.
[Q, R] = lqr_config();

% Values already defined by the new repository.
model.wheel_radius = read_cpp_float(config_source, 'wheel_radius_m');
model.wheel_mass = read_cpp_float(config_source, 'wheel_mass_kg');
min_leg_length = read_cpp_float(config_source, 'min_control_leg_length_m');
max_leg_length = read_cpp_float(config_source, 'max_control_leg_length_m');
leg_length_step = read_cpp_float(config_source, 'leg_length_resolution_m');

% SJTU model values not yet present in leg_config.hpp. Replace these with
% measured values when the new vehicle mechanics change.
model.half_wheel_track = 0.5 / 2;
model.body_com_height = 0.037;
model.leg_mass = 0.7682;
model.body_mass = 22.3;
model.wheel_inertia = model.wheel_mass * model.wheel_radius^2;
model.body_pitch_inertia = model.body_mass * (0.50^2 + 0.16^2) / 12;
model.body_yaw_inertia = model.body_mass * (0.50^2 + 0.35^2) / 12;
model.gravity = 9.78;

% [leg length, wheel-to-leg-COM, body-to-leg-COM, leg inertia].
% These expressions are exactly equivalent to the old 22-row lookup table.
leg_lengths = (min_leg_length:leg_length_step:max_leg_length).';
length_delta = leg_lengths - 0.14;
leg_data = [leg_lengths, ...
            0.03448 + 0.782 * length_delta, ...
            0.10552 + 0.218 * length_delta, ...
            0.12600 + 0.400 * length_delta];

length_count = size(leg_data, 1);
sample_count = length_count^2;
gain_samples = zeros(4, 10, sample_count);
length_pairs = zeros(sample_count, 2);

sample = 1;
for left_index = 1:length_count
    for right_index = 1:length_count
        left_leg = leg_data(left_index, :);
        right_leg = leg_data(right_index, :);
        [A, B] = sjtu_state_space(model, left_leg, right_leg);

        gain_samples(:, :, sample) = -lqr(A, B, Q, R);
        length_pairs(sample, :) = [left_leg(1), right_leg(1)];
        sample = sample + 1;
    end
end

left_length = length_pairs(:, 1);
right_length = length_pairs(:, 2);
fit_basis = [
    ones(sample_count, 1), ...
    left_length, ...
    right_length, ...
    left_length.^2, ...
    left_length .* right_length, ...
    right_length.^2
];

% One row for each gain in output-major order. The six columns match lqr.hpp:
% [a0, aL, aR, aLL, aLR, aRR].
gain_coefficients = zeros(40, 6);
row = 1;
for output = 1:4
    for state = 1:10
        gain_coefficients(row, :) = ...
            (fit_basis \ squeeze(gain_samples(output, state, :))).';
        row = row + 1;
    end
end

if ~write_lqr_hpp
    fprintf('/* Q = [%s] */\n', join(string(diag(Q).'), ', '));
    fprintf('/* R = [%s] */\n', join(string(diag(R).'), ', '));
    fprintf('/* a0 + aL*Ll + aR*Lr + aLL*Ll^2 + aLR*Ll*Lr + aRR*Lr^2 */\n');
    for row = 1:size(gain_coefficients, 1)
        fprintf('    {{%.6ff, %.6ff, %.6ff, %.6ff, %.6ff, %.6ff}},\n', ...
            gain_coefficients(row, :));
    end
end

if write_lqr_hpp
    write_coefficients(lqr_path, gain_coefficients);
end
end

function [A, B] = sjtu_state_space(model, left_leg, right_leg)
    Ll = left_leg(1);  Lr = right_leg(1);
    lwl = left_leg(2); lwr = right_leg(2);
    lbl = left_leg(3); lbr = right_leg(3);
    Ill = left_leg(4); Ilr = right_leg(4);

    Rw = model.wheel_radius;       Rl = model.half_wheel_track;
    lc = model.body_com_height;    g = model.gravity;
    mw = model.wheel_mass;         ml = model.leg_mass;
    mb = model.body_mass;          Iw = model.wheel_inertia;
    Ib = model.body_pitch_inertia; Iz = model.body_yaw_inertia;

    wheel_translation = mw * Rw^2 + Iw + ml * Rw^2 + mb * Rw^2 / 2;
    body_pitch = mw * Rw * lc + Iw * lc / Rw + ml * Rw * lc;
    wheel_yaw = Iz * Rw / (2 * Rl) + Iw * Rl / Rw;

    % Linearized Newton-Euler equations: M*q_ddot + G*angle + H*u = 0.
    % q_ddot = [wheel_l, wheel_r, leg_l, leg_r, pitch] acceleration.
    M = [
        Iw*Ll/Rw + mw*Rw*Ll + ml*Rw*lbl, 0, ml*lwl*lbl-Ill, 0, 0;
        0, Iw*Lr/Rw + mw*Rw*Lr + ml*Rw*lbr, 0, ml*lwr*lbr-Ilr, 0;
        -wheel_translation, -wheel_translation, ...
            -(ml*Rw*lwl + mb*Rw*Ll/2), -(ml*Rw*lwr + mb*Rw*Lr/2), 0;
        body_pitch, body_pitch, ml*lwl*lc, ml*lwr*lc, -Ib;
        wheel_yaw, -wheel_yaw, Iz*Ll/(2*Rl), -Iz*Lr/(2*Rl), 0
    ];
    G = [
        (ml*lwl + mb*Ll/2)*g, 0, 0;
        0, (ml*lwr + mb*Lr/2)*g, 0;
        0, 0, 0;
        0, 0, mb*g*lc;
        0, 0, 0
    ];
    H = [
        -(1 + Ll/Rw), 0, 1, 0;
        0, -(1 + Lr/Rw), 0, 1;
        1, 1, 0, 0;
        -lc/Rw, -lc/Rw, -1, -1;
        -Rl/Rw, Rl/Rw, 0, 0
    ];

    % Convert wheel/leg accelerations to [x, yaw, leg_l, leg_r, pitch].
    C = [
        Rw/2, Rw/2, 0, 0, 0;
        -Rw/(2*Rl), Rw/(2*Rl), -Ll/(2*Rl), Lr/(2*Rl), 0;
        0, 0, 1, 0, 0;
        0, 0, 0, 1, 0;
        0, 0, 0, 0, 1
    ];

    A = zeros(10, 10);
    A(1:2:9, 2:2:10) = eye(5);
    A(2:2:10, [5, 7, 9]) = C * (-(M \ G));
    B = zeros(10, 4);
    B(2:2:10, :) = C * (-(M \ H));
end

function value = read_cpp_float(source, name)
    number = '([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)';
    pattern = [regexptranslate('escape', name), '\s*=\s*', number, 'f?\s*;'];
    token = regexp(source, pattern, 'tokens', 'once');
    assert(~isempty(token), 'Cannot read %s from leg_config.hpp.', name);
    value = str2double(token{1});
end

function write_coefficients(header_path, coefficients)
    assert(isequal(size(coefficients), [40, 6]) && ...
           all(isfinite(coefficients(:))), ...
        'gain_coefficients must be a finite 40-by-6 matrix.');

    source = fileread(header_path);
    begin_marker = '        // LQR_COEFFICIENTS_BEGIN';
    end_marker = '        // LQR_COEFFICIENTS_END';
    begin_at = strfind(source, begin_marker);
    end_at = strfind(source, end_marker);
    assert(isscalar(begin_at) && isscalar(end_at) && begin_at < end_at, ...
        'Coefficient markers in lqr.hpp are missing or duplicated.');

    marker_end = begin_at + length(begin_marker) - 1;
    old_body = source(marker_end + 1:end_at - 1);
    old_rows = regexp(old_body, ...
        '(?m)^[ \t]*\{\{[^\r\n]*\}\},[ \t]*\r?$', 'match');
    assert(numel(old_rows) == 40, ...
        'Expected 40 coefficient rows between the lqr.hpp markers.');

    if contains(source, sprintf('\r\n'))
        eol = sprintf('\r\n');
    else
        eol = sprintf('\n');
    end

    generated_rows = '';
    for row = 1:size(coefficients, 1)
        generated_rows = [generated_rows, sprintf( ...
            '        {{%.6ff, %.6ff, %.6ff, %.6ff, %.6ff, %.6ff}},%s', ...
            coefficients(row, :), eol)]; %#ok<AGROW>
    end

    updated = [source(1:marker_end), eol, generated_rows, source(end_at:end)];
    if strcmp(updated, source)
        fprintf('%s is already up to date.\n', header_path);
        return;
    end

    file = fopen(header_path, 'wb');
    assert(file ~= -1, 'Cannot open %s for writing.', header_path);
    close_file = onCleanup(@() fclose(file));
    bytes = unicode2native(updated, 'UTF-8');
    written = fwrite(file, bytes, 'uint8');
    assert(written == numel(bytes), 'Failed to write all bytes to %s.', header_path);
    clear close_file;
    fprintf('Updated %s\n', header_path);
end
