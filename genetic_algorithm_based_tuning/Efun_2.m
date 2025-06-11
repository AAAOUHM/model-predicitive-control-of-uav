% function f = Efun_2(time_steps, simout, state2, time2, p)
% %  mpciter = size(u_cl, 1);
% %     xx_trimmed = xx(:, 1:mpciter)';
% %     simout = [xx_trimmed u_cl]; % Combine states and controls for Efun
% 
%     % Ensure inputs are column vectors
%     time_steps = time_steps(:);
%     time2 = time2(:);
% 
%     % Verify time2 is sampled at 0.01 s intervals
%     dt_ref = 0.01;
%     if ~all(abs(diff(time2) - dt_ref) < 1e-6)
%         error('time2 must be sampled at 0.01 s intervals');
%     end
% 
%     % Get mission time Tf from time2
%     Tf = time2(end);
% 
%     % Check simtim (from time_steps)
%     simtim = time_steps(end);
%     if simtim < 0 || Tf < 0
%         error('Simulation time or mission time cannot be negative');
%     end
% 
%     % Downsample state2 to match time_steps
%     state2_downsampled = zeros(length(time_steps), size(state2, 2));
%     for i = 1:size(state2, 2)
%         state2_downsampled(:, i) = interp1(time2, state2(:, i), time_steps, 'linear', 'extrap');
%     end
% 
%     % If simtim > Tf, use the final value of state2
%     if simtim > Tf
%         idx_beyond_Tf = time_steps > Tf;
%         state2_downsampled(idx_beyond_Tf, :) = repmat(state2(end, :), sum(idx_beyond_Tf), 1);
%     end
% 
%     % Extract state variables from simout (first 12 states)
%     x = simout(:, 1);        % x position
%     xdot = simout(:, 2);     % x velocity
%     y = simout(:, 3);        % y position
%     ydot = simout(:, 4);     % y velocity
%     z = simout(:, 5);        % z position
%     zdot = simout(:, 6);     % z velocity
%     roll = simout(:, 7);     % roll angle
%     rolldot = simout(:, 8);  % roll rate
%     pitch = simout(:, 9);    % pitch angle
%     pitchdot = simout(:, 10);% pitch rate
%     yaw = simout(:, 11);     % yaw angle
%     yawdot = simout(:, 12);  % yaw rate
% 
%     % Extract reference states from downsampled state2
%     x_ref = state2_downsampled(:, 1);
%     xdot_ref = state2_downsampled(:, 2);
%     y_ref = state2_downsampled(:, 3);
%     ydot_ref = state2_downsampled(:, 4);
%     z_ref = state2_downsampled(:, 5);
%     zdot_ref = state2_downsampled(:, 6);
%     roll_ref = state2_downsampled(:, 7);
%     rolldot_ref = state2_downsampled(:, 8);
%     pitch_ref = state2_downsampled(:, 9);
%     pitchdot_ref = state2_downsampled(:, 10);
%     yaw_ref = state2_downsampled(:, 11);
%     yawdot_ref = state2_downsampled(:, 12);
% 
%     % Define weights for each state
% %     w_pos = 1000;   % Weight for positions (x, y, z)
% %     w_vel = 100;    % Weight for velocities (xdot, ydot, zdot)
% %     w_angle = 500;  % Weight for angles (roll, pitch, yaw)
% %     w_rate = 50;    % Weight for angular rates (rolldot, pitchdot, yawdot)
% 
%     w_pos = 1;   % Weight for positions (x, y, z)
%     w_vel = 1;    % Weight for velocities (xdot, ydot, zdot)
%     w_angle =1;  % Weight for angles (roll, pitch, yaw)
%     w_rate = 1;    % Weight for angular rates (rolldot, pitchdot, yawdot)
% 
% 
%     
%     % Compute absolute errors
%     err_x = w_pos * abs(x - x_ref);
%     err_xdot = w_vel * abs(xdot - xdot_ref);
%     err_y = w_pos * abs(y - y_ref);
%     err_ydot = w_vel * abs(ydot - ydot_ref);
%     err_z = w_pos * abs(z - z_ref);
%     err_zdot = w_vel * abs(zdot - zdot_ref);
%     err_roll = w_angle * abs(roll - roll_ref);
%     err_rolldot = w_rate * abs(rolldot - rolldot_ref);
%     err_pitch = w_angle * abs(pitch - pitch_ref);
%     err_pitchdot = w_rate * abs(pitchdot - pitchdot_ref);
%     err_yaw = w_angle * abs(yaw - yaw_ref);
%     err_yawdot = w_rate * abs(yawdot - yawdot_ref);
% 
%     % Compute ITAE: sum of time-weighted absolute errors
%     % Since time_steps is sampled at intervals of T, compute Delta t
%     if length(time_steps) > 1
%         Delta_t = time_steps(2) - time_steps(1); % Should be equal to T from MTPMPC1TUNING
%     else
%         Delta_t = 0; % Edge case: only one time step
%     end
% 
%     % ITAE for each state: sum(t_k * |error(k)| * Delta_t)
%     itae_x = sum(time_steps .* err_x) * Delta_t;
%     itae_xdot = sum(time_steps .* err_xdot) * Delta_t;
%     itae_y = sum(time_steps .* err_y) * Delta_t;
%     itae_ydot = sum(time_steps .* err_ydot) * Delta_t;
%     itae_z = sum(time_steps .* err_z) * Delta_t;
%     itae_zdot = sum(time_steps .* err_zdot) * Delta_t;
%     itae_roll = sum(time_steps .* err_roll) * Delta_t;
%     itae_rolldot = sum(time_steps .* err_rolldot) * Delta_t;
%     itae_pitch = sum(time_steps .* err_pitch) * Delta_t;
%     itae_pitchdot = sum(time_steps .* err_pitchdot) * Delta_t;
%     itae_yaw = sum(time_steps .* err_yaw) * Delta_t;
%     itae_yawdot = sum(time_steps .* err_yawdot) * Delta_t;
% 
%     % Total ITAE
%     itae_total = itae_x + itae_xdot + itae_y + itae_ydot + itae_z + itae_zdot + ...
%                  itae_roll + itae_rolldot + itae_pitch + itae_pitchdot + ...
%                  itae_yaw + itae_yawdot;
% 
%     % Fitness is just ITAE (no energy term since trajectories are energy-optimal)
%     f = itae_total;
% end
% % % % % % 
% % 1st run rhuis code
% simout=[xx(:,1:end-1)' [u_cl]];
%  Efun(time_steps,simout,p)
% % simout=[xx(:,1:end-1)' [u_cl]];


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
function f = Efun_2(time_steps, simout, state2, time2, p)
    % Ensure inputs are column vectors
    time_steps = time_steps(:);
    time2 = time2(:);

    % Verify time2 is sampled at 0.01 s intervals
    dt_ref = 0.01;
    if ~all(abs(diff(time2) - dt_ref) < 1e-6)
        error('time2 must be sampled at 0.01 s intervals');
    end

    % Get mission time Tf from time2
    Tf = time2(end);

    % Check simtim (from time_steps)
    simtim = time_steps(end);
    if simtim < 0 || Tf < 0
        error('Simulation time or mission time cannot be negative');
    end

    % Downsample state2 to match time_steps
    state2_downsampled = zeros(length(time_steps), size(state2, 2));
    for i = 1:size(state2, 2)
        state2_downsampled(:, i) = interp1(time2, state2(:, i), time_steps, 'linear', 'extrap');
    end

    % If simtim > Tf, use the final value of state2
    if simtim > Tf
        idx_beyond_Tf = time_steps > Tf;
        state2_downsampled(idx_beyond_Tf, :) = repmat(state2(end, :), sum(idx_beyond_Tf), 1);
    end

    % Compute scaling factors based on the maximum absolute values of state2
    % Same as in mtpmpc3tuning.m
    scaling = max(abs(state2), [], 1); % Max of each column (state) over all rows (time steps)
    scaling = max(scaling, 1e-6); % Avoid division-by-zero

    % Extract state variables from simout (first 12 states)
    x = simout(:, 1);        % x position
    xdot = simout(:, 2);     % x velocity
    y = simout(:, 3);        % y position
    ydot = simout(:, 4);     % y velocity
    z = simout(:, 5);        % z position
    zdot = simout(:, 6);     % z velocity
    roll = simout(:, 7);     % roll angle
    rolldot = simout(:, 8);  % roll rate
    pitch = simout(:, 9);    % pitch angle
    pitchdot = simout(:, 10);% pitch rate
    yaw = simout(:, 11);     % yaw angle
    yawdot = simout(:, 12);  % yaw rate

    % Extract reference states from downsampled state2
    x_ref = state2_downsampled(:, 1);
    xdot_ref = state2_downsampled(:, 2);
    y_ref = state2_downsampled(:, 3);
    ydot_ref = state2_downsampled(:, 4);
    z_ref = state2_downsampled(:, 5);
    zdot_ref = state2_downsampled(:, 6);
    roll_ref = state2_downsampled(:, 7);
    rolldot_ref = state2_downsampled(:, 8);
    pitch_ref = state2_downsampled(:, 9);
    pitchdot_ref = state2_downsampled(:, 10);
    yaw_ref = state2_downsampled(:, 11);
    yawdot_ref = state2_downsampled(:, 12);

    % Define weights for each state (all set to 1 after normalization)
    w_pos = 1;   % Weight for positions (x, y, z)
    w_vel = 1;   % Weight for velocities (xdot, ydot, zdot)
    w_angle = 1; % Weight for angles (roll, pitch, yaw)
    w_rate = 1;  % Weight for angular rates (rolldot, pitchdot, yawdot)

%      % Define weights for each state (fixed, same for all missions)
%     w_pos = 1;   % Weight for positions (x, y, z)
%     w_vel = 0.5; % Weight for velocities (xdot, ydot, zdot)
%     w_angle = 10; % Weight for angles (roll, pitch, yaw)
%     w_rate = 5;  % Weight for angular rates (rolldot, pitchdot, yawdot)

    
    % Compute normalized absolute errors
    err_x = w_pos * abs(x - x_ref) / scaling(1);
    err_xdot = w_vel * abs(xdot - xdot_ref) / scaling(2);
    err_y = w_pos * abs(y - y_ref) / scaling(3);
    err_ydot = w_vel * abs(ydot - ydot_ref) / scaling(4);
    err_z = w_pos * abs(z - z_ref) / scaling(5);
    err_zdot = w_vel * abs(zdot - zdot_ref) / scaling(6);
    err_roll = w_angle * abs(roll - roll_ref) / scaling(7);
    err_rolldot = w_rate * abs(rolldot - rolldot_ref) / scaling(8);
    err_pitch = w_angle * abs(pitch - pitch_ref) / scaling(9);
    err_pitchdot = w_rate * abs(pitchdot - pitchdot_ref) / scaling(10);
    err_yaw = w_angle * abs(yaw - yaw_ref) / scaling(11);
    err_yawdot = w_rate * abs(yawdot - yawdot_ref) / scaling(12);

    % Compute ITAE: sum of time-weighted absolute errors
    if length(time_steps) > 1
        Delta_t = time_steps(2) - time_steps(1);
    else
        Delta_t = 0;
    end

    % ITAE for each state
    itae_x = sum(time_steps .* err_x) * Delta_t;
    itae_xdot = sum(time_steps .* err_xdot) * Delta_t;
    itae_y = sum(time_steps .* err_y) * Delta_t;
    itae_ydot = sum(time_steps .* err_ydot) * Delta_t;
    itae_z = sum(time_steps .* err_z) * Delta_t;
    itae_zdot = sum(time_steps .* err_zdot) * Delta_t;
    itae_roll = sum(time_steps .* err_roll) * Delta_t;
    itae_rolldot = sum(time_steps .* err_rolldot) * Delta_t;
    itae_pitch = sum(time_steps .* err_pitch) * Delta_t;
    itae_pitchdot = sum(time_steps .* err_pitchdot) * Delta_t;
    itae_yaw = sum(time_steps .* err_yaw) * Delta_t;
    itae_yawdot = sum(time_steps .* err_yawdot) * Delta_t;

    % Total ITAE
    itae_total = itae_x + itae_xdot + itae_y + itae_ydot + itae_z + itae_zdot + ...
                 itae_roll + itae_rolldot + itae_pitch + itae_pitchdot + ...
                 itae_yaw + itae_yawdot;

    % Fitness is just ITAE
    f = itae_total;
end
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
