function plotting_GA(best_xx, best_u_cl, best_time_steps, state2, control2, time2)
    % Ensure best_xx is trimmed to match best_time_steps
    num_steps = length(best_time_steps);
    best_xx = best_xx(:, 1:num_steps+1); % best_xx has one more column than best_time_steps

    % Plot control inputs
    figure
    hold on
    stairs(best_time_steps, best_u_cl(:, 1), 'k', 'linewidth', 1.5);
    stairs(best_time_steps, best_u_cl(:, 2), 'k', 'linewidth', 1.5);
    stairs(best_time_steps, best_u_cl(:, 3), 'k', 'linewidth', 1.5);
    stairs(best_time_steps, best_u_cl(:, 4), 'k', 'linewidth', 1.5);
    plot(time2, control2(:, 1), 'b');
    plot(time2, control2(:, 2), 'r');
    plot(time2, control2(:, 3), 'g');
    plot(time2, control2(:, 4), 'm');
    axis([0 max(best_time_steps) -1000 1000])
    ylabel('u (N)')
    legend('u1', 'u2', 'u3', 'u4', 'u1*', 'u2*', 'u3*', 'u4*')
    grid on

    % Plot 3D trajectory
    figure
    plot3(state2(:, 1), state2(:, 3), state2(:, 5), 'b')
    hold on
    plot3(best_xx(1, :), best_xx(3, :), best_xx(5, :), '-k.', 'MarkerSize', 7, 'LineWidth', 0.5)
    grid on
    xlabel('X-axis')
    ylabel('Y-axis')
    zlabel('Z-axis')
    title('MPC Trajectory Plot')
    legend('Reference', 'Optimized')

    % Plot positions
    figure
    subplot(131)
    plot(best_time_steps, best_xx(1, 1:end-1)); hold on
    plot(time2, state2(:, 1));
    legend('x', 'xref')
    subplot(132)
    plot(best_time_steps, best_xx(3, 1:end-1)); hold on
    plot(time2, state2(:, 3));
    legend('y', 'yref')
    subplot(133)
    plot(best_time_steps, best_xx(5, 1:end-1)); hold on
    plot(time2, state2(:, 5));
    legend('z', 'zref')

    % Plot omegas
    figure
    plot(best_time_steps, best_xx(13, 1:end-1)); hold on
    plot(best_time_steps, best_xx(14, 1:end-1));
    plot(best_time_steps, best_xx(15, 1:end-1));
    plot(best_time_steps, best_xx(16, 1:end-1));
    plot(time2, state2(:, 13));
    plot(time2, state2(:, 14));
    plot(time2, state2(:, 15));
    plot(time2, state2(:, 16));
    legend('omega1', 'omega2', 'omega3', 'omega4', 'omega1*', 'omega2*', 'omega3*', 'omega4*')

    % Plot roll and pitch
    figure
    subplot(121)
    plot(best_time_steps, best_xx(7, 1:end-1)); hold on 
    plot(time2, state2(:, 7));
    legend('roll', 'roll_ref')
    subplot(122)
    plot(best_time_steps, best_xx(9, 1:end-1)); hold on 
    plot(time2, state2(:, 9));
    legend('pitch', 'pitch_ref')
end