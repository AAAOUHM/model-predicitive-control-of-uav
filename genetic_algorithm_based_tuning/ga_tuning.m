% function cost = ga_tuning(kk)
% assignin('base','kk',kk);
% sim('pidtunebyga.slx');
% cost= itae(length(itae));
% % only after we stop ga we acess k 
% end
% 
% 
% % 
% % % % % % % % % % % % % % % % % % mtpmpc1_tuning
% function cost = ga_tuning(k, x0, state2, control2, time2, p)% cost is energy  from efun and is itae from efun2
%     % Run MTPMPC1TUNING with the given k
% %     try
%         [xx, u_cl, time_steps, k] = MTPMPC1TUNING(k, x0, state2, control2, time2);
% %     catch e
% %         fprintf('Error in MTPMPC1TUNING: %s\n', e.message);
% %         energy = Inf;
% %         return;
% %     end
% 
%     % Trim xx to match the number of control steps
%     mpciter = size(u_cl, 1);
%     xx_trimmed = xx(:, 1:mpciter)';
%     simout = [xx_trimmed u_cl]; % Combine states and controls for Efun
% 
%     % Compute the energy (fitness) using Efun
%     try
%         cost = Efun(time_steps, simout, p);%energy
%     catch e
%         fprintf('Error in Efun: %s\n', e.message);
%         cost = Inf;
%         return;
%     end
% end
% 
% % % % % % % % % % % % % % % % % % % % % % % % % % % 
function cost = ga_tuning(k, x0, state2, control2, time2, p) % cost is energy from efun and is itae from efun2
    % k is a 14-element vector: k(1:12), k(17), k(18)
    % Construct the full 18-element k vector for MTPMPC2TUNING
    k_full = zeros(18, 1);
    k_full(1:12) = k(1:12); % Position, velocity, attitude weights
    k_full(13:16) = 0;      % Rotor speed weights (fixed at 0)
    k_full(17) = k(13);     % Control weight R
    k_full(18) = k(14);     % Prediction horizon N

    % Run MTPMPC2TUNING with the full k vector
    try
%         [xx, u_cl, time_steps, k_full] = MTPMPC2TUNING(k_full, x0, state2, control2, time2);
             [xx, u_cl, time_steps, k_full] = mtpmpc3tuning(k_full, x0, state2, control2, time2);
    catch e
        fprintf('Error in code: %s\n', e.message);
        cost = Inf;
        return;
    end

    % Trim xx to match the number of control steps
    mpciter = size(u_cl, 1);
    xx_trimmed = xx(:, 1:mpciter)';
    simout = [xx_trimmed u_cl]; % Combine states and controls for Efun

    % Compute the ITAE (fitness) using Efun_2
    try
        cost = Efun_2(time_steps, simout, state2, time2, p); % ITAE
    catch e
        fprintf('Error in Efun_2: %s\n', e.message);
        cost = Inf;
        return;
    end
end
