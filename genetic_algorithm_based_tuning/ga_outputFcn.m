% % function [state, options, optchanged] = ga_outputFcn(options, state, flag)
% %     persistent best_xx best_u_cl best_time_steps best_k best_fval
% %     optchanged = false;
% % 
% %     if strcmp(flag, 'init')
% %         best_xx = [];
% %         best_u_cl = [];
% %         best_time_steps = [];
% %         best_k = [];
% %         best_fval = Inf;
% %     elseif strcmp(flag, 'iter')
% %         % Store the current population
% %         assignin('base', 'current_population', state.Population);
% %         
% %         [fval, idx] = min(state.Score);
% %         current_k = state.Population(idx, :);
% %         if fval < best_fval
% %             best_fval = fval;
% %             x0 = evalin('base', 'x0');
% %             state2 = evalin('base', 'state2');
% %             control2 = evalin('base', 'control2');
% %             time2 = evalin('base', 'time2');
% %             try
% %                 [xx, u_cl, time_steps, k] = MTPMPC1TUNING(current_k', x0, state2, control2, time2);
% %                 best_xx = xx;
% %                 best_u_cl = u_cl;
% %                 best_time_steps = time_steps;
% %                 best_k = k;
% %                 best_fval = fval;
% %                 assignin('base', 'best_xx', best_xx);
% %                 assignin('base', 'best_u_cl', best_u_cl);
% %                 assignin('base', 'best_time_steps', best_time_steps);
% %                 assignin('base', 'best_k', best_k);
% %                 assignin('base', 'best_fval', best_fval);
% %             catch e
% %                 fprintf('Error in ga_outputFcn during MTPMPC1TUNING: %s\n', e.message);
% %             end
% %         end
% %     end
% % end
% % % % % % % % % % % % % % % % % % r
% function [state, options, optchanged] = ga_outputFcn(options, state, flag)
%     persistent best_xx best_u_cl best_time_steps best_k best_fval history cost
%     optchanged = false;
% 
%     switch flag
%         case 'init'
%             best_xx = [];
%             best_u_cl = [];
%             best_time_steps = [];
%             best_k = [];
%             best_fval = Inf;
%             history(:,:,1) = state.Population;
%             cost(:,1) = state.Score;
%             assignin('base', 'current_population', state.Population);
%             assignin('base', 'current_scores', state.Score);
% 
%         case {'iter', 'interrupt'}
%             ss = size(history, 3);
%             history(:,:,ss+1) = state.Population;
%             cost(:,ss+1) = state.Score;
%             assignin('base', 'current_population', state.Population);
%             assignin('base', 'current_scores', state.Score);
% 
%             [fval, idx] = min(state.Score);
%             current_k = state.Population(idx, :);
%             if fval < best_fval
%                 best_fval = fval;
%                 x0 = evalin('base', 'x0');
%                 state2 = evalin('base', 'state2');
%                 control2 = evalin('base', 'control2');
%                 time2 = evalin('base', 'time2');
%                 try
%                     [xx, u_cl, time_steps, k] = MTPMPC1TUNING(current_k', x0, state2, control2, time2);
%                     best_xx = xx;
%                     best_u_cl = u_cl;
%                     best_time_steps = time_steps;
%                     best_k = k;
%                     best_fval = fval;
%                     assignin('base', 'best_xx', best_xx);
%                     assignin('base', 'best_u_cl', best_u_cl);
%                     assignin('base', 'best_time_steps', best_time_steps);
%                     assignin('base', 'best_k', best_k);
%                     assignin('base', 'best_fval', best_fval);
%                 catch e
%                     fprintf('Error in ga_outputFcn during MTPMPC1TUNING: %s\n', e.message);
%                 end
%             end
% 
%             if strcmp(flag, 'interrupt')
%                 save('ga_intermediate_results.mat', 'history', 'cost', 'best_k', 'best_fval', 'best_xx', 'best_u_cl', 'best_time_steps');
%             end
% 
%         case 'done'
%             ss = size(history, 3);
%             history(:,:,ss+1) = state.Population;
%             cost(:,ss+1) = state.Score;
%             assignin('base', 'current_population', state.Population);
%             assignin('base', 'current_scores', state.Score);
%             save('ga_final_results.mat', 'history', 'cost', 'best_k', 'best_fval', 'best_xx', 'best_u_cl', 'best_time_steps');
%     end
% end
% % % % % % % % % % % % % % % % % % % % % % % % % % 
function [state, options, optchanged] = ga_outputFcn(options, state, flag)
    persistent best_xx best_u_cl best_time_steps best_k best_fval history cost
    optchanged = false;

    switch flag
        case 'init'
            best_xx = [];
            best_u_cl = [];
            best_time_steps = [];
            best_k = [];
            best_fval = Inf;
            history(:,:,1) = state.Population;
            cost(:,1) = state.Score;
            assignin('base', 'current_population', state.Population);
            assignin('base', 'current_scores', state.Score);

        case {'iter', 'interrupt'}
            ss = size(history, 3);
            history(:,:,ss+1) = state.Population;
            cost(:,ss+1) = state.Score;
            assignin('base', 'current_population', state.Population);
            assignin('base', 'current_scores', state.Score);

            [fval, idx] = min(state.Score);
            current_k = state.Population(idx, :);
            if fval < best_fval
                best_fval = fval;
                x0 = evalin('base', 'x0');
                state2 = evalin('base', 'state2');
                control2 = evalin('base', 'control2');
                time2 = evalin('base', 'time2');
                try
                    % Map the 14-element current_k to the 18-element k vector
                    k_full = zeros(18, 1);
                    k_full(1:12) = current_k(1:12);
                    k_full(13:16) = 0;
                    k_full(17) = current_k(13);
                    k_full(18) = current_k(14);
%                     [xx, u_cl, time_steps, k_full] = MTPMPC2TUNING(k_full', x0, state2, control2, time2);
                     [xx, u_cl, time_steps, k_full] =  mtpmpc3tuning(k_full', x0, state2, control2, time2);
                    best_xx = xx;
                    best_u_cl = u_cl;
                    best_time_steps = time_steps;
                    best_k = k_full;
                    best_fval = fval;
                    assignin('base', 'best_xx', best_xx);
                    assignin('base', 'best_u_cl', best_u_cl);
                    assignin('base', 'best_time_steps', best_time_steps);
                    assignin('base', 'best_k', best_k);
                    assignin('base', 'best_fval', best_fval);
                catch e
                    fprintf('Error in ga_outputFcn during nmpc_code: %s\n', e.message);
                end
            end

            if strcmp(flag, 'interrupt')
                save('ga_intermediate_results.mat', 'history', 'cost', 'best_k', 'best_fval', 'best_xx', 'best_u_cl', 'best_time_steps');
            end

        case 'done'
            ss = size(history, 3);
            history(:,:,ss+1) = state.Population;
            cost(:,ss+1) = state.Score;
            assignin('base', 'current_population', state.Population);
            assignin('base', 'current_scores', state.Score);
            save('ga_final_results.mat', 'history', 'cost', 'best_k', 'best_fval', 'best_xx', 'best_u_cl', 'best_time_steps');
    end
end