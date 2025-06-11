% 
% addpath('C:\Users\Desktop\casadi-3.6.5-windows64-matlab2018b')
% import casadi.*
% 
% % % Load reference trajectory data
% % load('reference_trajectory.mat', 'state2', 'control2', 'time2', 'p');
% % 
% % Define results storage for a single mission
% results = zeros(1, 9); % Columns: k(1) to k(8), energy
% 
% % Define initial k (scaled to match Q scaling in MTPMPC1TUNING)
% k = [0.001; 0.0002; 0.0002; 0.00003; 0.09; 0.0001; 0; 10];
% 
% % Define number of variables
% numvars = length(k); % 8
% 
% % Define bounds (scaled)
% % %  MTPMPC1TUNING
% lb = [zeros(1,6) 0 6];
% ub = [ones(1,6)*100 10 12];
% 
% % Define population size
% popsiz = 30;
% 
% % Create initial population
% population = zeros(popsiz, numvars);
% population(1, :) = k'; % First individual row
% num_individuals = popsiz - 1; % 29 individuals
% for j = 1:numvars-1
%     population(2:end, j) = lb(j) + (ub(j) - lb(j)) * rand(num_individuals, 1);
% end
% population(2:end, numvars) = randi([6, 12], num_individuals, 1);
% 
% % Define GA settings
% IntCon = 8; % k(8) is an integer
% options = optimoptions('ga', ...
%     'PopulationSize', popsiz, ...
%     'MaxGenerations', 20, ... % Increase to 20 for full run
%     'UseParallel', true, ...
%     'Display', 'iter', ...
%     'StallGenLimit', 20, ...
%     'FunctionTolerance', 1e-6, ...
%     'InitialPopulationMatrix', population, ...
%     'OutputFcn', @ga_outputFcn);
% 
% % Define initial state
% n_states = 16;
% omegahvr = 911.85;
% x0 = [0; 0; 0; zeros(n_states - 4 - 3, 1); ones(4, 1) * omegahvr];
% 
% % Create an anonymous function to pass additional arguments to ga_tuning
% fitnessFcn = @(k) ga_tuning(k, x0, state2, control2, time2, p);
% 
% % Run optimization for a single mission
% assignin('base', 'x0', x0);
% assignin('base', 'state2', state2);
% assignin('base', 'control2', control2);
% assignin('base', 'time2', time2);
% tic;
% try
%     [k_opt, energy_opt] = ga(fitnessFcn, numvars, [], [], [], [], lb, ub, [], IntCon, options);
% catch e
%     fprintf('GA was interrupted: %s\n', e.message);
%     if evalin('base', 'exist(''best_k'', ''var'')')
%         k_opt = evalin('base', 'best_k');
%         energy_opt = evalin('base', 'best_fval');
%         fprintf('Best k found so far (k_opt):\n');
%         disp(k_opt');
%         fprintf('Best fitness value so far: %.2f\n', energy_opt);
%     else
%         fprintf('No best_k found yet.\n');
%         k_opt = [];
%         energy_opt = Inf;
%     end
% end
% elapsed_time = toc;
% best_xx = evalin('base', 'best_xx');
% best_u_cl = evalin('base', 'best_u_cl');
% best_time_steps = evalin('base', 'best_time_steps');
% best_k = evalin('base', 'best_k');
% best_fval = evalin('base', 'best_fval');
% fprintf('Optimized k vector (k_opt):\n');
% disp(k_opt);
% fprintf('Best k vector (from output function):\n');
% disp(best_k);
% fprintf('Best fitness value: %.2f\n', best_fval);
% fprintf('Elapsed time: %.2f seconds (%.2f hours)\n', elapsed_time, elapsed_time/3600);
% results(1, :) = [k_opt, energy_opt];
% assignin('base', 'best_xx_pos1', best_xx);
% assignin('base', 'best_u_cl_pos1', best_u_cl);
% assignin('base', 'best_time_steps_pos1', best_time_steps);
% assignin('base', 'best_k_pos1', best_k);
% assignin('base', 'best_fval_pos1', best_fval);
% 
% % Display results
% disp('Optimization Results:');
% disp('Optimized k (k1 to k8) | Energy');
% fprintf('k=[%s] | Energy=%.2f\n', ...
%     sprintf('%.1f, ', results(1, 1:numvars)), results(1, numvars+1));
% 
% % Plot the results
% plotting_GA(best_xx, best_u_cl, best_time_steps, state2, control2, time2);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
addpath('C:\Users\Desktop\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

% % Load reference trajectory data
% load('reference_trajectory.mat', 'state2', 'control2', 'time2', 'p');

% Define results storage for a single mission
results = zeros(1, 15); % Columns: k(1) to k(14), cost (14 optimized elements + 1 for cost)

% Define initial k for mtpmpc3tuning (14 variables: k(1:12), k(17), k(18))
k = [1; 4; 1; 4; 5; 4; ... % k(1:6) for x, xdot, y, ydot, z, zdot
     200; 800; 200; 800; 100; 400; ... % k(7:12) for phi, phidot, theta, thetadot, psi, psidot
     0.00005; 13]; % k(17:18) for R and N (N set to 13, within bounds 8 to 22)

 % Define initial k for mtpmpc3tuning (14 variables: k(1:12), k(17), k(18))
k = [1; 4; 1; 4; 5; 4; ... % k(1:6) for x, xdot, y, ydot, z, zdot
     .200; .800; .200; .800; 100; 400; ... % k(7:12) for phi, phidot, theta, thetadot, psi, psidot
     0.00005; 7]; % k(17:18) for R and N (N set to 13, within bounds 8 to 22)

 
% Define number of variables
numvars = length(k); % 14

% Define bounds (scaled)
lb = [zeros(1,6) zeros(1,4) zeros(1,2) 0 6];% reduced n from 8 to 6
ub = [ones(1,6)*1000000000 ones(1,4)*1000 500 500 0.1 22];
% only posn and vel
ub = [ones(1,6)*1000000000 zeros(1,4)*1000 00 00 0.1 22];
% only posn 
ub = [1000000 0 1000000 0 1000000 0 zeros(1,4)*1000 00 00 0.1 22];
% more posn and low vel terminal weights on attitudes reduced n to 18
ub = [1000000 100 1000000 100 1000000 100 ones(1,4)*1000 00 00 0.1 18];

% % % % % % % 

ub = [10000000 00 10000000 00 10000000 00 zeros(1,4)*1000 00 00 0.1 18];


% Define population size
popsiz = 30;

% Create initial population
population = zeros(popsiz, numvars);
population(1, :) = k'; % First individual row
num_individuals = popsiz - 1; % 29 individuals
for j = 1:numvars-1
    population(2:end, j) = lb(j) + (ub(j) - lb(j)) * rand(num_individuals, 1);
end
population(2:end, numvars) = randi([8, 22], num_individuals, 1); % k(18) is N

% Define GA settings
IntCon = 14; % k(14) is an integer (k(18) in full vector, but 14th in optimized vector)
options = optimoptions('ga', ...
    'PopulationSize', popsiz, ...
    'MaxGenerations', 20, ... % Increase to 20 for full run
    'UseParallel', true, ...
    'Display', 'iter', ...
    'StallGenLimit', 20, ...
    'FunctionTolerance', 1e-6, ... 
    'InitialPopulationMatrix', population, ...
    'OutputFcn', @ga_outputFcn);

% Define initial state
n_states = 16;
omegahvr = 911.85;
x0 = [0; 0; 0; zeros(n_states - 4 - 3, 1); ones(4, 1) * omegahvr];

% Create an anonymous function to pass additional arguments to ga_tuning
fitnessFcn = @(k) ga_tuning(k, x0, state2, control2, time2, p);

% Run optimization for a single mission
assignin('base', 'x0', x0);
assignin('base', 'state2', state2);
assignin('base', 'control2', control2);
assignin('base', 'time2', time2);
tic;
try
    [k_opt, cost_opt] = ga(fitnessFcn, numvars, [], [], [], [], lb, ub, [], IntCon, options);
catch e
    fprintf('GA was interrupted: %s\n', e.message);
    if evalin('base', 'exist(''best_k'', ''var'')')
        k_opt = evalin('base', 'best_k');
        cost_opt = evalin('base', 'best_fval');
        fprintf('Best k found so far (k_opt):\n');
        disp(k_opt');
        fprintf('Best fitness value (ITAE cost) so far: %.2f\n', cost_opt);
    else
        fprintf('No best_k found yet.\n');
        k_opt = [];
        cost_opt = Inf;
    end
end
elapsed_time = toc;
best_xx = evalin('base', 'best_xx');
best_u_cl = evalin('base', 'best_u_cl');
best_time_steps = evalin('base', 'best_time_steps');
best_k = evalin('base', 'best_k');
best_fval = evalin('base', 'best_fval');
fprintf('Optimized k vector (k_opt):\n');
disp(k_opt);
fprintf('Best k vector (from output function):\n');
disp(best_k);
fprintf('Best fitness value (ITAE cost): %.2f\n', best_fval);
fprintf('Elapsed time: %.2f seconds (%.2f hours)\n', elapsed_time, elapsed_time/3600);
results(1, 1:numvars+1) = [k_opt, cost_opt]; % Adjusted for new number of variables
assignin('base', 'best_xx_pos1', best_xx);
assignin('base', 'best_u_cl_pos1', best_u_cl);
assignin('base', 'best_time_steps_pos1', best_time_steps);
assignin('base', 'best_k_pos1', best_k);
assignin('base', 'best_fval_pos1', best_fval);

% Display results
disp('Optimization Results:');
disp('Optimized k (k1 to k14) | Cost (ITAE)');
fprintf('k=[%s] | Cost=%.2f\n', ...
    sprintf('%.1f, ', results(1, 1:numvars)), results(1, numvars+1));

% Plot the results
plotting_GA(best_xx, best_u_cl, best_time_steps, state2, control2, time2);
disp(best_k);






            