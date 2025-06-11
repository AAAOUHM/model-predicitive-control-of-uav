% clear all;close all;addpath('utils');
% x00y00z30 e=8717 t=6.63 

% x10y00z20 e=6881 t=5.08
% x50y00z20 e=10064 t=6.94
% x10y25z15 e=7813 / 78035 t=5.41

p = Parameter(2); % 1 for Sigma25 2 for MLara
auxdata.c1 = p.c1;auxdata.c2 = p.c2;auxdata.c3 = p.c3;auxdata.c4 = p.c4;auxdata.c5 = p.c5;      
auxdata.c6 = p.c6;auxdata.c7 = p.c7;auxdata.c8 = p.c8;auxdata.c9 = p.c9;auxdata.kb = p.kb;
auxdata.ktau = p.ktau;auxdata.M = p.M;auxdata.g = p.g;auxdata.L = p.L;auxdata.J = p.J;
auxdata.Iy = p.Iy;auxdata.Ix = p.Ix;auxdata.Iz = p.Iz;
auxdata.omegamax = p.omegamax;auxdata.wh = p.wh,auxdata.rate = p.rate;

%----------------- Provide All Bounds for Problem ------------------------%

t0 = 0;tf = 20;


x = 10; y = 25; z = 15;
x10 = 0;  x1f = x;     x1min = 0;     x1max = x+10;
x20 = 0;  x2f = 0;     x2min = -15;   x2max = 15;
x30 = 0;  x3f = y;     x3min = 0;     x3max = y+10;
x40 = 0;  x4f = 0;     x4min = -15;   x4max = 15;
x50 = 0;  x5f = z;     x5min = 0;     x5max = z+10;
x60 = 0;  x6f = 0;     x6min = -2;    x6max = +6;
deg=35;
x70 = 0;  x7f = 0;      x7min =  (pi/180)*-deg;   x7max = (pi/180)*+deg;
x80 = 0;  x8f = 0;      x8min =  -1.5;      x8max = 1.5;
x90 = 0;  x9f = 0;      x9min =  (pi/180)*-deg;   x9max = (pi/180)*+deg;
x100 = 0; x10f = 0;     x10min = -1.5;      x10max = +1.5;
x110 = 0; x11f = 0;     x11min = 0*(pi/180)*-180;   x11max = 0*(pi/180)*+180;
x120 = 0; x12f = 0;     x12min = -1.5;        x12max = +1.5;



x130 = p.wh;  x13f = p.wh;   x13min =   0;   x13max = auxdata.omegamax;
x140 = p.wh;  x14f = p.wh;   x14min =   0;   x14max = auxdata.omegamax;
x150 = p.wh;  x15f = p.wh;   x15min =   0;   x15max = auxdata.omegamax;
x160 = p.wh;  x16f = p.wh;   x16min =   0;   x16max = auxdata.omegamax;

umin = -p.rate;umax = p.rate;
u1min = umin; u1max =  umax;
u2min = umin; u2max =  umax;
u3min = umin; u3max =  umax;
u4min = umin; u4max =  umax;

tfmin = 0.1; tfmax= 100;

bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = 0;
bounds.phase.finaltime.upper = tf;
bounds.phase.initialstate.lower = [x10, x20, x30, x40, x50, x60, x70, x80, x90, x100, x110, x120, x130, x140, x150, x160];
bounds.phase.initialstate.upper = [x10, x20, x30, x40, x50, x60, x70, x80, x90, x100, x110, x120, x130, x140, x150, x160];
bounds.phase.state.lower = [x1min, x2min, x3min, x4min, x5min, x6min, x7min, x8min, x9min, x10min, x11min, x12min, x13min, x14min, x15min, x16min];
bounds.phase.state.upper = [x1max, x2max, x3max, x4max, x5max, x6max, x7max, x8max, x9max, x10max, x11max, x12max, x13max, x14max, x15max, x16max];
bounds.phase.finalstate.lower = [x1f, x2f, x3f, x4f, x5f, x6f, x7f, x8f, x9f, x10f, x11f, x12f, x13f, x14f, x15f, x16f];
bounds.phase.finalstate.upper = [x1f, x2f, x3f, x4f, x5f, x6f, x7f, x8f, x9f, x10f, x11f, x12f, x13f, x14f, x15f, x16f];
bounds.phase.control.lower = [u1min, u2min, u3min, u4min];
bounds.phase.control.upper = [u1max, u2max, u3max, u4max];
bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 400000;


tGuess                = [t0; tf];
x1Guess               = [x10; x1f];
x2Guess               = [x20; x2f];
x3Guess               = [x30; x3f];
x4Guess               = [x40; x4f];
x5Guess               = [x50; x5f];
x6Guess               = [x60; x6f];
x7Guess               = [x70; x7f];
x8Guess               = [x80; x8f];
x9Guess               = [x90; x9f];
x10Guess              = [x100; x10f];
x11Guess              = [x110; x11f];
x12Guess              = [x120; x12f];
x13Guess              = [x130; x13f];
x14Guess              = [x140; x14f];
x15Guess              = [x150; x15f];
x16Guess              = [x160; x16f];
u1Guess              = [u1min; u1max];
u2Guess              = [u2min; u2max];
u3Guess              = [u3min; u3max];
u4Guess              = [u4min; u4max];
guess.phase.state    = [x1Guess, x2Guess, x3Guess, x4Guess, x5Guess, x6Guess, x7Guess, x8Guess, x9Guess, x10Guess, x11Guess, x12Guess, x13Guess, x14Guess, x15Guess, x16Guess];
guess.phase.control  = [u1Guess,u2Guess,u3Guess,u4Guess];
guess.phase.time     = [tGuess];
guess.phase.integral = 0;


%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-PattersonRao';
mesh.tolerance       = 1e-6;
mesh.maxiterations   = 100;
mesh.colpointsmin    = 4;
mesh.colpointsmax    = 10;
N                    = 100;
mesh.phase.colpoints = 4*ones(1,N);
mesh.phase.fraction  = ones(1,N)/N;
mesh.phase.colpoints = 10*ones(1,N);
mesh.phase.fraction  = ones(1,N)/N;

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.mesh                            = mesh;
setup.name                            = 'LeastEnergyMain';
% % % % % % % % v important
setup.functions.endpoint              = @LeastEnergyEndpoint;
setup.functions.continuous            = @LeastEnergyContinuous;
setup.auxdata                         = auxdata;
setup.displaylevel                    = 2;
setup.bounds                          = bounds;
setup.guess                           = guess;
setup.nlp.solver                      = 'ipopt';
setup.nlp.ipoptoptions.linear_solver  = 'ma57';
setup.nlp.ipoptoptions.tolerance       = 1e-9;
setup.nlp.snoptoptions.tolerance       = 1e-8;
setup.derivatives.supplier            = 'sparseCD';
setup.derivatives.derivativelevel     = 'second';
setup.method                          = 'RPM-Differentiation';
setup.derivatives.derivativelevel     = 'second';
setup.mesh                            = mesh;
setup.scales.method                   = 'automatic-guess';

%-------------------------------------------------------------------------%
%----------------------- Solve Problem Using GPOPS2 ----------------------%
%-------------------------------------------------------------------------%
outputGPOPS = gpops2(setup);

%% Calculating F,M1,M2,M3 of V Kumar
state = outputGPOPS.result.solution.phase.state;
time = outputGPOPS.result.solution.phase.time;
control = outputGPOPS.result.solution.phase.control;

state(:,17) = p.kb*(state(:,13).*state(:,13) + state(:,14).*state(:,14) + state(:,15).*state(:,15) + state(:,16).*state(:,16));
state(:,18) = p.kb*p.L*(state(:,14).*state(:,14) - state(:,16).*state(:,16));
state(:,19) = p.kb*p.L*(state(:,15).*state(:,15) - state(:,13).*state(:,13));
state(:,20) = p.ktau*(state(:,13).*state(:,13) + state(:,15).*state(:,15) - state(:,14).*state(:,14) - state(:,16).*state(:,16));
%%

% %% State Space for VKumar 3D Sim
% state1 = [state(:,1) state(:,3) state(:,5) state(:,2) state(:,4) state(:,6) state(:,7) state(:,9) state(:,11) state(:,8) state(:,10)...
%           state(:,12) state(:,17) state(:,18) state(:,19) state(:,20)]; %State Space as in VKumar
% time1 = time;
% control1 = control;
% %%
% 
% %% Uniformly sampled state space
% tempt = linspace(0,tf,tf*100+1); %Creating equidistant time invervals of interval 0.01
% time2=tempt';
% state2=[]; %Initialization of uniformly sampled state space
% for j=1:length(state(1,:)) %Routing over each state
%     temp = state(:,j);
% for i = 1:length(tempt)-1    %Routing over each time interval of that state
%     temp3 = time>=tempt(i)&time<tempt(i+1); % Checking how many invervals are in range of a uniform time interval
%     if (sum(temp3)>0)   
%         temp1(i) = mean(temp(temp3)); % Picking mean of the state values
%     else
%         temp1(i)=temp1(i-1); % If no interval exists, picking the previous state value to avoid NaN values
%     end
% end
% temp1=temp1';
% tempstate2(:,j) = temp1;
% temp = [];
% end
% state2 = tempstate2;
% state2(tf*100+1,:) = state2(tf*100,:); % To account for the number of time samples
% 
% for j=1:length(control(1,:))
%     temp = control(:,j);
% for i = 1:length(tempt)-1    
%     temp3 = time>=tempt(i)&time<tempt(i+1);
%     if (sum(temp3)>0)   
%         temp1(i) = mean(temp(temp3));
%     else
%         temp1(i)=temp1(i-1);
%     end
% end
% temp1=temp1';
% tempcontrol2(:,j) = temp1;
% temp = [];
% end
% control2 = tempcontrol2;
% control2(tf*100+1,:) = control2(tf*100,:); % To account for the number of time samples
% [tmax,tpos]=max(time2(time2<max(time)));
% time2 = time2(1:tpos,:); %Evenly Sampled time
% state2 = state2(1:tpos,:); %Evenly Sampled state
% control2 = control2(1:tpos,:); %Evenly Sampled control
%   ttttttttttttttttttttttttt
% Uniform time vector
tempt = linspace(0, tf, tf*100+1)'; % Transpose directly to column vector sample at .001 seconds

% Uniformly resampling states
state2 = zeros(length(tempt), size(state, 2)); % Preallocate for speed
for j = 1:size(state, 2)
    state2(:, j) = interp1(time, state(:, j), tempt, 'previous', 'extrap'); 
end

% Uniformly resampling controls
control2 = zeros(length(tempt), size(control, 2)); % Preallocate for speed
for j = 1:size(control, 2)
    control2(:, j) = interp1(time, control(:, j), tempt, 'previous', 'extrap'); 
end

% Truncate at the maximum valid time to match original dataset
valid_idx = tempt <= max(time);
time2 = tempt(valid_idx, :); % Valid uniform time
state2 = state2(valid_idx, :); % Valid resampled states
control2 = control2(valid_idx, :); % Valid resampled controls
% 5565555555555555555
% 
col1 = state2(:,1);
col3 = state2(:,3);
col5 = state2(:,5);
save('asctec_import.mat', 'col1', 'col3', 'col5');

%%
outputGPOPS.result.solution.phase.integral %energy

%%for Sim
siminx1 =  [time2 state2(:,1)];         %x from GPOPS
siminx2 =  [time2 state2(:,2)];         %xdot from GPOPS
siminx3 =  [time2 state2(:,3)];         %y from GPOPS
siminx4 =  [time2 state2(:,4)];         %ydot from GPOPS
siminx5 =  [time2 state2(:,5)];         %z from GPOPS
siminx6 =  [time2 state2(:,6)];         %zdot from GPOPS
siminx7 =  [time2 state2(:,7)];         %phi from GPOPS
siminx8 =  [time2 state2(:,8)];         %phi_dot from GPOPS
siminx9 =  [time2 state2(:,9)];         %theta from GPOPS
siminx10 =  [time2 state2(:,10)];       %theta_dot from GPOPS
siminx11 =  [time2 state2(:,11)];       %psi from GPOPS
siminx12 =  [time2 state2(:,12)];       %_dot from GPOPS


siminx13 = [time2 state2(:,13)];        %RPM1 from GPOPS
siminu1 =  [time2 control2(:,1)];       %alpha from GPOPS
siminx14 = [time2 state2(:,14)];        %RPM2 from GPOPS
siminu2 =  [time2 control2(:,2)];       %alpha from GPOPS
siminx15 = [time2 state2(:,15)];        %RPM3 from GPOPS
siminu3 =  [time2 control2(:,3)];       %alpha from GPOPS
siminx16 = [time2 state2(:,16)];        %RPM4 from GPOPS
siminu4 =  [time2 control2(:,4)];       %alpha from GPOPS


length(col1)
pids=[5 15 0 5];