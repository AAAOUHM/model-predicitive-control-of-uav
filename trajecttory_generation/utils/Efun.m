function f = Efun(time2, simout, p)
    % Extract state and control variables from simout matrix
    x13 = simout(:,13);
    x14 = simout(:,14);
    x15 = simout(:,15);
    x16 = simout(:,16);
% %     for gpops
%     u1 = simout(:,21);
%     u2 = simout(:,22);
%     u3 = simout(:,23);

%     u4 = simout(:,24);
% %  for mpc
    u1 = simout(:,17);
    u2 = simout(:,18);
    u3 = simout(:,19);
    u4 = simout(:,20);
    
    % Compute the expression P
    P = 4*p.c4 + p.c3*(x13 + x14 + x15 + x16) + ...
        p.c2*(x13.*x13 + x14.*x14 + x15.*x15 + x16.*x16) + ...
        p.c1*(u1.*u1 + u2.*u2 + u3.*u3 + u4.*u4) + ...
        p.c7*(x13.*x13.*x13 + x14.*x14.*x14 + x15.*x15.*x15 + x16.*x16.*x16) + ...
        p.c8*(x13.*x13.*x13.*x13 + x14.*x14.*x14.*x14 + ...
              x15.*x15.*x15.*x15 + x16.*x16.*x16.*x16);

    % Define interpolation function
    fun = @(t, y) interp1(time2, P, t);

    % Integrate using ode45
    [t, E] = ode45(fun, time2', 0);
    f = E(end);
end
%%% write this:
% simout=[state2 control2];
% Efun(time2,simout,p)
% 
% simout=[xx(:,1:end-1) u_cl];

% simout=[xx' [u_cl;u_cl(end,:)]];
%  Efun(time_steps,simout,p)
% 
% % % % % % % % % % % % % % % % % % % % % % % % % 
% % 1st run rhuis code
% simout=[xx(:,1:end-1)' [u_cl]];
%  Efun(time_steps,simout,p)
% 







