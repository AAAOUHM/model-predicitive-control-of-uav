function [xx, u_cl, time_steps, k] = mtpmpc3tuning(k, x0, state2, control2, time2)
    % Add CasADi path and import
    addpath('C:\Users\Desktop\casadi-3.6.5-windows64-matlab2018b')
    import casadi.*

    % Precompute reference derivatives using analytical dynamics
    kb = 3.8305e-06; % Thrust coefficient
    mass = 1.30; % Mass

    sim_tim = 12; % simulation time
    n_states = 16;
    
    % Compute scaling factors based on the maximum absolute values of state2
    scaling = max(abs(state2), [], 1); % Max of each column (state) over all rows (time steps)
    scaling = max(scaling, 1e-6); % Avoid division-by-zero in Q computation
    
    % Apply scaling to Q
    Q = eye(n_states, n_states);
    for i = 1:16
        Q(i,i) = k(i) / (scaling(i)^2);
    end
    
      % Terminal weight (same as Q for simplicity)
    Q_N = 0*Q; 

%    Q IS DIFFERENT 
    for i = 7:16
        Q(i,i) =0*k(i) / (scaling(i)^2);
    end
    for i = 2:2:6
        Q(i,i) = 0*k(i) / (scaling(i)^2);
    end
    

    T = 0.08; % [s]
    u1 = SX.sym('u1'); u2 = SX.sym('u2'); u3 = SX.sym('u3'); u4 = SX.sym('u4');
    controls = [u1; u2; u3; u4]; n_controls = length(controls);
    R = k(17) * eye(n_controls, n_controls);
    N = k(18); % Prediction horizon

    % Drone params
    mass = 1.30;
    grv = 9.8;
    kb = 3.8305e-06;
    Ix = 0.081; Iy = 0.081; Iz = 0.142;
    l = 0.35 / 2;
    J = 3.9454e-05;
    ktau = 2.2518e-08;
    omegahvr = sqrt(mass * grv / (4 * kb)); % 911.85

    % Constraints on inputs
    u_max = 1000; u_min = -u_max;
    omega_max = 1047; omega_min = 0;
    zMAX = 1500; zMIN = 0;
    xMAX = 100; xMIN = -100;
    yMAX = 100; yMIN = -100;
    deg = 35;
    rMAX = (pi/180) * deg; rMIN = (pi/180) * -deg;
    pMAX = (pi/180) * deg; pMIN = (pi/180) * -deg;
    yawMAX = inf; yawMIN = -inf;
    rdMAX = 1.5; rdMIN = -1.5;
    pdMAX = 1.5; pdMIN = -1.5;
    yawdMAX = 0.1; yawdMIN = -0.1;

    x1 = SX.sym('x1'); x2 = SX.sym('x2'); x3 = SX.sym('x3');
    x4 = SX.sym('x4'); x5 = SX.sym('x5'); x6 = SX.sym('x6');
    x7 = SX.sym('x7'); x8 = SX.sym('x8'); x9 = SX.sym('x9');
    x10 = SX.sym('x10'); x11 = SX.sym('x11'); x12 = SX.sym('x12');
    x13 = SX.sym('x13'); x14 = SX.sym('x14'); x15 = SX.sym('x15'); x16 = SX.sym('x16');
    states = [x1; x2; x3; x4; x5; x6; x7; x8; x9; x10; x11; x12; x13; x14; x15; x16]; n_states = length(states);
   
    df = T / 0.01;
    staten = state2(1:df:end, :); % Downsampled state reference
    controln = control2(1:df:end, :); % Downsampled control reference
    ur_all = controln'; % Reference control inputs
    xr_all = [staten(:,1:16)]'; % Reference state trajectory

    rhs = [x2;
           (kb / mass) *( sin(x7) * sin(x11) + cos(x7) * cos(x11) * sin(x9) )* (x13^2 + x14^2 + x15^2 + x16^2);
           x4;
           (kb / mass) * (cos(x7) * sin(x9) * sin(x11) - sin(x7) * cos(x11)) * (x13^2 + x14^2 + x15^2 + x16^2);
           x6;
           (kb / mass) * cos(x9) * cos(x7) * (x13^2 + x14^2 + x15^2 + x16^2) - grv;
           x8;
           ((Iy - Iz) / Ix) * x10 * x12 + (l * kb / Ix) * (x14^2 - x16^2) - (J / Ix) * x10 * (x13 - x14 + x15 - x16);
           x10;
           ((Iz - Ix) / Iy) * x8 * x12 + (l * kb / Iy) * (x15^2 - x13^2) + (J / Iy) * x8 * (x13 - x14 + x15 - x16);
           x12;
           ((Ix - Iy) / Iz) * x8 * x10 + (ktau / Iz) * (x13^2 - x14^2 + x15^2 - x16^2);
           u1;
           u2;
           u3;
           u4];
N;
    f = Function('f', {states, controls}, {rhs});
    U = SX.sym('U', n_controls, N);
    P = SX.sym('P', n_states + N * (n_states + n_controls));
    X = SX.sym('X', n_states, (N + 1));

    obj = 0;
    g = [];
    st = X(:, 1);
    g = [g; st - P(1:n_states)];

    for kk = 1:N
        st = X(:, kk); con = U(:, kk);
        obj = obj + (st - P((n_states + n_controls) * kk - 3:(n_states + n_controls) * kk + (n_states - 1) - 3))' * Q * (st - P((n_states + n_controls) * kk - 3:(n_states + n_controls) * kk + (n_states - 1) - 3)) + ...
              (con - P((n_states + n_controls) * kk + (n_states - 1) - 3 + 1:(n_states + n_controls) * kk + (n_states - 1) - 3 + 1 + (n_controls - 1)))' * R * (con - P((n_states + n_controls) * kk + (n_states - 1) - 3 + 1:(n_states + n_controls) * kk + (n_states - 1) - 3 + 1 + (n_controls - 1)));
        st_next = X(:, kk + 1);
        k1 = f(st, con);
        k2 = f(st + T/2 * k1, con);
        k3 = f(st + T/2 * k2, con);
        k4 = f(st + T * k3, con);
        st_NEXTRK = st + T/6 * (k1 + 2 * k2 + 2 * k3 + k4);
        g = [g; st_next - st_NEXTRK];
    end

    % Add terminal cost
    st_N = X(:, N+1);
    obj = obj + (st_N - P((n_states + n_controls) * N - 3:(n_states + n_controls) * N + (n_states - 1) - 3))' * Q_N * (st_N - P((n_states + n_controls) * N - 3:(n_states + n_controls) * N + (n_states - 1) - 3));

    OPT_variables = [reshape(X, n_states * (N + 1), 1); reshape(U, n_controls * N, 1)];
    nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

    opts = struct;
    opts.ipopt.max_iter = 500; % Increased for better convergence
    opts.ipopt.print_level = 0; % Enable output for debugging
    opts.print_time = 0;
    opts.ipopt.acceptable_tol = 1e-6; % Relaxed tolerance
    opts.ipopt.acceptable_obj_change_tol = 1e-4;
    opts.ipopt.linear_scaling_on_demand = 'yes'; % Enable scaling

    solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

    args = struct;
    args.lbg = zeros(n_states + n_states * N, 1);
    args.ubg = zeros(n_states + n_states * N, 1);
    args.lbg(1:n_states) = 0;
    args.ubg(1:n_states) = 0;
    for kk = 1:N
        args.lbg(n_states + 1 + (kk-1) * n_states : n_states + kk * n_states) = 0;
        args.ubg(n_states + 1 + (kk-1) * n_states : n_states + kk * n_states) = 0;
    end

    args.lbx = [];
    args.ubx = [];
    for kk = 1:N+1
        args.lbx = [args.lbx; xMIN];
        args.ubx = [args.ubx; xMAX];
        args.lbx = [args.lbx; -15];
        args.ubx = [args.ubx; 15];
        args.lbx = [args.lbx; yMIN];
        args.ubx = [args.ubx; yMAX];
        args.lbx = [args.lbx; -15];
        args.ubx = [args.ubx; 15];
        args.lbx = [args.lbx; zMIN];
        args.ubx = [args.ubx; zMAX];
        args.lbx = [args.lbx; -2];
        args.ubx = [args.ubx; 6];
        args.lbx = [args.lbx; rMIN];
        args.ubx = [args.ubx; rMAX];
        args.lbx = [args.lbx; rdMIN];
        args.ubx = [args.ubx; rdMAX];
        args.lbx = [args.lbx; pMIN];
        args.ubx = [args.ubx; pMAX];
        args.lbx = [args.lbx; pdMIN];
        args.ubx = [args.ubx; pdMAX];
        args.lbx = [args.lbx; yawMIN];
        args.ubx = [args.ubx; yawMAX];
        args.lbx = [args.lbx; yawdMIN];
        args.ubx = [args.ubx; yawdMAX];
        args.lbx = [args.lbx; 0];
        args.ubx = [args.ubx; omega_max];
        args.lbx = [args.lbx; 0];
        args.ubx = [args.ubx; omega_max];
        args.lbx = [args.lbx; 0];
        args.ubx = [args.ubx; omega_max];
        args.lbx = [args.lbx; 0];
        args.ubx = [args.ubx; omega_max];
    end
    for kk = 1:N
        args.lbx = [args.lbx; u_min];
        args.ubx = [args.ubx; u_max];
        args.lbx = [args.lbx; u_min];
        args.ubx = [args.ubx; u_max];
        args.lbx = [args.lbx; u_min];
        args.ubx = [args.ubx; u_max];
        args.lbx = [args.lbx; u_min];
        args.ubx = [args.ubx; u_max];
    end
% n_states=16;
% omegahvr=911.85;
% x0 = [0;0;0;zeros(n_states-4-3,1);ones(4,1)*omegahvr];

    t0 = 0;
    xx(:, 1) = x0;
    t(1) = t0;
    u0 = zeros(N, n_controls);
    X0 = repmat(x0, 1, N + 1)';
    mpciter = 0;
    xx1 = [];
    u_cl = [];
    iteration_times = [];
    store_refxyz = [];

    main_loop = tic;
    while(mpciter < sim_tim / T)
        iteration_start_time = tic;
        current_time = mpciter * T;
        args.p(1:n_states) = x0;
        for kk = 1:N
            t_predict = current_time + (kk-1) * T;
            if (kk + mpciter) <= size(xr_all, 2)
                x_ref = xr_all(1, kk + mpciter);
                xd_ref = xr_all(2, kk + mpciter);
                y_ref = xr_all(3, kk + mpciter);
                yd_ref = xr_all(4, kk + mpciter);
                z_ref = xr_all(5, kk + mpciter);
                zd_ref = xr_all(6, kk + mpciter);
                r_ref = xr_all(7, kk + mpciter);
                rd_ref = xr_all(8, kk + mpciter);
                p_ref = xr_all(9, kk + mpciter);
                pd_ref = xr_all(10, kk + mpciter);
                yaw_ref = xr_all(11, kk + mpciter);
                yawd_ref = xr_all(12, kk + mpciter);
                x13_ref = xr_all(13, kk + mpciter);
                x14_ref = xr_all(14, kk + mpciter);
                x15_ref = xr_all(15, kk + mpciter);
                x16_ref = xr_all(16, kk + mpciter);
                u1_ref = ur_all(1, kk + mpciter);
                u2_ref = ur_all(2, kk + mpciter);
                u3_ref = ur_all(3, kk + mpciter);
                u4_ref = ur_all(4, kk + mpciter);
            else
                x_ref = xr_all(1, end);
                xd_ref = 0;
                y_ref = xr_all(3, end);
                yd_ref = 0;
                z_ref = xr_all(5, end);
                zd_ref = 0;
                r_ref = 0;
                rd_ref = 0;
                p_ref = 0;
                pd_ref = 0;
                yaw_ref = 0;
                yawd_ref = 0;
                x13_ref = omegahvr;
                x14_ref = omegahvr;
                x15_ref = omegahvr;
                x16_ref = omegahvr;
                u1_ref = 0;
                u2_ref = 0;
                u3_ref = 0;
                u4_ref = 0;
            end
            args.p((n_states + n_controls) * kk - 3:(n_states + n_controls) * kk + (n_states - 1) - 3) = [x_ref, xd_ref, y_ref, yd_ref, ...
                                                                                                         z_ref, zd_ref, r_ref, rd_ref, p_ref, pd_ref, yaw_ref, yawd_ref, ...
                                                                                                         x13_ref, x14_ref, x15_ref, x16_ref];
            args.p((n_states + n_controls) * kk + (n_states - 1) - 3 + 1:(n_states + n_controls) * kk + (n_states - 1) - 3 + 1 + (n_controls - 1)) = [u1_ref, u2_ref, u3_ref, u4_ref];
        end
        if mpciter < size(xr_all, 2)
            store_refxyz = [store_refxyz [xr_all(1, end); xr_all(3, end); xr_all(5, end)]];
        end
        args.x0 = [reshape(X0', n_states * (N + 1), 1); reshape(u0', n_controls * N, 1)];
        sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, ...
                     'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);
        u = reshape(full(sol.x(n_states * (N + 1) + 1:end))', n_controls, N)';
        xx1(:, :, mpciter + 1) = reshape(full(sol.x(1:n_states * (N + 1)))', n_states, N + 1)';
        u_cl = [u_cl; u(1, :)];
        t(mpciter + 1) = t0;
        [t0, x0, u0] = shift(T, t0, x0, u, f);
        xx(:, mpciter + 2) = x0;
        X0 = reshape(full(sol.x(1:n_states * (N + 1)))', n_states, N + 1)';
        X0 = [X0(2:end, :); X0(end, :)];
        mpciter = mpciter + 1;
        iteration_times = [iteration_times, toc(iteration_start_time)];
    end
    main_loop_time = toc(main_loop);
    average_mpc_time = main_loop_time / (mpciter + 1);
    worst_time = max(iteration_times);

    % Compute time_steps
    time_steps = 0:T:(size(u_cl, 1) - 1) * T;
end

function [t0, x0, u0] = shift(T, t0, x0, u, f)
    st = x0;
    con = u(1, :)';
    k1 = f(st, con);
    k2 = f(st + T/2 * k1, con);
    k3 = f(st + T/2 * k2, con);
    k4 = f(st + T * k3, con);
    st = st + T/6 * (k1 + 2 * k2 + 2 * k3 + k4);
    x0 = full(st);
    t0 = t0 + T;
    u0 = [u(2:size(u, 1), :); u(size(u, 1), :)];
end

% best_k =
% 
%   Columns 1 through 12
% 
%     2.1926    3.1656    8.0431    6.6871    6.3159    7.3586  213.6308  100.7875  378.6053  115.9890  233.3138  496.2620
% 
%   Columns 13 through 18
% 
%          0         0         0         0    0.0000   20.0000

% k=[4.6, 1.2, 0.1, 3.9, 9.6, 9.9, 367.5, 417.8, 148.3, 161.5, 156.5, 299.8, 0.0, 18.0, ] | Cost=3.55
% best_k =
% 
%   Columns 1 through 12
% 
%     9.8563    8.3953    0.9024    3.4231    9.7582    9.7888  979.3509  858.2294   21.1315   59.8899    2.0090   35.9708
% 
%   Columns 13 through 18
% 
%          0         0         0         0    0.0000   20.0000  
% 
% ans =k(17)=
% 
%    9.0000e-08
% by all attitudes=0(good weights)
% k=[14968004.3, 67219418.7, 38209510.4, 15276193.1, 73143133.9, 27155404.8, 380.9, 558.4, 558.7, 677.2, 90.7, 204.3, 0.1, 14.0, ]
% by all atiitudes zeros and x0 is non zero 2
% k=[14968004.3, 67219418.7, 38209510.4, 15276193.1, 73143133.9, 27155404.8, 380.9, 558.4, 558.7, 677.2, 90.7, 204.3,0,0,0,0, 0.1, 14.0, ]
% for x0=0 and only position weights
%  k=[99392.7, 0.0, 292878.2, 0.0, 919851.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 14.0, ] 
%  with t cost
% k=[409331.6, 37.1, 918058.8, 84.3, 83002.8, 92.1, 835.6, 510.2, 227.4, 218.0, 0.0, 0.0, 0 ,00 0 ,0,0.1, 14.0, ]
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % NOW ITAE WEIGHTS ALL ZERO EXCEPT POSITION wiht=1000
% k=[144140.1, 6.1, 988940.3, 96.7, 901468.4, 91.6, 984.2, 358.9, 896.5, 385.7, 0.0, 0.0,0 0 0 0  0.0, 18.0, ]

% same with incresed bounds

% k=[9643631.3, 0.0, 8599580.2, 0.0, 9016314.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 18.0, ] | Cost=24.40
% best_k(17)
% 
% ans =
% 
%    1.5128e-04