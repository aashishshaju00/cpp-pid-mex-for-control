%% demo_tracking_noise.m
% Tracking a time-varying reference with measurement noise.
% Goal of this demo:
%   Show why the D term gets ugly with noise and how a simple D low-pass helps
%
% This uses the pid_mex interface:
%   pid_mex('init', paramsStruct)
%   [u, dbg] = pid_mex('step', ref, meas, dt)
%   st = pid_mex('getState')   (optional)
%
% Two runs:
%   1) derivative filter OFF
%   2) derivative filter ON

clear; close all;

% ------------------------------------------------------------
% Build if needed
% ------------------------------------------------------------
% exist(...)=3 means "MEX function exists"
% if you just cloned repo or cleaned binaries, this will compile it.
if exist("pid_mex", "file") ~= 3
    build_mex();
end

% ------------------------------------------------------------
% Simulation settings
% ------------------------------------------------------------
dt = 0.001;      % controller update (1 kHz)
T  = 8.0;        % total sim time
N  = round(T/dt) + 1;
t  = (0:N-1) * dt;

rng(42);         % fix random seed so noise is repeatable each run

% ------------------------------------------------------------
% Plant model (simple first order)
% ------------------------------------------------------------
% Continuous model:
%   ydot = (-y + u)/tau
% This is a "toy plant" because it is stable and easy to integrate.
tau = 0.15;

% ------------------------------------------------------------
% Reference signal
% ------------------------------------------------------------
% I want something that moves but not too fast.
% ramp makes it not settle to a constant
% sine makes it oscillate gently
r = 0.2*t + 0.6*sin(2*pi*0.35*t);

% ------------------------------------------------------------
% Measurement noise
% ------------------------------------------------------------
% This is the important part:
% D term amplifies high freq stuff, so noise causes chatter.
noise_std = 0.03;

% ------------------------------------------------------------
% Cases to compare
% ------------------------------------------------------------
cases(1).name = "D filter OFF";
cases(1).deriv_filter_hz = 0.0;

cases(2).name = "D filter ON";
% NOTE: deriv_filter_hz is cutoff in Hz.
% If this is too low, D becomes sluggish.
% If too high, noise leaks in.
cases(2).deriv_filter_hz = 1.3029/(2*pi);   % try 5, 15, 30 (Hz) depending on dt/plant/noise

results = struct([]);

% ============================================================
% Main loop over both controller cases
% ============================================================
for ii = 1:numel(cases)

    % --------------------------------------------------------
    % Controller parameters
    % --------------------------------------------------------
    % Field names must match pid_mex parseParams() fields.
    % Anything missing will fall back to defaults inside C++.
    p = struct();
    p.Kp = 98.4128;  % not tuned properly see the docs file for more explanation
    p.Ki = 0.4971;
    p.Kd = -64.7658;

    % u limits represent actuator saturation
    % give enough authority to track the ramp
    p.u_min = -10;
    p.u_max =  10;

    % enable conditional-integration anti-windup
    p.enable_anti_windup = true;

    % case dependent D filter cutoff
    p.deriv_filter_hz    = cases(ii).deriv_filter_hz;

    % --------------------------------------------------------
    % Reset + init
    % --------------------------------------------------------
    % reset clears controller internal memory
    % init sets params and also resets again (in our code)
    % calling both is a bit redundant but explicit and safe.
    pid_mex('reset');
    pid_mex('init', p);

    % --------------------------------------------------------
    % Logs for plotting
    % --------------------------------------------------------
    % y_true = plant true state
    % y_meas = noisy measurement fed to controller
    y_true = zeros(1, N);
    y_meas = zeros(1, N);

    % control and term breakdown from debug struct
    u   = zeros(1, N);
    pT  = zeros(1, N);
    iT  = zeros(1, N);
    dT  = zeros(1, N);
    e   = zeros(1, N);
    sat = false(1, N);

    % integrator state sampled from getState (not every step)
    integ = zeros(1, N);

    % ========================================================
    % Time stepping
    % ========================================================
    for jj = 2:N
        % --------------------------------------------
        % noisy measurement
        % --------------------------------------------
        % Use jj-1 because y_true(jj) is not computed yet.
        y_meas(jj-1) = y_true(jj-1) + noise_std * randn();

        % --------------------------------------------
        % controller step
        % --------------------------------------------
        % ref uses r(jj), measurement uses y_meas(jj-1)
        %
        % pid_mex returns:
        %   u (scalar)
        %   dbg struct with {u,p,i,d,e,saturated}
        [u(jj), dbg] = pid_mex('step', r(jj), y_meas(jj-1), dt);

        % grab debug terms so we can see whats happening inside PID
        pT(jj)  = dbg.p;
        iT(jj)  = dbg.i;
        dT(jj)  = dbg.d;
        e(jj)   = dbg.e;
        sat(jj) = dbg.saturated;

        % --------------------------------------------
        % plant integration (Euler)
        % --------------------------------------------
        % ydot = (-y + u)/tau
        % y(jj) = y(jj-1) + dt * ydot
        ydot = (-y_true(jj-1) + u(jj)) / tau;
        y_true(jj) = y_true(jj-1) + dt * ydot;

        % --------------------------------------------
        % Optional internal state read (expensive)
        % --------------------------------------------
        % Calling getState every step is unnecessary and slower.
        % so sample it sometimes just to show that the state is accessible.
        if mod(jj, 50) == 0
            st = pid_mex('getState');
            integ(jj) = st.integral;
        else
            integ(jj) = integ(jj-1);
        end
    end

    % last measurement point (mainly for plotting)
    y_meas(end) = y_true(end) + noise_std * randn();

    % pack results
    results(ii).name  = cases(ii).name;
    results(ii).t     = t;
    results(ii).r     = r;
    results(ii).ytrue = y_true;
    results(ii).ymeas = y_meas;
    results(ii).u     = u;
    results(ii).pT    = pT;
    results(ii).iT    = iT;
    results(ii).dT    = dT;
    results(ii).e     = e;
    results(ii).sat   = sat;
    results(ii).integ = integ;
end

% ============================================================
% Plots
% ============================================================

% ------------------------------------------------------------
% Tracking plot
% ------------------------------------------------------------
% Compare true plant outputs (not noisy meas) against reference.
figure;
plot(results(1).t, results(1).r, 'linestyle', '-','color','k'); hold on;
plot(results(1).t, results(1).ytrue, 'linestyle','--','color','r');
plot(results(2).t, results(2).ytrue, 'linestyle',':','color','c');
grid on;
xlabel("Time (s)"); ylabel("r, y");
legend("Reference r", results(1).name, results(2).name, "Location", "best");
title("Tracking (true output)");

% ------------------------------------------------------------
% Control effort
% ------------------------------------------------------------
% If D is noisy, u will chatter a lot (even if y still tracks ok).
figure;
plot(results(1).t, results(1).u); hold on;
plot(results(2).t, results(2).u, '--');
grid on;
xlabel("Time (s)"); ylabel("u");
legend(results(1).name, results(2).name, "Location", "best");
title("Control effort: derivative filtering reduces noise-driven chatter");

% ------------------------------------------------------------
% D-term plot (the main point)
% ------------------------------------------------------------
% This should show unfiltered D being very noisy compared to filtered.
figure;
plot(results(1).t, results(1).dT); hold on;
plot(results(2).t, results(2).dT, '--');
grid on;
xlabel("Time (s)"); ylabel("D term");
legend(results(1).name, results(2).name, "Location", "best");
title("D term: unfiltered vs filtered (noise sensitivity)");

% ------------------------------------------------------------
% Saturation flag
% ------------------------------------------------------------
% Optional debug. Helps confirm whether u limits are active.
figure;
stairs(results(1).t, double(results(1).sat), "LineWidth", 1.0); hold on;
stairs(results(2).t, double(results(2).sat), "LineWidth", 1.0);
grid on;
xlabel("Time (s)"); ylabel("Saturated");
ylim([-0.1 1.1]);
legend(results(1).name, results(2).name, "Location", "best");
title("Saturation events");

% ------------------------------------------------------------
% Integrator state sampled via getState
% ------------------------------------------------------------
% Not continuous logging (since we sampled every 50 steps), but good enough
% to show that internal controller memory is accessible from MATLAB.
figure;
plot(results(1).t, results(1).integ, "LineWidth", 1.0); hold on;
plot(results(2).t, results(2).integ, "LineWidth", 1.0);
grid on;
xlabel("Time (s)"); ylabel("Integrator (sampled)");
legend(results(1).name, results(2).name, "Location", "best");
title("Integrator state sampled from pid_mex('getState')");
