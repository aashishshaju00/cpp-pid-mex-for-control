%DEMO_STEP_RESPONSE Demonstrates pid_mex used in a MATLAB simulation loop.
clc;
close all;

% Build MEX if missing
if exist("pid_mex", "file") ~= 3
    build_mex();
end

% PID parameters (these are not tuned parameters)
% Currently i have just used trial and error to showcase saturation and
% other important concepts.

params.Kp = 100;
params.Ki = 80;
params.Kd = 0.5;
params.u_min = -1;
params.u_max =  1;
params.enable_anti_windup = true;
params.deriv_filter_hz = 10;
pid_mex('init', params);

% Simulation settings
dt = 0.001;
T  = 3;
N  = round(T/dt) + 1;
t  = (0:N-1) * dt;

% Plant: first-order, ydot = (-y + u)/tau
tau = 0.08;

% Setpoint: step at 0.1s
r = zeros(1, N);
r(t >= 0.1) = 1.0;

% Preallocate logs
y  = zeros(1, N);
u  = zeros(1, N);
pu = zeros(1, N);
iu = zeros(1, N);
du = zeros(1, N);
sat = false(1, N);

for k = 2:N
    [out, dbg] = pid_mex('step', r(k), y(k-1), dt);

    u(k)   = out;
    pu(k)  = dbg.p;
    iu(k)  = dbg.i;
    du(k)  = dbg.d;
    sat(k) = dbg.saturated;

    % Plant integration (Euler)
    ydot = (-y(k-1) + u(k)) / tau;
    y(k) = y(k-1) + dt * ydot;
end

% Plot
figure;
plot(t, r, t, y);
grid on;
xlabel("Time (s)");
ylabel("r, y");
legend("Setpoint r", "Output y");

figure;
plot(t, u);
grid on;
xlabel("Time (s)");
ylabel("Control u");
title("Controller output (saturated)");

figure;
plot(t, pu, t, iu, t, du);
grid on;
xlabel("Time (s)");
ylabel("Terms");
legend("P", "I", "D");

figure;
stairs(t, double(sat));
grid on;
xlabel("Time (s)");
ylabel("Saturation flag");
ylim([-0.1 1.1]);