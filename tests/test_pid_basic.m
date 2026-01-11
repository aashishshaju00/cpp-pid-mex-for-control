%% test_pid_basic.m
% Basic + advanced behavioral tests for pid_mex + PidController
%
% This file is meant to be a sanity check suite:
%   - does pid_mex return the right outputs
%   - do P / I / D behave like expected in simple scenarios
%   - does saturation clamp
%   - does conditional anti-windup actually stop windup
%   - does derivative filtering reduce noise sensitivity
%
% If any assert fails, MATLAB throws an error and the test run stops.
clear; clc;
% ------------------------------------------------------------
% Make sure pid_mex exists on path
% ------------------------------------------------------------
% exist(...)=3 means it is a compiled MEX on path.
% If it’s missing, either build_mex didn’t run or MATLAB path is wrong.
if exist("pid_mex", "file") ~= 3
    error("pid_mex MEX not found on MATLAB path. Run tests/run_tests.m or build_mex first.");
end
% ============================================================
% Test A: Output format + debug struct fields
% ============================================================
% This is basically checking the user-facing API:
%   [u, dbg] = pid_mex('step', ...)
% dbg should have the fields we promised in pid_mex.cpp makeOutputStruct()

params = defaultParams();
params.Kp = 1.0;
params.u_min = -10;
params.u_max =  10;

pid_mex('reset');
pid_mex('init', params);

dt = 0.01;
ref = 1.0;
y   = 0.0;

[u, dbg] = pid_mex('step', ref, y, dt);

% small helper lambdas for assertions
assertNear = @(a,b,tol,msg) assert(abs(a-b) <= tol, msg);
assertTrue = @(cond,msg) assert(cond, msg);

% u should be a plain scalar number
assertTrue(isnumeric(u) && isscalar(u), "u must be a numeric scalar.");
% dbg should be a struct
assertTrue(isstruct(dbg), "dbg must be a struct.");

% this helper should assert that all these fields exist
mustHaveFields(dbg, {"u","p","i","d","e","saturated"});

% dbg.u is redundant but should match the scalar u output
assertNear(dbg.u, u, 1e-12, "dbg.u must match u output (within tolerance).");

fprintf("  Test A passed: output format and debug fields.\n");

% ============================================================
% Test B: Pure proportional correctness (Kp only)
% ============================================================
% If Ki=Kd=0, then controller should reduce to:
%   u = Kp * (ref - y)
% This should be exact (no integration, no filtering, no saturation issues)

params = defaultParams();
params.Kp = 2.0;
params.Ki = 0.0;
params.Kd = 0.0;
params.u_min = -100;
params.u_max =  100;
params.enable_anti_windup = true;
params.deriv_filter_hz = 0.0;

pid_mex('reset');
pid_mex('init', params);

dt = 0.05;   % arbitrary positive dt
ref = 1.0;
y   = 0.0;

[u, dbg] = pid_mex('step', ref, y, dt);

% Expected:
%   e = 1
%   p = Kp*e = 2
%   i = 0
%   d = 0
%   u = 2
assertNear(dbg.e, 1.0, 1e-12, "Pure P: error should be 1.");
assertNear(dbg.p, 2.0, 1e-12, "Pure P: p term should be Kp*e = 2.");
assertNear(dbg.i, 0.0, 1e-12, "Pure P: i term should be 0.");
assertNear(dbg.d, 0.0, 1e-9,  "Pure P: d term should be ~0."); % derivative on first sample can be tiny
assertNear(u,     2.0, 1e-12, "Pure P: u should be 2 (no saturation).");
assertTrue(~dbg.saturated, "Pure P: should not be saturated with wide limits.");

fprintf("  Test B passed: pure proportional correctness.\n");

% ============================================================
% Test C: Integral accumulation with constant error (Ki only)
% ============================================================
% With Kp=0, Kd=0, Ki=1:
%   integral[k] = integral[k-1] + e*dt
%   i_term = Ki * integral
% With constant error e=1, i_term should grow roughly linearly vs time.

params = defaultParams();
params.Kp = 0.0;
params.Ki = 1.0;
params.Kd = 0.0;
params.u_min = -100;
params.u_max =  100;
params.enable_anti_windup = true;
params.deriv_filter_hz = 0.0;

pid_mex('reset');
pid_mex('init', params);

dt = 0.01;
N  = 200;
ref = 1.0;
y   = 0.0;

Iterm = zeros(1, N);
U     = zeros(1, N);

for k = 1:N
    [u, dbg] = pid_mex('step', ref, y, dt);
    U(k) = u;
    Iterm(k) = dbg.i;
end

% check monotonic increase (allow tiny tolerance for floating point)
tol = 1e-12;
nondecreasing = all(diff(Iterm) >= -tol);
assertTrue(nondecreasing, "Ki-only: I term should be nondecreasing for constant positive error.");

% rough magnitude check at the end:
% expected integral ~= N*dt*e
% with Ki=1, i_term should be about N*dt
expectedEnd = N * dt;            % since e=1, Ki=1
actualEnd   = Iterm(end);

% keep this wide, it's not meant to be super strict
assertTrue(actualEnd > 0.5*expectedEnd, "Ki-only: I term too small; integration may be wrong.");
assertTrue(actualEnd < 1.5*expectedEnd, "Ki-only: I term too large; integration may be wrong.");

fprintf("  Test C passed: integral accumulation under constant error.\n");

% ============================================================
% Test D: Saturation clamps output (upper and lower)
% ============================================================
% Here, we force u_unsat to be huge so clamp must activate.

params = defaultParams();
params.Kp = 1000.0;
params.Ki = 0.0;
params.Kd = 0.0;
params.u_min = -1.0;
params.u_max =  1.0;
params.enable_anti_windup = true;
params.deriv_filter_hz = 0.0;

pid_mex('reset');
pid_mex('init', params);

dt = 0.01;

% Upper saturation case: ref positive -> error positive -> u wants to go big positive
ref = 1.0; y = 0.0;
[u, dbg] = pid_mex('step', ref, y, dt);
assertNear(u, params.u_max, 1e-12, "Upper sat: u must clamp to u_max.");
assertTrue(dbg.saturated, "Upper sat: saturated flag must be true.");

% Lower saturation case: ref negative -> u wants to go big negative
ref = -1.0; y = 0.0;
[u, dbg] = pid_mex('step', ref, y, dt);
assertNear(u, params.u_min, 1e-12, "Lower sat: u must clamp to u_min.");
assertTrue(dbg.saturated, "Lower sat: saturated flag must be true.");

fprintf("  Test D passed: saturation clamping and flag.\n");

% ============================================================
% Test E: Conditional integration anti-windup blocks windup
% ============================================================
% Scenario:
%   - actuator saturates high (u = u_max)
%   - error stays positive
% conditional integration rule should reject integration in that situation.
%
% Expected behavior:
%   integrator state should remain basically constant

params = defaultParams();
params.Kp = 1000.0;      % force saturation
params.Ki = 5.0;         % would wind up quickly if anti-windup fails
params.Kd = 0.0;
params.u_min = -1.0;
params.u_max =  1.0;
params.enable_anti_windup = true;
params.deriv_filter_hz = 0.0;

pid_mex('reset');
pid_mex('init', params);

dt = 0.01;
N  = 200;

ref = 1.0;
y   = 0.0;

integralLog = zeros(1, N);

for k = 1:N
    % Keep plant output fixed (y=0) so error never changes sign.
    pid_mex('step', ref, y, dt);

    % Query internal state from the controller (expensive but fine here)
    st = pid_mex('getState');
    integralLog(k) = st.integral;
end

% integrator should not drift much
deltaI = integralLog(end) - integralLog(1);
assertTrue(abs(deltaI) < 1e-6, "Anti-windup: integrator changed under sustained saturation; expected near-constant.");

fprintf("  Test E passed: conditional integration blocks windup under saturation.\n");

% ============================================================
% Test F: Derivative filter reduces D-term RMS under noisy measurement
% ============================================================
% This test isolates the D path only:
%   Kp=Ki=0, Kd=1
% ref = 0
% meas = noise
%
% derivative of noise has huge variance, so unfiltered D should be big.
% filtered D should have lower RMS after startup.

dt = 0.001;
N  = 4000;
rng(0);

measNoise = 0.05 * randn(1, N);
ref = 0.0;

% ----------------------------
% Case 1: D filter OFF
% ----------------------------
params = defaultParams();
params.Kp = 0.0;
params.Ki = 0.0;
params.Kd = 1.0;
params.u_min = -100;
params.u_max =  100;
params.enable_anti_windup = true;
params.deriv_filter_hz = 0.0;

pid_mex('reset');
pid_mex('init', params);

dOff = zeros(1, N);

for k = 2:N
    [~, dbg] = pid_mex('step', ref, measNoise(k-1), dt);
    dOff(k) = dbg.d;
end

% ----------------------------
% Case 2: D filter ON
% ----------------------------
% cutoff choice here is arbitrary-ish, just enough to reduce RMS
params.deriv_filter_hz = 15.0;

pid_mex('reset');
pid_mex('init', params);

dOn = zeros(1, N);

for k = 2:N
    [~, dbg] = pid_mex('step', ref, measNoise(k-1), dt);
    dOn(k) = dbg.d;
end

% RMS comparison
% ignore first few samples because filter starts from 0 and needs to settle
skip = 50;
rmsOff = sqrt(mean(dOff(skip:end).^2));
rmsOn  = sqrt(mean(dOn(skip:end).^2));

assertTrue(rmsOn < rmsOff, "Derivative filter: expected RMS(d_filtered) < RMS(d_unfiltered).");

fprintf("  Test F passed: derivative filter reduces D-term RMS under noise.\n");

fprintf("\nAll tests in test_pid_basic.m PASSED.\n");
%% ---------- Helpers ----------

function mustHaveFields(s, fields)
    for i = 1:numel(fields)
        f = fields{i};
        assert(isfield(s, f), "Missing field: " + f);
    end
end

function params = defaultParams()
    params = struct();
    params.Kp = 0.0;
    params.Ki = 0.0;
    params.Kd = 0.0;
    params.u_min = -Inf;
    params.u_max =  Inf;
    params.enable_anti_windup = true;
    params.deriv_filter_hz = 0.0;
end