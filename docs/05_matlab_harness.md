**05_matlab_implementation.md**

**MATLAB Harness and Demo Scripts**

This document describes how MATLAB is used as a simulation and
visualization harness around the C++ PID controller. MATLAB does not
implement any control logic. It only builds the MEX file, runs closed
loop simulations, and visualizes internal PID signals returned from C++.

The primary entry point for MATLAB users is the demo script:

matlab/demo_tracking_noise.m

This single file exercises most features of the controller, including
noise sensitivity, derivative filtering, saturation, and anti windup.

**1. Role of MATLAB in the architecture**

MATLAB plays three roles in this repository:

- Compiles the C++ controller into a MEX file

- Provides a time stepping simulation loop

- Visualizes control behavior using plots

All PID math runs inside PidController::step(). MATLAB only calls
pid_mex('step', ...) with scalar values.

This separation allows the controller to be treated as a black box from
MATLAB’s point of view.

**2. Build and initialization workflow**

The demo script begins by ensuring the MEX file exists:

if exist("pid_mex", "file") ~= 3

build_mex();

end

build_mex.m compiles:

- pid_mex.cpp

- PidController.cpp

and places pid_mex.\<mexext\> in the MATLAB folder so it is on the path.

Before running any control loop, the controller must be reset and
initialized:

pid_mex('reset');

pid_mex('init', p);

reset clears the internal state.  
init loads the parameter struct and resets again inside the C++ code.

This guarantees a clean starting condition for each experiment.

**3. Plant and signal model**

The MATLAB harness implements a simple continuous time first order
plant:

``` math
\dot{y} = \frac{- y + u}{\tau}
```
with Euler integration:

ydot = (-y_true(jj-1) + u(jj)) / tau;

y_true(jj) = y_true(jj-1) + dt \* ydot;

This plant is deliberately simple. It is stable, easy to integrate, and
highlights controller behavior rather than plant complexity.

The reference is time varying:

r = 0.2\*t + 0.6\*sin(2\*pi\*0.35\*t);

This avoids steady state conditions and forces the controller to track
continuously.

Measurement noise is added before the controller sees the signal:

y_meas = y_true + noise_std \* randn();

This is what stresses the derivative term.

**4. Controller execution loop**

At every time step, MATLAB calls into the C++ controller:

\[u(jj), dbg\] = pid_mex('step', r(jj), y_meas(jj-1), dt);

The returned values are:

- u(jj) the control signal

- dbg a struct containing internal PID terms

These fields map directly to the PidOutput struct in C++:

pT(jj) = dbg.p;

iT(jj) = dbg.i;

dT(jj) = dbg.d;

e(jj) = dbg.e;

sat(jj) = dbg.saturated;

This allows MATLAB to plot and inspect the internal decomposition of the
controller without reimplementing any equations.

**5. Accessing internal controller state**

The demo also samples the internal integrator state using:

st = pid_mex('getState');

integ(jj) = st.integral;

This shows that:

- The C++ controller maintains state between calls

- MATLAB can inspect that state for debugging and validation

getState is not called every step because it is slower than step.
Sampling every 50 steps is sufficient for plotting.

**6. Comparing derivative filtering cases**

The demo runs two complete simulations:

cases(1).deriv_filter_hz = 0.0; % filter off

cases(2).deriv_filter_hz = ...; % filter on

Everything else is held constant. This isolates the effect of the
derivative filter.

With filtering disabled, noise passes directly into the D term. With
filtering enabled, high frequency noise is attenuated by the first order
low pass inside PidController.

Both results are stored in the results struct for plotting and
comparison.

The same MEX interface used here is also used inside Simulink, making
this MATLAB harness a lightweight, scriptable version of the full
multirate simulation.
