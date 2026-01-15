**example_model.slx**

**Multirate Simulink harness for C++ PID (MEX-based)**

**Purpose of this model**

This Simulink model is not meant to be a control design environment. It
is a **deployment-style sample test environment** for the C++ PID
controller that lives in PidController.cpp and is exposed to MATLAB via
pid_mex.

The goals of this model are:

- Exercise the exact same C++ controller that would run in embedded code

- Run the controller at a realistic control rate (100 Hz)

- Run the plant at a higher simulation rate (1000 Hz)

- Drive the system with a realistic, time-varying reference

- Verify that the controller tracks correctly across steps, ramps, and
  sinusoids

This makes the Simulink model behave like a real digital control system,
not like a continuous-time academic example.

**High-level structure**

The model has three major pieces:

1.  **Reference Generator**  
    Produces a time-varying reference ref(t) using steps, ramps, and
    sinusoidal wiggles.

2.  **Discrete-time Controller (100 Hz)**  
    A MATLAB Function block that calls the compiled C++ PID through
    pid_mex.

3.  **Discrete-time Plant (1000 Hz)**  
    A fast simulated plant that represents the physical system being
    controlled.

These three pieces are tied together by explicit multirate blocks so
that timing is handled deterministically.

**Controller initialization and reset**

The C++ controller inside pid_mex is stateful. It owns:

- the integrator

- the derivative filter state

- the previous error and measurement

That means it must be explicitly initialized and reset.

This is done from Simulink using **model callbacks**.

**InitFcn (model start)**

When the model starts, this code runs:

“

params.Kp = 98.4128;

params.Ki = 0.4971;

params.Kd = 64.7658;

params.u_min = -10;

params.u_max = 10;

params.enable_anti_windup = true;

params.deriv_filter_hz = 1.3029/(2\*pi);

pid_mex('init', params);

”

This does three critical things:

1.  Creates a fresh C++ PidController instance inside the MEX

2.  Loads all gains and limits into C++ memory

3.  Resets the integrator, derivative filter, and internal state

From this point on, every pid_mex('step', ...) call uses this same
controller instance.

**StopFcn (model stop)**

When the model stops:

“

pid_mex('reset');

”

This clears the persistent controller inside the MEX.  
This prevents state leakage between runs and guarantees repeatability.

Without this, stopping and restarting Simulink would keep the old
integrator and derivative history, which would be physically wrong.

**The controller block: calling C++ from Simulink**

The controller is implemented with a **MATLAB Function block**:

“

function u = controller_100Hz(ref, y_meas)

% Call MEX from MATLAB execution (not codegen)

coder.extrinsic('pid_mex');

dt_ctrl = 0.01;

% Pre-allocate output as a double so Simulink knows the type/size

u = 0.0;

% Call MEX. This executes in MATLAB, not generated code.

u = pid_mex('step', ref, y_meas, dt_ctrl);

end

”

**Why coder.extrinsic is required**

Simulink normally tries to generate C code from MATLAB Function
blocks.  
That would fail here, because pid_mex is a precompiled MEX binary.

coder.extrinsic('pid_mex') tells Simulink:

Do not try to generate code for this.  
Call it in the MATLAB execution engine at runtime.

This lets Simulink run the simulation while the actual control math runs
in our C++ PID implementation.

The effect is that the simulation is driven by real C++ control logic,
not MATLAB PID code.

**Why the controller runs at 100 Hz**

The controller block is set to **D2 = 0.01 s** (100 Hz).

That means:

- Simulink executes controller_100Hz every 10 ms

- Each call advances the C++ PID by exactly one control step

- dt_ctrl = 0.01 matches the actual sample time

This matches a real embedded control loop.

**Why the plant runs at 1000 Hz**

The plant block is set to **D1 = 0.001 s** (1000 Hz).

This gives:

- Higher numerical accuracy for the simulated dynamics

- Smooth output for plotting

- More realistic physics than running the plant at 100 Hz

This is common in hardware-in-the-loop and SiL setups.

**How multirate is handled**

The two rates are connected explicitly.

**Controller output → Plant input**

The controller produces u at 100 Hz.

That signal goes through a **Zero-Order Hold (Ts = 0.001)** before
entering the 1000 Hz plant.

This means:

- The control signal is held constant for 10 plant steps

- The plant sees piecewise-constant actuation, exactly like a DAC

This is how digital controllers drive continuous plants in the real
world.

**Plant output → Controller input**

The plant produces y at 1000 Hz.

That signal goes through:

- a **Unit Delay (z⁻¹)** to break algebraic loops

- a **Rate Transition / ZOH at 0.01 s**

This resamples the fast plant output down to 100 Hz so the controller
always sees a clean, synchronous measurement.

Simulink handles all buffering and rate alignment for us.
