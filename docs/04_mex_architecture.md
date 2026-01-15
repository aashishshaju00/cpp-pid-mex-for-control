MATLAB MEX Interface Architecture

This document describes how the C++ PID controller is exposed to MATLAB
and Simulink through a MEX (matlab executable) interface. All control
logic remains in C++. The MEX layer is only responsible for data
handling b/w Matlab and Cpp, persistence, safety checks, and command
routing.

The MEX file provides a procedural command based API on top of a single
persistent PidController instance.

**1. Design goals**

The MEX layer is designed to provide:

* A single persistent controller instance shared across MATLAB calls
* A command based interface similar to a C API
* Full access to internal PID signals for testing and plotting
* Deterministic behavior suitable for Simulink multirate execution

**2. Persistent controller instance**

The MEX file owns exactly one controller instance. This instance
persists across calls to pid\_mex() inside a MATLAB session.

This design allows MATLAB and Simulink to call pid\_mex('step', ...)
repeatedly without reinitializing the controller.

The g\_is\_init flag enforces that init or reset must be called before the
first step.

**3. Cleanup on MEX unload**

This function is registered with mexAtExit. It runs when the MEX file is
cleared or MATLAB exits. This ensures that controller state is not
accidentally reused across sessions.

**4. Command dispatch model**

All MEX calls follow the pattern:

pid\_mex('command', ...)

The first argument must be a string. The MEX layer routes execution
based on this string.

Supported commands:

|**Command**|**Purpose**|
|-|-|
|init|Initialize controller parameters and reset state|
|step|Advance the controller one time step|
|reset|Reset state while keeping current parameters|
|setParams|Update parameters without resetting state|
|getState|Return the current internal state|

Any unsupported string results in an error.

**5. Parameter parsing**

MATLAB passes parameters as a struct. The MEX layer converts this into a
PidParams struct.

Each field is optional. If a field is not present, the default value
defined in PidParams is preserved.

**6. step command**

This is the main control entry point.

The MEX layer enforces:

* ref, meas, and dt must be real scalar doubles
* The controller must have been initialized

Return values:

* u is the saturated control output
* dbg is a struct containing internal PID signals

**7. MEX build system**

The MEX file is built using matlab/build\_mex.m:

This compiles and links:

* pid\_mex.cpp
* PidController.cpp

and places the resulting pid\_mex.<mexext> into the MATLAB folder so it
is on the MATLAB path.

**8. MATLAB usage pattern**

Typical MATLAB usage:

params.Kp = 100;

params.Ki = 80;

params.Kd = 0.5;

params.u\_min = -1;

params.u\_max = 1;

params.enable\_anti\_windup = true;

params.deriv\_filter\_hz = 10;

pid\_mex('init', params);

\[out, dbg] = pid\_mex('step', ref, meas, dt);

The controller persists between calls, allowing it to be used inside
loops and Simulink blocks.

