**03_cpp_implementation.md**

C++ PID Controller Implementation

This section documents the C++ layer of the controller. All control
behavior described in 01_pid_theory.md and 02_discrete_implementation.md
is implemented here.

The C++ implementation consists of three parts:

- Data structures that define parameters, state, and outputs

- The PidController class that implements the discrete time control law

- A small set of helper functions for safety and saturation

Only this layer performs control calculations.

**1. Parameter container: PidParams**

PidParams is a plain data structure holding all tunable controller
settings.

All fields are initialized with safe defaults. This allows PidParams to
be used without explicitly setting every field, which is important when
parameters are provided from MATLAB structs.

No validation is performed here. Validation and numerical safety checks
are done inside step().

**2. Controller state: PidState**

PidState stores all memory variables required by the discrete time
equations.

These correspond directly to the states introduced in
02_discrete_implementation.md.

The has_prev flag is used to suppress derivative calculation on the
first call, where no valid previous sample exists.

**3. Output packaging: PidOutput**

PidOutput packages all intermediate signals for debugging, plotting, and
MATLAB access.

This struct allows MATLAB and Simulink to inspect the internal PID
behavior without reimplementing any logic.

**4. Controller class**

The class owns exactly two pieces of data:

- params\_, which holds the tuning parameters

- state\_, which holds all dynamic state

There is no global state inside the class. Persistence is provided at
the MEX layer.

**5. Safety checks at the top of step()**

The controller refuses to run if:

- dt is nonpositive

- Any input is NaN or infinite

In this case the output is forced to zero. No state is updated. This
prevents numerical corruption of the integrator or derivative state.

**6. Error and proportional term**

This implements:

e\[k\] = r\[k\] − y\[k\]  
P\[k\] = Kp · e\[k\]

**7. Backward difference derivative**

This implements:

ė\[k\] = ( e\[k\] − e\[k−1\] ) / Ts

On the first call, no derivative is computed and the previous values are
initialized.

**8. First order derivative filter**

This matches the discrete time filter derived in
02_discrete_implementation.md:

d_f\[k\] = α d_f\[k−1\] + (1 − α) ė\[k\]

with

α = exp(−2π f_c Ts)

If filtering is disabled, the raw derivative is used.

**9. Integral candidate**

This implements forward Euler integration:

I\[k\] = I\[k−1\] + e\[k\] Ts

The integrator is not committed yet. It is only a candidate until anti
windup is evaluated.

**10. Unsaturated output and saturation**

Saturation is applied only to the output. The integrator state is not
modified here.

**11. Conditional anti windup**

This implements conditional integration:

- If saturated high and error is positive, reject

- If saturated low and error is negative, reject

Otherwise integration is allowed.

**12. Commit or freeze integrator**

The integrator state is only updated if allowed by anti windup logic.

**13. Final recomputation**

After the integrator decision, the final output is recomputed and
saturated again. This ensures internal consistency between p, i, d, and
u.

**14. Reset and initialization**

reset() clears state\_.  
init(params) sets parameters and resets state.

This ensures a clean controller whenever MATLAB calls pid_mex('init').
