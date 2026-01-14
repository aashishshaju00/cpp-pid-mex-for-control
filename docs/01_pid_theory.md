**01_pid_theory.md**

**PID control with filtered derivative**

This section introduces the PID control law used throughout this
repository.  
The goal is not to present every possible PID variant, but to define the
exact form that is later discretized, implemented in C++, tested, and
used in Simulink.

**1. Control objective**

The controller is designed to make the measured output $`y(t)`$follow a
reference signal $`r(t)`$.

The tracking error is defined as

``` math
e(t) = r(t) - y(t)
```

The controller produces a control input $`u(t)`$that drives the plant so
that $`e(t) \rightarrow 0`$.

For the demos in this repository, the plant is a first-order system

``` math
G(s) = \frac{1}{\tau s + 1}
```

This model is only used to generate realistic responses in MATLAB and
Simulink. The PID itself does not depend on the plant being first-order.

**2. Continuous-time PID with derivative filtering**

The controller used here has the following transfer function:

``` math
C(s) = K_{p} + \frac{K_{i}}{s} + \frac{K_{d}\text{ }s}{1 + s/\omega_{c}}
```

This is a standard PID controller with a **first-order low-pass filter
on the derivative term**.

Written in time-domain form:

``` math
u(t) = K_{p}e(t) + K_{i}\int_{0}^{t}{e(\tau)\text{ }d\tau +}K_{d}\text{ }d_{f}(t)
```

where the filtered derivative $`d_{f}(t)`$satisfies

``` math
\frac{d}{dt}d_{f}(t) = \omega_{c}\left( \frac{de(t)}{dt} - d_{f}(t) \right)
```

The filter cutoff $`\omega_{c}`$limits how much high-frequency noise
enters the derivative term.

**3. Why the derivative is filtered**

A pure derivative

``` math
\frac{de(t)}{dt}
```

amplifies high-frequency noise. In real systems, measurements always
contain noise, so an unfiltered derivative can dominate the control
signal and destabilize the loop.

The filtered derivative

``` math
\frac{K_{d}s}{1 + s/\omega_{c}}
```

acts like a differentiator at low frequencies but rolls off at high
frequencies. This gives the phase lead and damping benefits of the
derivative without excessive noise amplification.

**4. Role of the P, I, and D terms**

Each term has a distinct effect on the closed-loop behavior.

**Proportional term** $`\mathbf{K}_{\mathbf{p}}\mathbf{e}`$

- Increases responsiveness

- Reduces rise time

- Too much $`K_{p}`$causes oscillation and overshoot

**Integral term**
$`\mathbf{K}_{\mathbf{i}}\mathbf{\int e}\text{ }\mathbf{dt}`$

- Eliminates steady-state error

- Compensates for constant disturbances

- Too much $`K_{i}`$causes slow oscillations and windup

**Derivative term** $`\mathbf{K}_{\mathbf{d}}\mathbf{d}_{\mathbf{f}}`$

- Adds damping

- Improves stability and phase margin

- Reduces overshoot

- Must be filtered to avoid noise amplification

**5. Saturation and anti-windup**

Real actuators have limits. The control signal is constrained:

``` math
u_{\min} \leq u(t) \leq u_{\max}
```

When the controller saturates, the integrator can continue accumulating
error even though the actuator is no longer responding. This leads to
**integral windup**, which causes large overshoot and slow recovery.

This repository uses **conditional integration**:

- The integrator is only updated when the output is not saturated

- Or when the integrator would drive the output back toward the
  unsaturated region

This keeps the integral term bounded and improves recovery after
saturation.

**6. Bandwidth perspective and tuning intuition**

For the example plant

``` math
G(s) = \frac{1}{\tau s + 1}
```

the plant’s natural bandwidth is approximately

``` math
\omega_{p} \approx \frac{1}{\tau}
```

For $`\tau \approx 0.15`$s, this gives

``` math
\omega_{p} \approx 6.7\text{ rad/s}( \approx 1\text{ Hz})
```

The controller should have a closed-loop bandwidth on the same order or
slightly below this value.

The derivative filter cutoff should satisfy

``` math
\text{closed-loop bandwidth} \ll \omega_{c} \ll \text{noise bandwidth}
```

In the demos, $`\omega_{c}`$is chosen so that:

- it is well above the closed-loop bandwidth

- it is low enough to suppress measurement noise

This allows the derivative to add phase lead and damping without
injecting noise into the control signal.

**7. Summary**

The PID controller used in this repository is

- fully discrete

- includes saturation

- includes conditional anti-windup

- uses a filtered derivative

The continuous-time equations defined here are the starting point for
the discrete-time implementation in the next section.
