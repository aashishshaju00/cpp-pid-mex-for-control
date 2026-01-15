**Guide to test_pid_basic.m (what each test is checking and why)**

This test script is a sanity-check suite for a discrete PID controller
implemented in C++ and called through pid_mex. The intent is not to
validate a plant model or tuning. It is to validate that the controller
math, saturation behavior, anti-windup logic, and derivative filtering
behave like a textbook discrete PID under controlled inputs.

**What pid_mex(‘step’, ref, y, dt) represents**

Each call executes one controller update at sample time dt:

- error: $`e\lbrack k\rbrack = r\lbrack k\rbrack - y\lbrack k\rbrack`$

- P-term: $`P\lbrack k\rbrack = K_{p}e\lbrack k\rbrack`$

- I-state candidate:
  $`I_{\text{cand}}\lbrack k\rbrack = I\lbrack k - 1\rbrack + e\lbrack k\rbrack\text{ }dt`$

- I-term: $`I_{\text{term}}\lbrack k\rbrack = K_{i}I\lbrack k\rbrack`$

- D-term:
  $`D\lbrack k\rbrack = K_{d} \cdot \frac{e\lbrack k\rbrack - e\lbrack k - 1\rbrack}{dt}`$(with
  optional low-pass filtering on the derivative signal)

- command: $`u_{\text{unsat}} = P + I + D`$

- saturation: $`u = clamp(u_{\text{unsat}},u_{\min},u_{\max})`$

- conditional integration anti-windup: integrator update may be rejected
  if saturated and error pushes further into saturation

The debug struct dbg returns the terms used for that step: {u, p, i, d,
e, saturated}.

**Check whether MEX exists first**

The first check:

if exist("pid_mex","file") ~= 3

ensures MATLAB sees a compiled MEX binary. If this fails, no test
results are meaningful since the controller call is missing.

**Test A: Output format and debug fields**

**What it checks**

- u is a numeric scalar

- dbg is a struct

- dbg contains the fields promised by the interface

- dbg.u matches the returned scalar u

This validates the MEX contract. Even if the controller math is correct,
a mismatch in outputs breaks downstream scripts and Simulink wrappers.
It also catches common MEX mistakes like returning arrays, returning
wrong field names, or changing the debug struct without updating tests.

**Test B: Pure proportional correctness (Kp only)**

**Setup**

- $`K_{p} = 2`$, $`K_{i} = 0`$, $`K_{d} = 0`$

- wide limits so there is no saturation

- $`r = 1`$, $`y = 0`$so $`e = 1`$

**Theory**

With only use proportional control:

``` math
u = K_{p}e = 2 \cdot 1 = 2
```
and

- $`p = 2`$

- $`i = 0`$

- $`d = 0`$

- saturated = false

This is the cleanest algebra check. It confirms:

- error sign convention is correct (ref - meas)

- proportional multiplication is correct

- no unwanted coupling from integrator or derivative state

- saturation is not triggering unexpectedly

**Test C: Integral accumulation with constant error (Ki only)**

**Setup**

- $`K_{p} = 0`$, $`K_{i} = 1`$, $`K_{d} = 0`$

- wide limits so there is no saturation

- constant error: $`r = 1`$, $`y = 0 \Rightarrow e = 1`$

- run for N steps

**Theory**

The integrator is an accumulator of error over time:

Continuous-time idea:

``` math
I(t) = \int_{0}^{t}{e(\tau)\text{ }d\tau}
```
If error is constant and positive (say $`e = 1`$), then:

``` math
I(t) = t
```
so the integral grows linearly with time.

The integral state update in discrete time (rectangle rule) is:

``` math
I\lbrack k\rbrack = I\lbrack k - 1\rbrack + e\lbrack k\rbrack\text{ }dt
```
If $`e\lbrack k\rbrack = 1`$always and $`I\lbrack 0\rbrack = 0`$, then:

``` math
I\lbrack k\rbrack = k\text{ }dt
```
Since $`K_{i} = 1`$:

``` math
i\_ term\lbrack k\rbrack = K_{i}I\lbrack k\rbrack = k\text{ }dt
```
So the expected curve is a straight line with slope $`dt/dt = 1`$in
continuous time, and slope $`dt`$per sample in discrete index.

So:

- dbg.i should increase monotonically (nondecreasing)

- the final value should be on the order of $`N\text{ }dt`$

**Why the test uses two checks**

1.  monotonic increase checks the sign and basic accumulation

2.  end magnitude check catches scale errors like missing dt, using
    dt^2, or resetting the integrator

The bounds are loose because minor implementation details (first step
initialization, recompute order) can shift the exact final value
slightly.

**Test D: Saturation clamps output (upper and lower)**

**Setup**

- huge $`K_{p}`$and error so $`u_{\text{unsat}}`$wants to be very large

- tight limits: $`\lbrack - 1,1\rbrack`$

**Theory**

Saturation is a hard nonlinearity:

``` math
u = \min(\max(u_{\text{unsat}},u_{\min}),u_{\max})
```
So:

- with positive error, u = u_max

- with negative error, u = u_min

- dbg.saturated = true in both cases

This verifies:

- clamp function correctness

- saturation flag correctness

- sign convention for upper vs lower saturation

**Test E: Conditional integration anti-windup blocks windup**

**Setup**

- force saturation high: $`K_{p}`$very large, $`u_{\max} = 1`$

- $`K_{i}`$also nontrivial so windup would happen fast if allowed

- constant positive error: $`r = 1`$, $`y = 0 \Rightarrow e > 0`$

- repeatedly call step and read internal integrator state with getState

**The windup problem (what would happen without anti-windup)**

If the integrator always updates:

``` math
I\lbrack k\rbrack = I\lbrack k - 1\rbrack + e\text{ }dt
```
it grows linearly even though the actuator is pinned at $`u_{\max}`$.
This stored integral later causes overshoot and long recovery.

**Conditional integration rule being tested**

When at upper limit:

- if $`u = u_{\max}`$and $`e > 0`$, reject integration (since error
  pushes further into saturation)

- if $`e < 0`$, allow integration (since it helps come off saturation)

In this test $`e > 0`$always, so the integrator should not change
materially:

``` math
I\lbrack k\rbrack \approx I\lbrack 0\rbrack
```
This catches the most common practical PID bug: integrator growing while
saturated. The test is reading the internal state directly, so it is
checking real controller memory, not just output signals.

**Test F: Derivative filter reduces D-term RMS under noise**

A differentiator amplifies high-frequency content. Measurement noise is
typically high-frequency (or at least broadband), so the derivative of
noise has high variance.

If:

``` math
y\lbrack k\rbrack = n\lbrack k\rbrack\text{(noise)}
```
then:

``` math
\frac{y\lbrack k\rbrack - y\lbrack k - 1\rbrack}{dt}
```
can be very large when $`dt`$is small.

**Setup**

- isolate derivative path: $`K_{p} = 0`$, $`K_{i} = 0`$, $`K_{d} = 1`$

- ref = 0

- meas = noise sequence

- compare D-term magnitude with filter OFF vs filter ON

**Theory**

Backward-difference derivative is:

``` math
d_{\text{raw}}\lbrack k\rbrack = \frac{e\lbrack k\rbrack - e\lbrack k - 1\rbrack}{dt}
```
If $`e`$is dominated by noise, $`d_{\text{raw}}`$becomes large because
differentiation amplifies high-frequency content.

Filtering applies a first-order low-pass to the derivative estimate:

``` math
d_{f}\lbrack k\rbrack = \alpha d_{f}\lbrack k - 1\rbrack + (1 - \alpha)d_{\text{raw}}\lbrack k\rbrack
```
This suppresses high-frequency components, so average magnitude should
drop.

<u>Why RMS is used:</u> The derivative of zero-mean noise is also
roughly zero-mean, so mean is not meaningful. RMS measures typical
magnitude:

``` math
RMS(d) = \sqrt{\mathbb{E}\lbrack d^{2}\rbrack}
```
After initial transient, the filtered D-term should have lower RMS than
unfiltered.

Skipping initial samples avoids penalizing the filtered case due to its
startup transient (filter state begins at 0).
