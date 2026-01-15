**Discrete-time PID with filtered derivative, saturation, and
anti-windup**

This section derives the discrete-time control law used by the C++
implementation. Every equation in this document corresponds directly to
a line of code in PidController::step().

The controller runs at a fixed sample time $`T_{s}`$.

**1. Discrete-time error**

At each controller update $`k`$,

``` math
e\lbrack k\rbrack = r\lbrack k\rbrack - y\lbrack k\rbrack
```

This error is computed from the sampled reference and measurement.

**2. Proportional term**

The proportional contribution is

``` math
P\lbrack k\rbrack = K_{p}\text{ }e\lbrack k\rbrack
```

This term responds immediately to tracking error.

**3. Raw derivative (backward difference)**

The derivative of the error is approximated using a backward difference:

``` math
\dot{e}\lbrack k\rbrack = \left\{ \begin{matrix}
0, & k = 0 \\
\frac{e\lbrack k\rbrack - e\lbrack k - 1\rbrack}{T_{s}}, & k \geq 1
\end{matrix} \right.\ 
```

This avoids needing future samples.

**4. Filtered derivative**

A first-order low-pass filter is applied to the raw derivative.

The continuous-time filter is

``` math
D_{f}(s) = \frac{1}{1 + \frac{s}{\omega_{c}}}
```

Under a zero-order hold assumption, the exact discrete-time form is

``` math
\mathbf{d}_{\mathbf{f}}\mathbf{\lbrack k\rbrack = \alpha}\text{ }\mathbf{d}_{\mathbf{f}}\mathbf{\lbrack k - 1\rbrack + (1 - \alpha)}\text{ }\dot{\mathbf{e}}\mathbf{\lbrack k\rbrack}
```
with

``` math
\alpha = e^{- 2\pi f_{c}T_{s}}
```
where $`f_{c}`$is the derivative filter cutoff in Hz.

Derivation of this

**Start from the continuous-time model**

The first-order low-pass filter applied to the raw derivative is defined
by the ODE:

``` math
\tau\text{ }{\dot{d}}_{f}(t) + d_{f}(t) = d_{\text{raw}}(t)
```
Rearrange:

``` math
{\dot{d}}_{f}(t) = - \frac{1}{\tau}d_{f}(t) + \frac{1}{\tau}d_{\text{raw}}(t)
```
This is a linear first-order ODE.

**Zero-order hold assumption**

Assuming the input is **constant over one sample interval**:

``` math
d_{\text{raw}}(t) = d_{\text{raw}}\lbrack k\rbrack,t \in \lbrack kT_{s},(k + 1)T_{s})
```
This is the standard assumption used in digital control

**Solve the ODE over one interval**

The ODE has the standard solution:

``` math
d_{f}(t) = Ce^{- t/\tau} + d_{\text{raw}}\lbrack k\rbrack
```
Now enforcing the initial condition at the start of the interval:

``` math
d_{f}(kT_{s}) = d_{f}\lbrack k - 1\rbrack
```
So:

``` math
d_{f}\lbrack k - 1\rbrack = Ce^{- kT_{s}/\tau} + d_{\text{raw}}\lbrack k\rbrack
```
Solve for $`C`$:

``` math
C = \left( d_{f}\lbrack k - 1\rbrack - d_{\text{raw}}\lbrack k\rbrack \right)e^{kT_{s}/\tau}
```
Substituting it back and evaluating at the end of the interval
$`t = (k + 1)T_{s}`$:

``` math
d_{f}((k + 1)T_{s}) = \left( d_{f}\lbrack k - 1\rbrack - d_{\text{raw}}\lbrack k\rbrack \right)e^{- T_{s}/\tau} + d_{\text{raw}}\lbrack k\rbrack
```
Defining

``` math
d_{f}\lbrack k\rbrack = d_{f}((k + 1)T_{s})
```
Then:

``` math
\mathbf{d}_{\mathbf{f}}\mathbf{\lbrack k\rbrack =}\mathbf{e}^{\mathbf{-}\mathbf{T}_{\mathbf{s}}\mathbf{/}\mathbf{\tau}}\mathbf{d}_{\mathbf{f}}\mathbf{\lbrack k - 1\rbrack +}\left( \mathbf{1}-\mathbf{e}^{\mathbf{-}\mathbf{T}_{\mathbf{s}}\mathbf{/}\mathbf{\tau}} \right)\mathbf{d}_{\text{raw}}\mathbf{\lbrack k\rbrack}
```

The derivative contribution is then

``` math
D\lbrack k\rbrack = K_{d}\text{ }d_{f}\lbrack k\rbrack
```
If filtering is disabled ($`f_{c} = 0`$), the raw derivative is used
directly:

``` math
D\lbrack k\rbrack = K_{d}\text{ }\dot{e}\lbrack k\rbrack
```
**5. Integral candidate (rectangle rule)**

The integral of the error is approximated using forward Euler
integration:

``` math
I_{\text{cand}}\lbrack k\rbrack = I\lbrack k - 1\rbrack + e\lbrack k\rbrack\text{ }T_{s}
```

The corresponding control contribution is

``` math
I_{\text{cand}}^{u}\lbrack k\rbrack = K_{i}\text{ }I_{\text{cand}}\lbrack k\rbrack
```

This value is not committed immediately because actuator saturation may
require rejecting it.

**6. Unsaturated control signal**

The ideal PID output before saturation is

``` math
u_{\text{unsat}}\lbrack k\rbrack = P\lbrack k\rbrack + I_{\text{cand}}^{u}\lbrack k\rbrack + D\lbrack k\rbrack
```
**7. Output saturation**

The actuator is limited:

``` math
u\lbrack k\rbrack = clamp(u_{\text{unsat}}\lbrack k\rbrack,\text{ }u_{\min},\text{ }u_{\max})
```

Saturation is detected if

``` math
u\lbrack k\rbrack \neq u_{\text{unsat}}\lbrack k\rbrack
```

Only the **final control output** is saturated. The integrator state is
not directly clamped.

**8. Anti-windup via conditional integration**

If the actuator is saturated, blindly integrating the error can drive
the integrator in a direction the actuator cannot follow. This causes
overshoot and long recovery.

The controller therefore accepts the candidate integrator only when it
would not push the output deeper into saturation.

The logic is:

- If not saturated  
  → accept the integrator

- If saturated at the upper limit ($`u = u_{\max}`$)

  - reject integration if $`e\lbrack k\rbrack > 0`$

- If saturated at the lower limit ($`u = u_{\min}`$)

  - reject integration if $`e\lbrack k\rbrack < 0`$

Formally:

``` math
\text{accept} = \left\{ \begin{matrix}
\text{true}, & \text{not saturated} \\
\text{false}, & u = u_{\max}\text{ and }e\lbrack k\rbrack > 0 \\
\text{false}, & u = u_{\min}\text{ and }e\lbrack k\rbrack < 0 \\
\text{true}, & \text{otherwise}
\end{matrix} \right.\ 
```
If accepted:

``` math
I\lbrack k\rbrack = I_{\text{cand}}\lbrack k\rbrack
```
If rejected:

``` math
I\lbrack k\rbrack = I\lbrack k - 1\rbrack
```
The integral contribution used is always

``` math
I^{u}\lbrack k\rbrack = K_{i}\text{ }I\lbrack k\rbrack
```
**9. Final control recomputation**

After deciding whether to accept or reject the integrator update, the
final control is recomputed:

``` math
u\lbrack k\rbrack = clamp(P\lbrack k\rbrack + I^{u}\lbrack k\rbrack + D\lbrack k\rbrack,\text{ }u_{\min},\text{ }u_{\max})
```
This guarantees that the reported $`P`$, $`I`$, $`D`$, and $`u`$are
internally consistent.

**10. State update**

At the end of each step, the controller stores:

``` math
{e\lbrack k\rbrack \rightarrow e\lbrack k - 1\rbrack
}{y\lbrack k\rbrack \rightarrow y\lbrack k - 1\rbrack
}{d_{f}\lbrack k\rbrack \rightarrow d_{f}\lbrack k - 1\rbrack
}{I\lbrack k\rbrack \rightarrow I\lbrack k - 1\rbrack}
```

These states are used on the next call.

All of these operate at a fixed sample time $`T_{s}`$.  
This discrete-time system is what the C++ code, MATLAB tests, and
Simulink model execute.
