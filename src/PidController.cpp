#include "PidController.hpp"

#include <cmath>    // std::abs, std::isfinite, std::exp

// =====================================================================================
// PidController.cpp
//
// This file contains the full implementation of a discrete-time PID controller
// designed to be called from MATLAB and Simulink through a MEX interface.
//
// Design philosophy:
//   - This class owns all control logic and all internal state.
//   - The MEX file is just a thin wrapper that passes inputs in and outputs back.
//   - The structure mirrors how an embedded controller would be written.
//
// The controller implements:
//   - Proportional term
//   - Integral term (rectangle rule integration)
//   - Derivative term (backward difference)
//   - Optional 1st-order low-pass filter on the derivative
//   - Output saturation
//   - Conditional-integration anti-windup
// =====================================================================================


// -------------------------------
// Constructors
// -------------------------------

PidController::PidController(const PidParams& params) {
    // We delegate construction to init().
    // This mirrors how the controller will be created once in the MEX file
    // and then configured from MATLAB.
    init(params);
}


// -------------------------------
// init / reset / setParams / getters
// -------------------------------

void PidController::init(const PidParams& params) {
    // init() is a combined "configure + reset" function.
    //
    // This is useful in MEX because:
    //  - The controller object persists across MATLAB calls
    //  - MATLAB can call 'init' when a new simulation starts
    //  - All internal state should be wiped at that moment

    params_ = params;

    // Safety check: ensure actuator limits are not reversed.
    // Silent reversed limits would cause extremely confusing behavior.
    if (params_.u_min > params_.u_max) {
        const double tmp = params_.u_min;
        params_.u_min = params_.u_max;
        params_.u_max = tmp;
    }

    // Reset all internal controller memory
    reset();
}

void PidController::reset() {
    // This wipes all controller memory:
    //  - integrator
    //  - previous error
    //  - filtered derivative
    state_ = PidState{};  // resets everything to default values
}

void PidController::setParams(const PidParams& params) {
    // Allows changing gains and limits without wiping integrator state.
    // This is useful for gain scheduling or tuning while running.

    params_ = params;

    if (params_.u_min > params_.u_max) {
        const double tmp = params_.u_min;
        params_.u_min = params_.u_max;
        params_.u_max = tmp;
    }
}

const PidParams& PidController::getParams() const {
    // Return read-only access to tuning parameters
    return params_;
}

const PidState& PidController::getState() const {
    // Return read-only access to controller memory for debugging
    return state_;
}


// -------------------------------
// Helper function
// -------------------------------

double PidController::clamp(double x, double low, double high) {
    // Enforces actuator limits.
    // This is the only place where saturation is applied.
    if (x < low) return low;
    if (x > high) return high;
    return x;
}


// -------------------------------
// step()
// -------------------------------
//
// This function executes exactly one control update:
//   u[k] = PID( ref[k], meas[k], dt )
//
// It is designed to be called repeatedly from MATLAB or Simulink
// with a fixed or variable timestep dt.
//

PidOutput PidController::step(double ref, double meas, double dt) {
    PidOutput out{};

    // ------------------------------------------------------------------
    // 0) Input validation
    // ------------------------------------------------------------------
    // MATLAB passes everything as double, including NaNs or bad dt.
    // A broken dt will instantly destroy a discrete controller.
    if (!(dt > 0.0) || !std::isfinite(dt) || !std::isfinite(ref) || !std::isfinite(meas)) {
        // Fail-safe output
        out.u = 0.0;
        out.e = 0.0;
        return out;
    }

    // ------------------------------------------------------------------
    // 1) Error
    // ------------------------------------------------------------------
    // e[k] = reference - measurement
    out.e = ref - meas;

    // ------------------------------------------------------------------
    // 2) Proportional term
    // ------------------------------------------------------------------
    out.p = params_.Kp * out.e;

    // ------------------------------------------------------------------
    // 3) Derivative term
    // ------------------------------------------------------------------
    //
    // We compute a backward-difference derivative of the error:
    //   d_raw = (e[k] - e[k-1]) / dt
    //
    // The first sample has no history, so we initialize safely.

    double d_raw = 0.0;
    if (state_.has_prev) {
        const double de = out.e - state_.prev_error;
        d_raw = de / dt;
    } else {
        // First call: no derivative yet
        d_raw = 0.0;
        state_.has_prev = true;
        state_.prev_error = out.e;
        state_.prev_meas = meas;
    }

    // ------------------------------------------------------------------
    // Optional derivative filtering
    // ------------------------------------------------------------------
    //
    // High-frequency noise is amplified by differentiation.
    // We therefore pass the raw derivative through a 1st-order low-pass filter:
    //
    //   d_filt[k] = alpha * d_filt[k-1] + (1 - alpha) * d_raw
    //
    // where:
    //   alpha = exp( -2*pi*fc*dt )
    //
    // This is the exact discrete-time form of a continuous first-order low-pass
    // This is the ZOH (Zero Order Hold) approximation of the continuous-time low-pass filter over one sample period.
    // with cutoff frequency fc under a zero-order hold assumption.

    if (params_.deriv_filter_hz > 0.0) {
        const double alpha = std::exp(-2.0 * 3.14159265358979323846 *
                                      params_.deriv_filter_hz * dt);

        state_.d_filt = alpha * state_.d_filt + (1.0 - alpha) * d_raw;
        out.d = params_.Kd * state_.d_filt;
    } else {
        // No filtering, use raw derivative directly
        out.d = params_.Kd * d_raw;
    }

    // ------------------------------------------------------------------
    // 4) Integral candidate
    // ------------------------------------------------------------------
    //
    // We first compute a *candidate* integral using rectangle integration:
    //
    //   I_candidate = I[k-1] + e[k] * dt
    //
    // We do not commit it yet, because saturation may require us to reject it.

    const double integral_candidate = state_.integral + out.e * dt;
    const double i_candidate = params_.Ki * integral_candidate;

    // ------------------------------------------------------------------
    // 5) Unsaturated control signal
    // ------------------------------------------------------------------
    //
    // This is what the PID would output if the actuator had infinite authority.

    const double u_unsat = out.p + i_candidate + out.d;

    // ------------------------------------------------------------------
    // 6) Apply saturation
    // ------------------------------------------------------------------
    out.u = clamp(u_unsat, params_.u_min, params_.u_max);
    out.saturated = (out.u != u_unsat);

    // ------------------------------------------------------------------
    // 7) Anti-windup: conditional integration
    // ------------------------------------------------------------------
    //
    // If the actuator is saturated, blindly integrating the error
    // can cause the integrator to grow in a direction the actuator
    // cannot follow. This leads to overshoot and long recovery.
    //
    // We therefore:
    //   - integrate normally when not saturated
    //   - integrate when saturated only if the error would push the output
    //     back toward the unsaturated region

    bool accept_integral = true;

    if (params_.enable_anti_windup && out.saturated) {
        if (out.u >= params_.u_max) {
            // At upper limit: positive error would push further into saturation
            if (out.e > 0.0) accept_integral = false;
        } else if (out.u <= params_.u_min) {
            // At lower limit: negative error would push further into saturation
            if (out.e < 0.0) accept_integral = false;
        }
    }

    if (accept_integral) {
        // Commit the new integral
        state_.integral = integral_candidate;
        out.i = i_candidate;
    } else {
        // Freeze integrator
        out.i = params_.Ki * state_.integral;
    }

    // ------------------------------------------------------------------
    // 8) Update history
    // ------------------------------------------------------------------
    state_.prev_error = out.e;
    state_.prev_meas  = meas;

    // ------------------------------------------------------------------
    // 9) Recompute final output using the committed integrator
    // ------------------------------------------------------------------
    //
    // If we rejected the candidate integrator, we must recompute u
    // so that P + I + D is internally consistent.

    const double u_recomputed = out.p + out.i + out.d;
    out.u = clamp(u_recomputed, params_.u_min, params_.u_max);
    out.saturated = (out.u != u_recomputed);

    return out;
}
