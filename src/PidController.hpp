#pragma once

#include <cstdint>   // for int32_t (optional)
/*
    PID Controller Header
    Goal:
    - Keep all tuning parameters in one struct (Params)
    - Keep all memory/state in one struct (State)
    - Provide a clean class interface:
        init(params)
        reset()
        step(reference, measurement, dt)
*/
/// 1) Parameters: these are NOT signals and do NOT change every time step.
///    This is like the MATLAB constants / tunable gains.
struct PidParams {
    // PID gains
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    // Output limits (actuator saturation)
    double u_min = -1.0;
    double u_max =  1.0;

    // Anti-windup option:
    // If true, the integrator will be clamped when the output saturates.
    bool enable_anti_windup = true;

    // Optional: derivative filtering (simple 1st-order low-pass on derivative term)
    // 0 means "no filtering"
    double deriv_filter_hz = 0.0;
};


/// 2) State: these ARE the memory variables.
///    Analogous to MATLAB persistent variables
struct PidState {
    // Integral of error
    double integral = 0.0;

    // Previous error (for derivative on error)
    double prev_error = 0.0;

    // Previous measurement (for derivative on measurement, less kick)
    double prev_meas = 0.0;

    // Filtered derivative state (if derivative filter is enabled)
    double d_filt = 0.0;

    // Whether step() has been called at least once (important for derivative initialization)
    bool has_prev = false;
};

/// 3) Output packaging: for debugging and plotting.

struct PidOutput {
    double u = 0.0;     // final control output after saturation
    double p = 0.0;     // proportional contribution
    double i = 0.0;     // integral contribution
    double d = 0.0;     // derivative contribution
    double e = 0.0;     // current error (ref - measurement)
    bool saturated = false;
};


/// 4) The Controller Class: this owns params + state.
///    OOP equivalent of a MATLAB Function block with persistent variables.
    class PidController {
    public:
        // Constructor: creates a controller with default params.
        PidController() = default;

        // Optional constructor: create and initialize with params right away.
        explicit PidController(const PidParams& params);

        // init: sets parameters and resets state.
        // Why have init? Because MEX usually creates the object once,
        // and then you want to configure it from MATLAB.
        void init(const PidParams& params);

        // reset: clears internal memory (integral, prev error, etc.)
        // Equivalent to clearing MATLAB persistent variables.
        void reset();

        // step: one control update (one simulation time step).
        // Inputs:
        //  - ref: desired value
        //  - meas: measured value
        //  - dt: timestep in seconds
        //
        // Returns a PidOutput (u and debug terms).
        PidOutput step(double ref, double meas, double dt);

        // Set new parameters without resetting state (sometimes useful).
        void setParams(const PidParams& params);

        // Getters: useful for debugging and unit tests.
        const PidParams& getParams() const;
        const PidState&  getState()  const;

    private:
        // Helper function: clamps/saturates a value between low and high. Used in the anti windup mechanism in the Cpp script.
        static double clamp(double x, double low, double high);

    private:
        PidParams params_{};
        PidState  state_{};
    };