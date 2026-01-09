#include "mex.h"
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>

#include "PidController.hpp"

// =====================================================================================
// pid_mex.cpp
//
// This file is the MATLAB MEX "glue layer" around the C++ PID controller.
//
// What this file should do (and only this):
//   - Parse MATLAB inputs (mxArray*) safely
//   - Convert MATLAB types (double, struct, logical) into C++ types (PidParams, double)
//   - Dispatch commands like: init, step, reset, setParams, getState
//   - Return MATLAB outputs (scalars and structs)
//
// What this file should NOT do:
//   - Implement control logic (that belongs in PidController.cpp)
//   - Store any control math outside the controller class
//
// Mental model for MATLAB users:
//   - This MEX function behaves like a MATLAB function with persistent variables.
//   - The static globals below are the "persistent variables" that survive between calls.
//   - Clearing the MEX (clear mex) destroys them, just like clearing persistent state.
// =====================================================================================


// Persistent controller instance inside the MEX (like MATLAB persistent variables)
// In MATLAB terms:
//   - g_pid is the persistent controller object
//   - g_is_init tells us whether init() was called at least once
static PidController g_pid;
static bool g_is_init = false;


// -----------------------------
// Small utilities
// -----------------------------
//
// These helpers are intentionally small and strict.
// The goal is to fail early with clear MATLAB errors if inputs are wrong,
// instead of allowing silent undefined behavior.
//

static std::string getString(const mxArray* a) {
    // First input must be a command string: 'init', 'step', ...
    if (!mxIsChar(a)) {
        mexErrMsgTxt("First argument must be a command string (e.g., 'init', 'step', 'reset').");
    }

    // Fixed-size buffer is fine here because our command names are small.
    // If you later add long commands, increase buf size.
    char buf[128];
    mxGetString(a, buf, sizeof(buf));
    return std::string(buf);
}

static bool hasField(const mxArray* s, const char* name) {
    // mxGetFieldNumber returns -1 if the field does not exist
    return (mxGetFieldNumber(s, name) >= 0);
}

static double getScalarField(const mxArray* s, const char* name, double defaultVal) {
    // Read a scalar double field from a MATLAB struct.
    //
    // This supports user-friendly behavior:
    //   - If the field does not exist, return defaultVal
    //   - If the field exists but has wrong type/shape, throw an error

    if (!mxIsStruct(s) || mxGetNumberOfElements(s) != 1) {
        mexErrMsgTxt("params must be a 1x1 struct.");
    }
    if (!hasField(s, name)) {
        // Field not provided by user, keep default
        return defaultVal;
    }

    const mxArray* f = mxGetField(s, 0, name);
    if (f == nullptr || !mxIsDouble(f) || mxIsComplex(f) || mxGetNumberOfElements(f) != 1) {
        mexErrMsgTxt("Parameter field must be a real double scalar.");
    }

    return mxGetScalar(f);
}

static bool getBoolField(const mxArray* s, const char* name, bool defaultVal) {
    // Read a boolean field from MATLAB:
    //   - accept logical scalar
    //   - also accept numeric scalar (0 -> false, nonzero -> true)
    //
    // This is helpful because many MATLAB users will pass 0/1 doubles.

    if (!mxIsStruct(s) || mxGetNumberOfElements(s) != 1) {
        mexErrMsgTxt("params must be a 1x1 struct.");
    }
    if (!hasField(s, name)) {
        return defaultVal;
    }

    const mxArray* f = mxGetField(s, 0, name);
    if (f == nullptr) mexErrMsgTxt("Invalid params field.");

    if (mxIsLogical(f) && mxGetNumberOfElements(f) == 1) {
        return mxIsLogicalScalarTrue(f);
    }
    if (mxIsDouble(f) && !mxIsComplex(f) && mxGetNumberOfElements(f) == 1) {
        return (mxGetScalar(f) != 0.0);
    }

    mexErrMsgTxt("Boolean parameter field must be a logical scalar or numeric scalar.");
    return defaultVal; // never reached, but keeps compiler happy
}

static PidParams parseParams(const mxArray* paramsStruct) {
    // Convert MATLAB params struct into a C++ PidParams.
    //
    // Design choice:
    //   - If user passes empty [] or omits the struct, we use defaults (PidParams{}).
    //   - For each field, we either read the user value or keep the default.
    //
    // This makes the MEX user API very forgiving:
    //   pid_mex('init')                   -> defaults
    //   pid_mex('init', struct(...))      -> overrides only what is provided

    PidParams p;  // default constructor initializes defaults

    if (paramsStruct == nullptr || mxIsEmpty(paramsStruct)) {
        // Allow pid_mex('init', []) or pid_mex('init') style behavior
        return p;
    }

    if (!mxIsStruct(paramsStruct) || mxGetNumberOfElements(paramsStruct) != 1) {
        mexErrMsgTxt("Second argument to 'init' must be a 1x1 params struct.");
    }

    // Gains
    p.Kp = getScalarField(paramsStruct, "Kp", p.Kp);
    p.Ki = getScalarField(paramsStruct, "Ki", p.Ki);
    p.Kd = getScalarField(paramsStruct, "Kd", p.Kd);

    // Saturation limits
    p.u_min = getScalarField(paramsStruct, "u_min", p.u_min);
    p.u_max = getScalarField(paramsStruct, "u_max", p.u_max);

    // Feature toggles / filter settings
    p.enable_anti_windup = getBoolField(paramsStruct, "enable_anti_windup", p.enable_anti_windup);
    p.deriv_filter_hz    = getScalarField(paramsStruct, "deriv_filter_hz", p.deriv_filter_hz);

    return p;
}

static mxArray* makeOutputStruct(const PidOutput& out) {
    // Create a MATLAB struct for debugging and plotting.

    const char* fields[] = {"u","p","i","d","e","saturated"};
    mxArray* s = mxCreateStructMatrix(1, 1, 6, fields);

    mxSetField(s, 0, "u",         mxCreateDoubleScalar(out.u));
    mxSetField(s, 0, "p",         mxCreateDoubleScalar(out.p));
    mxSetField(s, 0, "i",         mxCreateDoubleScalar(out.i));
    mxSetField(s, 0, "d",         mxCreateDoubleScalar(out.d));
    mxSetField(s, 0, "e",         mxCreateDoubleScalar(out.e));
    mxSetField(s, 0, "saturated", mxCreateLogicalScalar(out.saturated));

    return s;
}

// Optional cleanup handler (called automatically if MEX is cleared)
static void atExitCleanup() {
    // When MATLAB clears the MEX, we reset our persistent controller.
    // This prevents stale state from lingering if MATLAB unloads/reloads the binary.
    g_pid.reset();
    g_is_init = false;
}

// -----------------------------
// MEX entry point
// -----------------------------
//
// Signature is mandated by MATLAB.
// MATLAB calls mexFunction whenever pid_mex(...) is called.
//
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // nlhs = number of left-hand outputs requested
    // nrhs = number of right-hand inputs provided

    if (nrhs < 1) {
        mexErrMsgTxt("Usage: pid_mex('init', params) or pid_mex('step', ref, meas, dt) or pid_mex('reset').");
    }

    // Register cleanup once per MEX load.
    // Calling mexAtExit multiple times is safe; MATLAB keeps one handler.
    mexAtExit(atExitCleanup);

    // First argument is always the command string
    const std::string cmd = getString(prhs[0]);

    // -------------------------
    // init
    // -------------------------
    if (cmd == "init") {
        // MATLAB usage:
        //   pid_mex('init', paramsStruct)
        //   pid_mex('init')             -> use defaults
        //
        // paramsStruct can omit fields; omitted fields remain at defaults.

        PidParams params;
        if (nrhs >= 2) {
            params = parseParams(prhs[1]);
        } else {
            params = PidParams{};
        }

        // Configure controller and wipe its internal memory
        g_pid.init(params);
        g_is_init = true;

        // No outputs for init
        return;
    }
    // -------------------------
    // reset
    // -------------------------
    if (cmd == "reset") {
        // reset() wipes controller state but keeps current params.
        // This is similar to restarting a simulation from time = 0.
        //
        // Allow reset even if init was never called:
        //   - This keeps the interface forgiving.
        //   - Default params are still valid inside g_pid.
        g_pid.reset();

        // After reset, the controller is in a usable state.
        // (Params are whatever the controller currently holds.)
        g_is_init = true;

        return;
    }

    // -------------------------
    // step
    // -------------------------
    if (cmd == "step") {
        // MATLAB usage:
        //   u = pid_mex('step', ref, meas, dt)
        //   [u, dbg] = pid_mex('step', ref, meas, dt)
        //
        // This performs exactly ONE control update.
        // The controller's internal state persists for the next call.

        if (!g_is_init) {
            mexErrMsgTxt("pid_mex: controller not initialized. Call pid_mex('init', params) first.");
        }
        if (nrhs < 4) {
            mexErrMsgTxt("Usage: pid_mex('step', ref, meas, dt)");
        }

        // Type/shape checks: enforce scalar real doubles.
        // This avoids subtle bugs if a user accidentally passes vectors or complex values.
        if (!mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != 1 || mxIsComplex(prhs[1]) ||
            !mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1 || mxIsComplex(prhs[2]) ||
            !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != 1 || mxIsComplex(prhs[3])) {
            mexErrMsgTxt("'ref', 'meas', and 'dt' must be real double scalars.");
        }

        const double ref  = mxGetScalar(prhs[1]);
        const double meas = mxGetScalar(prhs[2]);
        const double dt   = mxGetScalar(prhs[3]);

        // Run one controller step
        const PidOutput out = g_pid.step(ref, meas, dt);

        // Output behavior is intentionally flexible:
        //   - If user asks for 0 outputs, we do nothing (MATLAB will just execute the call)
        //   - If user asks for 1 output, return u only (fast path)
        //   - If user asks for >=2 outputs, return u plus a debug struct
        //
        // Example:
        //   u = pid_mex('step', ...)
        //   [u, dbg] = pid_mex('step', ...)

        if (nlhs >= 1) {
            plhs[0] = mxCreateDoubleScalar(out.u);
        }
        if (nlhs >= 2) {
            plhs[1] = makeOutputStruct(out);
        }

        return;
    }

    // -------------------------
    // setParams
    // -------------------------
    if (cmd == "setParams") {
        // MATLAB usage:
        //   pid_mex('setParams', paramsStruct)
        //
        // Unlike init(), this does NOT reset internal controller memory.
        // This is useful for on-the-fly tuning.

        if (nrhs < 2) {
            mexErrMsgTxt("Usage: pid_mex('setParams', paramsStruct)");
        }

        const PidParams params = parseParams(prhs[1]);
        g_pid.setParams(params);

        // After setting params, controller remains usable.
        g_is_init = true;

        return;
    }

    // -------------------------
    // getState (optional)
    // -------------------------
    if (cmd == "getState") {
        // MATLAB usage:
        //   st = pid_mex('getState')
        //
        // This returns the controller's *internal memory* as a MATLAB struct
        // so it can be plotted, debugged, or logged.

        if (nlhs < 1) {
            mexErrMsgTxt("Usage: st = pid_mex('getState')");
        }
        if (!g_is_init) {
            mexErrMsgTxt("pid_mex: controller not initialized. Call pid_mex('init', params) first.");
        }

        const PidState& st = g_pid.getState();

        const char* fields[] = {"integral","prev_error","prev_meas","d_filt","has_prev"};
        mxArray* s = mxCreateStructMatrix(1, 1, 5, fields);

        mxSetField(s, 0, "integral",   mxCreateDoubleScalar(st.integral));
        mxSetField(s, 0, "prev_error", mxCreateDoubleScalar(st.prev_error));
        mxSetField(s, 0, "prev_meas",  mxCreateDoubleScalar(st.prev_meas));
        mxSetField(s, 0, "d_filt",     mxCreateDoubleScalar(st.d_filt));
        mxSetField(s, 0, "has_prev",   mxCreateLogicalScalar(st.has_prev));

        plhs[0] = s;
        return;
    }

    // If we reach here, user provided an unsupported command string
    mexErrMsgTxt("Unknown command. Supported: 'init', 'step', 'reset', 'setParams', 'getState'.");
}
