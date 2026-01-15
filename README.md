# cpp-pid-mex-for-control

## Overview
This repository demonstrates a discrete-time PID controller implemented in modern C++ and accessed from MATLAB and Simulink through a compiled MEX file. The focus is on showing how a controller written in C++ can be executed from MATLAB and Simulink in a stateful and efficient way. Rather than providing another MATLAB-only PID example, the repository illustrates an approach for implementing, testing, and running a controller across C++ and MATLAB-based environments.

## Scope
This project provides:

- A fully discrete-time PID controller implemented in C++
- Support for:
  - Output saturation
  - Conditional anti-windup (conditional integration)
  - Filtered derivative
- A MEX interface that exposes the C++ controller to MATLAB
- A test suite that verifies numerical and behavioral correctness
- A multirate Simulink example where:
  - The controller runs at a fixed control rate
  - The plant runs at a faster simulation rate
  - Sampling and zero-order holds are explicitly modeled

The same C++ controller is used in all cases. MATLAB and Simulink are harnesses around it.

## Out of scope
This repository does not attempt to cover:

- Auto-tuning
- MPC or optimal control
- Nonlinear plants
- Embedded hardware drivers
- Real-time operating systems

The focus is narrow and deliberate: discrete-time PID implementation and how to use it from MATLAB and Simulink in a realistic way.

## Why this exists
Many MATLAB PID examples:

- Use built-in PID blocks
- Hide sampling, saturation, and anti-windup
- Do not reflect how controllers are implemented in C or C++

This repository is different:

- The controller lives in C++
- maintains its own state
- called through a binary interface (MEX)
- validated by deterministic tests
- exercised in a multirate Simulink model

This makes it a practical bridge between classical control theory and how control software is written, tested, and integrated in robotics, automotive, and related fields.

## Who this is for
This repository is intended for:

- Control engineers moving from MATLAB into C++
- Engineers in robotics or automotive software roles
- Students who want to understand how digital controllers are implemented in practice

Basic familiarity with PID control, MATLAB, and basic C/C++ experience is assumed. No prior experience with MEX is required.

## Documentation (recommended reading order)
The documentation is designed to be read in order:

1. [PID control with filtered derivative](docs/01_pid_theory.md)
2. [Discrete-time implementation (matches the code)](docs/02_discrete_implementation.md)
3. [C++ implementation details](docs/03_cpp_implementation.md)
4. [MEX interface architecture](docs/04_mex_architecture.md)
5. [MATLAB harness and demos](docs/05_matlab_usage.md)
6. [Simulink multirate harness](docs/06_simulink_usage.md)
7. [Test suite and behavioral checks](docs/07_test_pid.md)

## Repository structure (high level)
- `src/`  
  C++ PID controller and MEX interface (`PidController.*`, `pid_mex.cpp`)
- `matlab/`  
  MATLAB build script and demos (calls `pid_mex`)
- `tests/`  
  Deterministic MATLAB tests for correctness and expected behaviors
- `simulink/`  
  Multirate Simulink model that calls the same MEX controller
- `docs/`  
  Technical documentation explaining the full stack

## References
1. Åström, K. J., and Hägglund, T., *PID Controllers: Theory, Design and Tuning*, 2nd ed., ISA, 1995.
2. Åström, K. J., and Hägglund, T., *Advanced PID Control*, ISA, 2006.
3. MathWorks, “Anti-Windup Control Using PID Controller Block,” Simulink Documentation.
4. Douglas, B., “Anti-windup for PID control | Understanding PID Control, Part 2,” MathWorks Video, 2018.

### Related repositories and examples
5. hfoffani, “PID-controller,” GitHub repository (PID Controller). (https://github.com/hfoffani/PID-controller)
6. Kiamanesh, B., “Anti-Windup-PID-Controller,” GitHub repository. (https://github.com/BanaanKiamanesh/Anti-Windup-PID-Controller)
7. LukasLeonardKoening, “PID-Controller,” GitHub repository (Project 7 - PID Controller). (https://github.com/LukasLeonardKoening/PID-Controller)
8. jtilly., “mex,” GitHub repository (Using Fortran and C++ mex files with Matlab). (https://github.com/jtilly/mex)
9. MathWorks, “PID Controller,” Simulink Documentation. (https://www.mathworks.com/help/simulink/slref/pidcontroller.html)