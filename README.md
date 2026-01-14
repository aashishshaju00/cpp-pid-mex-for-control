**cpp-pid-mex-for-control**

**Overview**

This repository demonstrates a discrete-time PID controller implemented
in modern C++ and called from MATLAB and Simulink using a compiled MEX
file to provide fast, stateful control execution that reflects how
digital controllers are used in practice.

The goal is not to provide another MATLAB PID example. The goal is to
show how a real controller can be written in C++, verified, and
exercised from MATLAB and Simulink in a way that is close to how control
software is developed, tested, and integrated in professional
environments.

**Scope**

This project provides:

* A **fully discrete-time PID controller** implemented in C++
* Support for

  * output saturation
  * conditional anti-windup
  * filtered derivative

* A **MEX interface** that exposes the C++ controller to MATLAB
* A **test suite** that verifies the numerical and behavioral
  correctness of the controller
* A **multirate Simulink implementation** where

  * the controller runs at a fixed control rate
  * the plant runs at a faster simulation rate
  * sampling and zero-order holds are explicitly modeled

The same C++ controller is used in all cases. MATLAB and Simulink are
only harnesses around it.

**Out of scope of this repo**

This repository does **not** attempt to cover:

* auto-tuning
* MPC or optimal control
* nonlinear plants
* embedded hardware drivers
* real-time operating systems

The focus is narrow and deliberate:  
**a single, correct, discrete-time PID implementation and how to use it
from MATLAB and Simulink in a realistic way.**

**Why I made this**

Most MATLAB PID examples:

* use built-in PID blocks
* hide sampling, saturation, and anti-windup
* do not reflect how controllers are implemented in C or C++
* do not carry state across calls the way real controllers do

This repository is different:

* the controller lives in C++
* it maintains its own state
* it is called through a binary interface (MEX)
* it is validated by tests
* it is exercised in a multirate Simulink model

This makes it a useful bridge between classical control theory and the
way control software is written, tested, and integrated in robotics,
automotive, and related fields.

**Who this is for**

This demonstration is intended for:

* control engineers who want to move from MATLAB into C++
* engineers working in robotics or automotive software roles
* students who want to understand how digital controllers are actually
  implemented

The material assumes basic familiarity with PID control and MATLAB, but
it does not assume prior experience with MEX or modern C++.

The documentation is designed to be read in order. Each section builds
on the previous one.

The recommended learning path is:

00\_overview.md

01\_pid\_theory.md

02\_discrete\_implementation.md

03\_cpp\_implementation.md

04\_mex\_architecture.md

07\_test\_pid.md

05\_matlab\_harness.md

06\_simulink\_harness.md

This progression mirrors how a controller is developed:

1. What a PID is
2. How it is discretized
3. How it is implemented in C++
4. How it is exposed to MATLAB
5. How it is verified
6. How it is run in MATLAB
7. How it is integrated into Simulink

By the end, the reader will have followed a complete path from control
equations to a working multirate simulation driven by C++ code.

**Repository structure (high level)**

At a high level, the repository contains:

* src/  
  The C++ PID controller and MEX interface
* matlab/  
  MATLAB demos and build scripts
* tests/  
  Deterministic MATLAB tests for the controller
* simulink/  
  A multirate Simulink model using the C++ controller
* docs/  
  The technical documentation that explains how everything fits together







**References**

**1) PID control theory**

K. J. Åström and T. Hägglund, *PID Controllers: Theory, Design and
Tuning*, 2nd ed., Research Triangle Park, NC, USA: Instrumentation,
Systems, and Automation Society, 1995.

**2) Advanced PID methods including anti-windup**

K. J. Åström and T. Hägglund, *Advanced PID Control*, Research Triangle
Park, NC, USA: Instrumentation, Systems, and Automation Society, 2006.

**3) Anti-windup examples in Simulink**

MathWorks, “Anti-Windup Control Using PID Controller Block,” *Simulink
Documentation*, accessed Jan. 13, 2026.

**4) Anti-windup explanation (accessible resource)**

B. Douglas, “Anti-windup for PID control | Understanding PID Control,
Part 2,” *MathWorks Video*, Jun. 6, 2018.

**Open-Source Code / Repositories**

**5) Generic C++ PID implementation**

hfoffani, *PID-controller*, GitHub repository.

**6) C++ anti-windup PID library**

B. Kiamanesh, *Anti-Windup-PID-Controller*, GitHub repository.

**7) C++ PID implementation example (Arduino)**

T. Ttapa, “PID C++ Implementation,” *Control Theory and C++*, online
tutorial.

**8) MEX interface examples**

J. Tilly, *mex*, GitHub repository (collection of MEX examples).

**9) PID and anti-windup in Simulink (reference)**

MathWorks, *PID Controller*, *Simulink Documentation* (general
overview).

