# FOPID temperature control with S-Curve reference

This repository contains my configurations for controlling a **thermal system** (heating/cooling).  
“A PID that works in simulation only” is not my aim. Instead, my objective is to create a reliable controller and a reference generator that could work in reality, with special emphasis on the Teensy 4.0 microcontroller.

It is made up of three primary components:

- **FOPID controller class** (Fractional-Order PID)
- **S-Curve temperature reference generator** (program / ramp planner)
- **Objective function (Cost Function)** utilized for adjustment of parameters according to my own control priorities

---

## Why not a simple step reference with a classic PID?

There are several challenges in controlling thermal systems::

- Their response is **slow**, characterized by **inertia**.  
- Temperatures can continue to climb even after power shutdowns.
- **Delay**, **sensor position**, and **hysteresis** are factors that influence performance measurement tracking.
- Applications related to comfort, treatment, or safety cannot withstand **sudden temperature changes**.
- Real-world actuators are bounded by some constraints like **maximum output**, **maximum rate**, or **safe boundaries**.


A classic PID controller could be used, but it would necessarily mean trading-off between:

- **less overshoot ↔ slower response**
- **faster response ↔ more overshoot or more aggressive control effort**

I wanted more flexibility, so I moved to **FOPID** and paired it with a **smooth reference profile**.

---

## What led me to work with FOPID

FOPID is an extension of the ordinary PID controller, which allows **fractional orders** for both the integral and derivative components. This means that:

\[
u(t)=K_p e(t)+K_i D^{-\lambda}\{e(t)\}+K_d D^{\mu}\{e(t)\}
\]

Where:

- **\(K_p, K_i, K_d\)** are the usual gains  
- **\(\lambda\)** is the *integral order*  
- **\(\mu\)** is the *derivative order*

These additional parameters (\(\lambda, \mu\)) allow a better trade-off between **overshoot**, **response speed**, and **smoothness** compared to a standard PID.

---

## System architecture

I organized the work into distinct layers:

### Reference Generation (`SCurveProfile`)
- Where should the temperature go over time?
- It creates smooth ramps instead of step jumps

### Controller (`FOPID`)
- determines the system's method of tracking and monitoring the reference.

### Constraints (actuator realism)
- Ensures that the output is physically possible  
- Includes **saturation**, **rate limitation**, and **anti-windup logic**

### Objective Function (tuning)
- Translates control performance into a **numerical value**

---

## 1) Why the S-curve reference exists

I try to avoid step references in temperature plans since they tend to:

- the controller to **slam the actuator**
- output saturation
- risk of overshoot
- temperature transitions that are either uncomfortable or unsafe

Instead, I create my **S-curve profile** with:

- Slow Acceleration
- faster mid-ramp
- smooth deceleration near the target

---

## Why a 5th-order polynomial?

I implemented the S-curve using a **5th-order polynomial**, because it allows control over:

- position (temperature)
- slope (\(dT/dt\))
- curvature (\(d^2T/dt^2\))

at both the beginning and the end of the ramp.

This helps to ensure that transitions occur smoothly without harsh differences that would otherwise trigger aggressive control maneuvering.

---

## Program phases

A temperature cycle comprises the following three stages:

1. **Rise**: `T_start → T_target` (S-Curve ramp)
2. **Hold**: maintain `T_target`
3. **Fall**: `T_target → T_end` (S-curve ramp)

This is suited for temperature control in therapy or program applications where smooth transitions between levels of temperature and temperature plateaus are critical.

---

## 2) Actuator constraints

A common mistake is to design a controller that looks perfect in simulation but fails on real hardware.

Real actuators are limited by the following:

- **Saturation** (e.g., PWM 0–100%)
- **Rate limits** (outputs should not jump abruptly)
- **Safety boundaries** (Temperature Limits, Sensor Failures)

If these constraints are disregarded:

- Integrator windup can occur
- releasing saturation produce a large overshoot
- control behavior becomes noisy or unstable

Therefore, the controller/output stage is required to process:

- saturation explicitly
- **anti-windup** behavior
- optional output smoothing or rate limiting

---

## 3) Objective function

- Optimizing for tracking error alone can result in aggressive control or saturation.
- Reducing overshoot alone could make the system too sluggish.
  
So, the objective function is a combination of various terms, such as:

- **Tracking error**: IAE / ISE
- **Overshoot penalty**
- **Settling / speed penalty**
- **Control effort penalty**
- **Smoothness penalty** (e.g., Δu, total variation)
- **Saturation penalty**

---

## Genetic Algorithm tuning (why GA)

With FOPID, the list of parameters increases to  
\(K_p, K_i, K_d, \lambda, \mu\).  
Adding constraints makes the optimization problem non-convex with local minima.

I therefore employ a **Genetic Algorithm (GA)** because:

- It does not require gradients
- it explores the search space globally
- it deals with discontinuities (constraint penalties) quite effectively

GA tests each controller through a simulated game with the objective function used for evaluation.  
In other words, I am tuning the **exact controller I plan to implement**, not an abstract model.

---

## Fractional operator implementation: ORA + balanced reduction

Fractional operators such as \(s^\alpha\) cannot be directly realized in the discrete domain.  
I approximate them using the **Oustaloup Recursive Approximation (ORA)** within the frequency band \([\omega_L, \omega_H]\).

ORA converts the fractional operator into a **rational transfer function**, that can be simulated by MATLAB software.

However, the disadvantage is that ORA can become high order, causing:

- slower simulations (especially problematic inside GA)
- heavy state updates
- numerical complexity

To fix this issue, I employ **balanced model reduction** with MATLAB's **balred**.

---

## HSV logic

Balanced reduction is based on **Hankel Singular Values (HSV)**:

- large HSV → state strongly affects input–output behavior (keep it)
- small HSV → state has negligible effect (can be safely removed)

I analyze the decay of the HSV system and select a reduced order that maintains the major behavior by eliminating the unnecessary states.

---

## MatchDC and StateElimMethod

Thermal control is dominated by **low-frequency behavior**.  
If the reduction causes any change in the DC gain, it could impair the regulation around the desired temperature.

Therefore, I explicitly configure options for reduction, such as:

- **`StateElimMethod`**: decides the elimination of states
- **`MatchDC`**: preserves DC gain consistency

The aim is to achieve a lower order of models without compromising the performance in the steady state.

---

## App Designer integration

I used an **object-oriented structure** (e.g., `FOPID`, `SCurveProfile`) since I developed a MATLAB Application Designer **application** for this project.

This allows:

- persistent controller/profile objects
- clean parameter updates from callbacks
- repeatable simulations
- organized plotting and logging

ORA and balanced reduction are implemented as reusable build steps rather than duplicated code inside callbacks.

---

## Will it work on Teensy 4.0?

Portability is one of the design objectives.

Important factors for hardware include:

- fixed sampling time (`Ts`)
- reasonable computational load
- saturation and anti-windup
- sensor noise handling (derivative terms amplify noise)

Though the actual plant would be different from their simulation, this framework offers:

- a realistic smooth reference
- a controller compatible with actuator limits
- a tuning approach that corresponds with real-world considerations

---

## Repository contents (logical view)

- **FOPID class**
  - `Kp, Ki, Kd, λ, μ`
  - discrete update logic
  - optional anti-windup and output limits
- **SCurveProfile class**
  - `T_begin, T_goal, T_finish`
  - `t_rise, t_hold, t_fall`
- **objectiveFunction**
  - returns a scalar cost for GA
- **ORA + reduction utilities**
  - ORA construction
  - `balred` with HSV, MatchDC, StateElimMethod
- **Optional App/UI**
  - MATLAB App Designer interface

---

## Conceptual workflow

1. Create a reference profile (temperatures + durations)
2. Initialize FOPID parameters
3. Run simulation and evaluate behavior
4. Optimize parameters using GA
5. Use ORA + balanced reduction to keep implementation lightweight

---

## Design decisions 

- **S-curve instead of step** → smoother and safer thermal control  
- **5th-order polynomial ramps** → smooth start/end behavior  
- **FOPID** → increased tuning flexibility  
- **Constraints and anti-windup** → hardware realism  
- **GA tuning** → robust global optimization  
- **ORA + balanced reduction** → practical fractional implementation  
- **OOP structure** → seamless App Designer integration  

---

## Next steps

- Identify the real plant using experimental data
- Apply the controller on Teensy 4.0
- Re-tune objective weights based on hardware results

---

## Notes

This project is my attempt to build a control solution that can move from **simulation to real hardware** without falling apart.
