# FOPID Temperature Control with S-Curve Reference

This repository holds my configuration for managing a **thermal system** (heating/cooling).  
My objective is **not** to develop “a PID that functions in simulation only”.  
Instead, I aim to design a **controller and reference generator that are reliable, practical, and suitable for real implementation**, specifically targeting a **Teensy 4.0** microcontroller.

The project is composed of three main elements:

- **FOPID controller class** (Fractional-Order PID)
- **S-curve temperature reference generator** (program / ramp planner)
- **Objective function (cost function)** used to tune parameters based on my own control priorities

---

## Why not just a basic step reference with a classic PID?

Thermal systems pose several challenges in practical control applications:

- They respond **slowly** and exhibit **inertia**.  
  Temperature may continue to rise even after power is removed.
- Tracking performance is affected by **delay**, **sensor placement**, and **hysteresis**.
- Many applications (comfort, therapy, safety) **cannot tolerate sudden temperature changes**.
- Real actuators have constraints such as **maximum output**, **rate limits**, and **safety boundaries**.

A traditional PID controller can work, but it usually forces a compromise between:

- **less overshoot ↔ slower response**
- **faster response ↔ more overshoot or aggressive control effort**

I wanted more flexibility, so I moved to **FOPID** and paired it with a **smooth reference profile**.

---

## What led me to use FOPID

FOPID extends the classical PID structure by allowing **fractional (non-integer) orders** for both the integral and derivative terms. Conceptually:

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

I structured the project into clear layers:

### Reference generation (`SCurveProfile`)
- Where should the temperature go over time?
- Produces smooth ramps instead of step jumps

### Controller (`FOPID`)
- Determines how the system tracks the reference

### Constraints (actuator realism)
- Ensures the output is physically feasible  
- Includes **saturation**, **rate limiting**, and **anti-windup logic**

### Objective function (tuning)
- Converts control performance into a **single scalar score**

---

## 1) Why the S-curve reference exists

I avoid step references in temperature programs because they often cause:

- the controller to **slam the actuator**
- output saturation
- increased overshoot risk
- uncomfortable or unsafe temperature transitions

Instead, I generate an **S-curve profile** with:

- slow acceleration
- faster mid-ramp
- smooth deceleration near the target

---

## Why a 5th-order polynomial?

I implemented the S-curve using a **5th-order polynomial**, because it allows control over:

- position (temperature)
- slope (\(dT/dt\))
- curvature (\(d^2T/dt^2\))

at both the beginning and the end of the ramp.

This results in **smooth transitions** with fewer sudden changes that would otherwise provoke aggressive control action.

---

## Program phases

The temperature program consists of three phases:

1. **Rise**: `T_start → T_target` (S-curve ramp)
2. **Hold**: maintain `T_target`
3. **Fall**: `T_target → T_end` (S-curve ramp)

This structure is well suited for therapy or program-based temperature control, where controlled transitions and stable plateaus are essential.

---

## 2) Actuator constraints: avoiding the “simulation trap”

A common mistake is designing a controller that looks perfect in simulation but fails on real hardware.

Real actuators impose limits such as:

- **Saturation** (e.g., PWM 0–100%)
- **Rate limits** (outputs should not jump abruptly)
- **Safety bounds** (temperature limits, sensor fault handling)

If these constraints are ignored:

- integrator windup can occur
- releasing saturation can cause large overshoot
- control behavior becomes noisy or unstable

For this reason, the controller/output stage must handle:

- saturation explicitly
- **anti-windup** behavior
- optional output smoothing or rate limiting

---

## 3) Objective function

Tuning often determines whether a project succeeds or fails.

- Minimizing only tracking error can lead to aggressive control and saturation.
- Minimizing only overshoot can make the system unnecessarily slow.

Therefore, the objective function combines multiple terms, such as:

- **Tracking error**: IAE / ISE
- **Overshoot penalty**
- **Settling / speed penalty**
- **Control effort penalty**
- **Smoothness penalty** (e.g., Δu, total variation)
- **Saturation penalty**

---

## Genetic Algorithm tuning (why GA)

With FOPID, the parameter set expands to  
\(K_p, K_i, K_d, \lambda, \mu\).  
Including constraints makes the optimization problem **non-convex** and full of local minima.

I therefore use a **Genetic Algorithm (GA)** because:

- it does not require gradients
- it explores the search space globally
- it handles discontinuities (constraint penalties) well

GA evaluates each controller by running a simulation and scoring it using the objective function.  
In other words, I am tuning the **exact controller I plan to implement**, not an abstract model.

---

## Fractional operator implementation: ORA + balanced reduction

Fractional operators such as \(s^\alpha\) cannot be implemented directly in discrete time.  
I approximate them using **Oustaloup Recursive Approximation (ORA)** over a frequency band \([\omega_L, \omega_H]\).

ORA converts the fractional operator into a **rational transfer function**, which MATLAB can simulate and convert to state space.

The drawback is that ORA can become **high order**, leading to:

- slower simulations (especially problematic inside GA)
- heavy state updates
- numerical complexity

To address this, I apply **balanced model reduction** using MATLAB’s `balred`.

---

## HSV logic

Balanced reduction relies on **Hankel Singular Values (HSV)**:

- large HSV → state strongly affects input–output behavior (keep it)
- small HSV → state has negligible impact (safe to remove)

I examine the HSV decay and choose a reduced order that preserves the dominant dynamics while removing redundant states.

---

## MatchDC and StateElimMethod

Thermal control is dominated by **low-frequency behavior**.  
If reduction alters the DC gain, steady-state regulation around the target temperature can degrade.

For this reason, I explicitly configure reduction options such as:

- **`StateElimMethod`**: determines how states are removed
- **`MatchDC`**: preserves DC gain consistency

The goal is to reduce model order **without damaging steady-state performance**.

---

## OOP design choice and App Designer integration

I used an **object-oriented structure** (e.g., `FOPID`, `SCurveProfile`) because I built a MATLAB **App Designer** application around this project.

This allows:

- persistent controller/profile objects
- clean parameter updates from callbacks
- repeatable simulations
- organized plotting and logging

ORA and balanced reduction are implemented as reusable build steps rather than duplicated code inside callbacks.

---

## Will this work on Teensy 4.0?

Portability is a core design goal.

Key considerations for hardware include:

- fixed sampling time (`Ts`)
- reasonable computational load
- saturation and anti-windup
- sensor noise handling (derivative terms amplify noise)

While the real plant will differ from simulation, this framework provides:

- a realistic smooth reference
- a controller compatible with actuator limits
- a tuning strategy aligned with real-world priorities

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

## Design decisions (summary)

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
- Implement the controller on Teensy 4.0
- Re-tune objective weights based on hardware results

---

## Notes

This project is my attempt to build a control solution that can move from **simulation to real hardware** without falling apart.
