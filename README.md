# FOPID Temperature Control with S-Curve Reference

This repository is my end-to-end control setup for a **thermal system** (heating/cooling). The goal isn’t “a PID that works in simulation.” The goal is a controller + reference generator that stays **smooth, safe, and realistic**, and can later be moved onto a microcontroller (my target is **Teensy 4.0**).

The project has three main building blocks:

1. **FOPID controller class** (Fractional-Order PID)
2. **S-curve temperature reference generator** (program/ramp planner)
3. **Objective function (cost function)** to tune parameters based on what *I* define as “good control”

---

## Why not a basic step reference + classic PID?

Thermal systems present several practical challenges in real-world control applications:

- They’re **slow and inertial**. Even after cutting power, temperature can keep rising.
- **Delay / sensor placement / hysteresis** can mess up tracking.
- Many applications (comfort, therapy, safety) *don’t want* sudden temperature changes.
- Real actuators are limited: **min/max output, rate limits, safety limits**.

A classic PID can “work,” but you often end up trading:
- less overshoot ↔ slower response  
- faster response ↔ more overshoot / more aggressive control effort

I wanted more flexibility, so I moved to **FOPID** and paired it with a **smooth reference**.

---

## What is FOPID and why I used it

FOPID extends the standard PID idea by allowing the **integral and derivative orders** to be fractional (non-integer). Conceptually:

\[
u(t)=K_p e(t)+K_i \, D^{-\lambda}\{e(t)\}+K_d \, D^{\mu}\{e(t)\}
\]

- \(K_p, K_i, K_d\) are the usual gains
- \(\lambda\) is the **integral order**
- \(\mu\) is the **derivative order**
-  
Those extra degrees of freedom (\(\lambda, \mu\)) let me find a better balance between overshoot, speed, and smoothness than I could with PID alone.

---

## System architecture

I structured the project into clear layers:

1. **Reference generation (`SCurveProfile`)**
   - “Where do I want temperature to go over time?”
   - Smooth ramps instead of step jumps
2. **Controller (`FOPID`)**
   - “How do I drive the plant to follow that reference?”
3. **Constraints (actuator realism)**
   - “Is the output physically applicable?”
   - saturation, rate limiting, anti-windup logic
4. **Objective function (tuning)**
   - Convert control quality into a single score

---

## 1) S-curve reference: why it exists

I avoid step references for temperature programs because step targets usually cause:

- controller “slams” the actuator
- output hits saturation
- overshoot risk increases
- temperature changes feel uncomfortable/unsafe

Instead, I generate a **smooth S-curve profile**:  
slow acceleration → faster mid-ramp → slow down near the target.

### Why a 5th-order polynomial
I implemented the S-curve using a **5th-order polynomial** style approach because it’s a clean engineering compromise:

- It allows controlling boundary conditions not only for position (temperature),
  but also for **slope (dT/dt)** and **curvature (d²T/dt²)** at the start/end.
- Practical result: the ramp starts and ends **smoothly**, with fewer sudden “kicks”
  that tend to create aggressive controller action.

### Program phases
The temperature program is split into three phases:

1. **Rise**: `T_start → T_target` (S-curve ramp)
2. **Hold**: maintain `T_target`
3. **Fall**: `T_target → T_end` (S-curve ramp)

This maps naturally to “therapy/program” style temperature control where you want a controlled transition, a stable plateau, then a controlled exit.

---

## 2) Actuator constraints: avoiding the “simulation trap”

A common trap is building a controller that looks perfect until you run it on hardware.

Real actuators have limits such as:

- **Saturation (min/max):** e.g., PWM 0–100%
- **Rate limiting:** output shouldn’t jump instantly
- **Safety bounds:** temperature limits, fault handling (sensor errors), etc.

If constraints are ignored:
- integrator windup happens
- when saturation releases, the system can overshoot hard
- control becomes noisy/aggressive

That’s why the controller/output stage needs to handle:
- saturation cleanly
- (ideally) **anti-windup**
- optionally output smoothing / rate limits

---

## 3) Objective function

Tuning is where most projects either become clean or messy.

I wrote an objective function

- If you only minimize error, you may get aggressive control and saturation.
- If you only minimize overshoot, you may get a system that’s unnecessarily slow.

So the cost function can combine multiple terms, for example:

- **Tracking error**: IAE / ISE (integral of |e| or e²)
- **Overshoot penalty**: extra punishment for going above target
- **Settling / speed penalty**: discourage being too slow
- **Control effort penalty**: discourage huge u(t)
- **Smoothness penalty**: discourage jumpy output (Δu, total variation, etc.)
- **Saturation penalty**: punish staying saturated too long

---

## Genetic Algorithm tuning (why GA is in this project)

With classical PID, tuning can often be done with local methods because the search space is small. With FOPID, the parameter space is larger (`Kp, Ki, Kd, λ, μ`), and once constraints/penalties are included the optimization landscape becomes messy: nonlinear, non-convex, and full of local minima.

That’s why I use a **Genetic Algorithm (GA)** as the optimizer:

- it does not require gradients,
- it explores the space more globally (less likely to get stuck),
- it handles discontinuities well (saturation penalties, constraint violations).

In this repo, GA evaluates a candidate controller by running the simulation and scoring it using the objective function. In other words, I’m not tuning an abstract controller — I’m tuning the exact controller structure I actually plan to implement.

---

## Fractional operator implementation: ORA + `balred` (balanced reduction)

Fractional terms like \(s^\alpha\) cannot be implemented directly in a discrete-time loop. To make FOPID usable, I approximate fractional operators using **Oustaloup Recursive Approximation (ORA)** over a chosen frequency band \([ \omega_L, \omega_H ]\). ORA turns \(s^\alpha\) into a rational transfer function that MATLAB can simulate and convert into state-space.

The practical problem: ORA can become **high order**. High order means:
- slower simulation (painful inside GA),
- heavier state updates (painful for embedded),
- more numerical overhead and debugging effort.

So after ORA, I apply **balanced model reduction** using MATLAB’s `balred` to compress the approximation while keeping the behavior that matters.

---

## HSV logic

Balanced reduction relies on **Hankel Singular Values (HSV)**. HSV gives a clean, practical interpretation:

- large HSV → state strongly affects input–output dynamics (keep it)
- small HSV → state contributes very little (safe to remove)

In practice, I look at the HSV drop-off and choose a reduced order that keeps the important dynamics without carrying dead weight. This step is one of the main reasons the controller becomes feasible for repeated tuning runs and later embedded deployment.

---

## `MatchDC` and `StateElimMethod`: protecting steady-state behavior

Thermal control is dominated by low-frequency behavior. If the reduced approximation shifts steady-state gain, you can end up with biased regulation around the target temperature.

That’s why I configure reduction options intentionally, including:

- **`StateElimMethod`**: controls how states are eliminated during reduction
- **`MatchDC`**: enforces DC gain matching so steady-state behavior stays consistent

The intention is simple: reduce order without breaking the near-DC behavior that thermal regulation actually depends on.

---

## OOP design choice (and why it matters for my App Designer application)

I used an **object-oriented structure** (e.g., `FOPID`, `SCurveProfile`) because I built a MATLAB **App Designer** application around this project. In a UI-driven workflow:

- the app holds persistent instances of the controller and profile objects,
- callbacks can update parameters cleanly,
- simulations remain repeatable,
- plotting/logging is easier to organize.

Keeping the approximation and reduction steps modular also matters here: ORA + `balred` becomes a reusable “build step” that the app can call, instead of copy/pasted code scattered across callbacks.

---

## 4) Will this work on Teensy 4.0?

That’s the point of the repo design: it’s meant to be **portable**.

What matters for real hardware:

- **Fixed sampling time (Ts)** and consistent update loop
- computational load (ORA order + reduced order must be reasonable)
- saturation + anti-windup
- sensor noise handling (derivative terms amplify noise → filtering/limiting matters)

The real plant will never match simulation perfectly, but this framework gives me:
- a realistic, smooth reference
- a controller that can be constrained like real hardware
- a tuning method that matches real priorities

So when I move to Teensy, I’m not reinventing the logic—just adapting implementation details.

---

## Repository contents (logical view)

(Exact filenames may vary, but this is the structure.)

- **`FOPID` class**
  - parameters: `Kp, Ki, Kd, λ, μ`
  - discrete update step / internal states
  - (optional) anti-windup & output limiting logic
- **`SCurveProfile` class**
  - `T_start, T_target, T_end`
  - `t_rise, t_hold, t_fall`
  - smooth 5th-order polynomial based ramps
- **`objectiveFunction`**
  - runs simulation and returns a single scalar cost
  - used by GA during tuning
- **ORA + reduction utilities**
  - build ORA approximation for fractional operators
  - reduce order using `balred` with HSV guidance
  - options like `MatchDC` / `StateElimMethod`
- (optional) **App/UI**
  - App Designer interface for quick experiments and visualization

---

## How to use (conceptual workflow)

1. Create a reference profile:
   - define temperatures and durations (rise/hold/fall)
2. Pick initial FOPID parameters:
   - `Kp, Ki, Kd, λ, μ`
3. Run simulation / test:
   - reference vs measured temperature
   - control output `u(t)`
   - saturation/overshoot/smoothness behavior
4. Tune using GA + objective function:
   - GA searches parameters to minimize the objective score
   - ORA + `balred` keep the fractional implementation realistic and lightweight

---

## Design decisions

- **S-curve instead of step** → smoother, safer, more realistic thermal programs  
- **5th-order polynomial ramps** → smooth start/end behavior, reduced “shock”  
- **FOPID** → more tuning freedom than classic PID  
- **Constraints + anti-windup mindset** → prevents hardware-unfriendly behavior  
- **GA tuning** → practical global search for a non-convex FOPID problem  
- **ORA + `balred` (HSV, MatchDC)** → implement fractional operators without carrying a high-order model  
- **OOP structure** → integrates cleanly with my MATLAB App Designer application  

---

## Next steps (my roadmap)

- Identify the real plant using experimental data (step test / data-driven fitting)
- Implement the final control loop on Teensy 4.0:
  - fixed Ts scheduling
  - sensor filtering
  - safety/fault states
- Re-tune objective weights using real hardware behavior

---

## Notes

It’s my attempt to build a control solution that can be **moved from simulation to real hardware** without falling apart.
