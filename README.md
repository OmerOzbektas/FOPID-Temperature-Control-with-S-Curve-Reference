# FOPID Temperature Control with S-Curve Reference

This repository is my end-to-end control setup for a **thermal system** (heating/cooling). The goal isn’t “a PID that works in simulation.” The goal is a controller + reference generator that stays **smooth, safe, and realistic**, and can later be moved onto a microcontroller (my target is **Teensy 4.0**).

The project has three main building blocks:

1. **FOPID controller class** (Fractional-Order PID)
2. **S-curve temperature reference generator** (program/ramp planner)
3. **Objective function (cost function)** to tune parameters based on what *I* define as “good control”

I’m writing this from the perspective of a **4th-year Electrical & Electronics Engineering student** who wants something that can survive real hardware constraints (PWM limits, actuator saturation, smoothness, safety), not just look pretty on plots.

---

## Why not a basic step reference + classic PID?

Thermal plants are deceptively annoying in practice:

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

**My engineering reason (simple):**  
Those extra degrees of freedom (\(\lambda, \mu\)) let me find a better balance between overshoot, speed, and smoothness than I could with PID alone.

### Practical note (microcontroller reality)
Fractional operators aren’t implemented “directly” in real time. You need a **discrete approximation** (filters / recursive forms / limited order approximations). The FOPID class in this repo is written with that reality in mind: the design is meant to be portable to embedded targets rather than being purely symbolic/continuous-time math.

---

## System architecture (how I think about it)

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
   - “What does *good* mean?”
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
- **Rate limiting:** output shouldn’t jump instantly (power electronics, EMI, comfort, safety)
- **Safety bounds:** temperature limits, fault handling (sensor errors), etc.

If constraints are ignored:
- integrator windup happens
- when saturation releases, the system can overshoot hard
- control becomes noisy/aggressive

That’s why the controller/output stage needs to handle:
- saturation cleanly
- (ideally) **anti-windup**
- optionally output smoothing / rate limits

Even if the exact implementation differs between MATLAB and Teensy firmware, the design philosophy stays the same: **the controller must respect what the hardware can do.**

---

## 3) Objective function: turning “good control” into a number

Tuning is where most projects either become clean or messy.

I wrote an objective function because “good control” is not a single metric:

- If you only minimize error, you may get aggressive control and saturation.
- If you only minimize overshoot, you may get a system that’s unnecessarily slow.

So the cost function can combine multiple terms, for example:

- **Tracking error**: IAE / ISE (integral of |e| or e²)
- **Overshoot penalty**: extra punishment for going above target
- **Settling / speed penalty**: discourage being too slow
- **Control effort penalty**: discourage huge u(t)
- **Smoothness penalty**: discourage jumpy output (Δu, total variation, etc.)
- **Saturation penalty**: punish staying saturated too long

This is basically me encoding my engineering priorities into math so parameter search / tuning becomes systematic instead of guesswork.

---

## 4) Will this work on Teensy 4.0?

That’s the point of the repo design: it’s meant to be **portable**.

What matters for real hardware:

- **Fixed sampling time (Ts)** and consistent update loop
- computational load (FOPID approximation order must be reasonable)
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
  - used for tuning/optimization experiments
- (optional) **App/UI**
  - quick parameter changes, visualization, experiments

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
4. Tune using the objective function:
   - adjust weights based on what matters (comfort vs speed vs energy)

---

## Design decisions (short summary)

- **S-curve instead of step** → smoother, safer, more realistic thermal programs  
- **5th-order polynomial ramps** → smooth start/end behavior, reduced “shock”  
- **FOPID** → more tuning freedom than classic PID  
- **Constraints + anti-windup mindset** → prevents hardware-unfriendly behavior  
- **Objective function** → systematic tuning based on real priorities  

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

This repo is not just “control theory practice.”  
It’s my attempt to build a control solution that can be **moved from simulation to real hardware** without falling apart.
