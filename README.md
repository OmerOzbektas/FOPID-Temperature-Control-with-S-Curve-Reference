# **FOPID Temperature Control with S-Curve Reference**

This repository contains my complete setup for controlling a thermal system (**heating/cooling**). My goal is not to create *“a PID that works in simulation.”* Instead, I aim for a controller and reference generator that are safe and realistic and implement it with a microcontroller, specifically the **Teensy 4.0**.

The project consists of three main components:

- **FOPID controller class** (*Fractional-Order PID*)
- **S-curve temperature reference generator** (*program/ramp planner*)
- **Objective function (cost function)** to tune parameters based on my needs.

---

## **Why not just a basic step reference with a classic PID?**

Thermal systems present some challenges in real-world control:

- They are **slow** and have **inertia**. Even after cutting power, temperature can continue to rise.
- **Delay**, **sensor placement**, and **hysteresis** can disrupt tracking.
- Many applications (comfort, therapy, safety) do not tolerate sudden temperature changes.
- Real actuators have limitations, including **minimum/maximum output**, **rate limits**, and **safety limits**.

A classic PID can work, but it often results in a trade-off between:

- less overshoot and a slower response  
- a faster response and more overshoot or aggressive control effort  

I wanted more flexibility, so I switched to **FOPID** and paired it with a **smooth reference**.

---

## **Why Did I use FOPID**

FOPID builds on the standard PID by allowing **fractional (non-integer) orders** for both the integral and derivative components. Conceptually:

\[
u(t)=K_p e(t)+K_i \, D^{-\lambda}\{e(t)\}+K_d \, D^{\mu}\{e(t)\}
\]

- **\(K_p, K_i, K_d\)** are the usual gains  
- **\(\lambda\)** is the derivative order  
- **\(\mu\)** is the Integral order  

These extra degrees of freedom (**\(\lambda, \mu\)**) allow me to get a better balance between **overshoot**, **speed**, and **smoothness** than with PID alone.

---

## **System architecture**

I organized the project into clear layers:

- **Reference generation (`SCurveProfile`)**  
  *Where do I want the temperature to go over time?*
- **Controller (`FOPID`)**  
  *How do I make the system follow that reference?*
- **Constraints (actuator realism)**  
  *Is the output physically feasible?*  
  This includes **saturation**, **rate limiting**, and **anti-windup logic**.
- **Objective function (tuning)**  
  Converts control quality into a **single score**.

---

## **1) S-curve reference: why it exists**

I avoid step references for temperature programs since step targets often lead to:

- the controller slamming the actuator  
- outputs hitting saturation  
- increased overshoot risk  
- temperature changes feeling uncomfortable or unsafe  

Instead, I create a smooth **S-curve profile**:

- slow acceleration  
- faster mid-ramp  
- slowdown near the target  

### **Why a 5th-order polynomial?**

I implemented the S-curve using a **5th-order polynomial**

It allows me to control not only **position (temperature)** but also **slope (dT/dt)** and **curvature (d²T/dt²)** at both the start and end.

### **Program phases**

The temperature program consists of three phases:

- **Rise:** `T_start → T_target` (S-curve ramp)  
- **Hold:** maintain `T_target`  
- **Fall:** `T_target → T_end` (S-curve ramp)  

This structure aligns with therapy or program-type temperature control, where controlled transitions, stable plateaus, and controlled exits are essential.

---

## **2) Actuator constraints: avoiding the “simulation trap”**

A common issue is building a controller that seems perfect until tested on real hardware.

Real actuators have limits such as:

- **Saturation (minimum/maximum):** for example, PWM `0–100%`  
- **Rate limiting:** the output should not jump instantly  
- **Safety bounds:** including temperature limits and handling faults (sensor errors)  

If constraints are ignored:

- integral windup can occur  
- if saturation releases, the system can overshoot significantly  
- control can become noisy or aggressive  

This is why the controller/output stage must manage:

- saturation effectively  
- ideally, prevent integral windup  
- optionally, output smoothing and rate limits  

---

## **3) Objective function**

Tuning can make or break a project.

If you only minimize error, you may end up with aggressive control and saturation.  
If you only minimize overshoot, you may create a system that is unnecessarily slow.

So the cost function combines multiple terms, for instance:

- **Tracking error:** IAE or ISE (integral of `|e|` or `e²`)  
- **Overshoot penalty:** extra punishment for exceeding the target  
- **Settling or speed penalty:** discouraging excessive slowness  
- **Control effort penalty:** discouraging large `u(t)` values  
- **Smoothness penalty:** discouraging jumpy outputs (`Δu`, total variation, etc.)  
- **Saturation penalty:** punishing prolonged saturation  

---

## **Genetic Algorithm tuning (why GA is included)**

With classic PID, tuning can often use local methods due to a smaller search space. However, with FOPID, the parameter space expands (`Kp, Ki, Kd, λ, μ`). When considering constraints and penalties, the optimization landscape becomes more complex: **nonlinear**, **non-convex**, and filled with **local minima**.

This is why I use a **Genetic Algorithm (GA)** as the optimizer:

- It does not require gradients  
- It explores the space more thoroughly, making it less likely to get stuck  
- It manages discontinuities well (saturation penalties and constraint violations)  

In this repository, GA evaluates a candidate controller by running the simulation and scoring it using the objective function. In other words, I'm tuning the actual controller I plan to implement, not an abstract version.

---

## **Fractional operator implementation: ORA + `balred` (balanced reduction)**

Fractional terms like \(s^\alpha\) cannot be directly implemented in a discrete-time loop. To make FOPID usable, I approximate fractional operators using **Oustaloup Recursive Approximation (ORA)** over a chosen frequency band \([\omega_L, \omega_H]\). ORA converts \(s^\alpha\) into a rational transfer function that MATLAB can simulate and convert into state-space.

The practical issue: ORA can result in high orders. High orders mean:

- slower simulation (difficult within GA)  
- heavier state updates (challenging for embedded systems)  
- increased numerical overhead and debugging difficulty  

After ORA, I apply balanced model reduction using MATLAB’s **`balred`** to compress the approximation while maintaining critical behavior.

---

## **HSV logic**

Balanced reduction depends on **Hankel Singular Values (HSV)**. HSV offers a clear, practical understanding:

- large HSV indicates that the state strongly influences input-output dynamics (**keep it**)  
- small HSV suggests that the state contributes little (**safe to remove**)  

In practice, I analyze the HSV drop-off and select a reduced order that maintains important dynamics without unnecessary components. This step is vital for repeated tuning runs and later embedded deployment.

---

## **`MatchDC` and `StateElimMethod`: protecting steady-state behavior**

Thermal control is dominated by low-frequency dynamics. If the reduced approximation alters steady-state gain, you might end up with biased regulation around the target temperature.

For this reason, I intentionally configure reduction options, including:

- **`StateElimMethod`**: dictates how states are removed during reduction  
- **`MatchDC`**: ensures DC gain matching so that steady-state behavior remains consistent  

The intent is straightforward: reduce order without compromising the near-DC behavior essential for thermal regulation.

---

## **OOP design choice (and its importance for my App Designer application)**

I used an object-oriented structure (e.g., **`FOPID`**, **`SCurveProfile`**) because I built a MATLAB **App Designer** application around this project. In a UI-driven workflow:

- the app maintains persistent instances of the controller and profile objects  
- callbacks can update parameters smoothly  
- simulations remain repeatable  
- plotting and logging are easier to organize  

Keeping the approximation and reduction steps modular is also important: ORA and `balred` become reusable **build steps** that the app can call, rather than duplicated code scattered across callbacks.

---

## **4) Will this work on Teensy 4.0?**

That’s the primary design point of this repository: it is meant to be **portable**.

What matters for real hardware includes:

- **Fixed sampling time (Ts)** and a consistent update loop  
- **Computational load** (ORA order and the reduced order must be manageable)  
- **Saturation and anti-windup**  
- Handling **sensor noise** (derivative terms amplify noise, so filtering and limiting are vital)  

The real plant will never perfectly match the simulation, but this framework provides:

- a realistic, smooth reference  
- a controller that can be constrained like actual hardware  
- a tuning method that aligns with real priorities  

So when I transition to Teensy, I’m not completely overhauling the logic—just adjusting the implementation details.

---

## **Repository contents (logical view)**

(Exact filenames may change, but this is the structure.)

- **FOPID class**
  - parameters: `Kp, Ki, Kd, λ, μ`
  - discrete update step and internal states
  - optional anti-windup and output limiting logic
- **SCurveProfile class**
  - `T_start, T_target, T_end`
  - `t_rise, t_hold, t_fall`
  - smooth 5th-order polynomial-based ramps
- **objectiveFunction**
  - runs simulations and returns a single scalar cost
  - used by GA during tuning
- **ORA + reduction utilities**
  - builds ORA approximations for fractional operators
  - reduces order using `balred` with HSV guidance
  - includes options like `MatchDC` and `StateElimMethod`
- **Optional App/UI**
  - App Designer interface for quick experiments and visualization

---

## **How to use (conceptual workflow)**

- Create a reference profile: define temperatures and durations (rise/hold/fall)  
- Set initial FOPID parameters: `Kp, Ki, Kd, λ, μ`  
- Run simulations/tests: compare reference versus measured temperature, control output `u(t)`, saturation/overshoot/smoothness behavior  
- Tune using GA and the objective function: GA searches parameters to minimize the objective score  
- ORA and `balred` maintain a realistic and lightweight fractional implementation  

---

## **Design decisions**

- **S-curve instead of step** → smoother, safer, more realistic thermal programs  
- **5th-order polynomial ramps** → smooth start/end behavior, reduced shock  
- **FOPID** → more tuning flexibility than classic PID  
- **Constraints and anti-windup focus** → prevent undesirable hardware behavior  
- **GA tuning** → practical global search for the non-convex FOPID problem  
- **ORA and `balred` (HSV, MatchDC)** → implement fractional operators without carrying a high-order model  
- **OOP structure** → integrates well with my MATLAB App Designer application  

---

## **Next steps (my roadmap)**

- Identify the real plant using experimental data (step test/data-driven fitting)  
- Implement the final control loop on Teensy 4.0:
  - fixed Ts scheduling  
  - sensor filtering  
  - safety and fault states  
- Re-tune objective weights based on real hardware behavior  

---

## **Notes**

This is my attempt to create a control solution that can transition from simulation to real hardware without breaking down.
