# System Identification & Characterization Plan

Goal: identify the minimum set of physical parameters needed to drive a per-axis
2-mass (motor rotor ↔ belt spring ↔ carriage) dynamic model that predicts, for a given
commanded trajectory, the **actual executed motion including accumulated lost-step
offset** , so a trajectory can be judged executable *before* it is run.

Machine assumptions (confirmed): independent Cartesian (one motor + one belt per axis,
X and Y dynamically independent), **open-loop steppers** (lost steps are permanent and
cumulative), setpoints arrive as **position only**.

Do **X and Y separately** , they are not symmetric. The Y axis moves the entire X gantry,
so its moving mass (and therefore its torque demand and its slip behaviour) is much larger.
Expect Y to be the worse offender.

---

## 0. The enabling capability: measuring ACTUAL position without encoders

Open-loop means the controller has no idea where the carriage really is, so every test
below that compares commanded vs. actual needs an **external** position reference. Pick
per test from cheapest → best:

| Tool | Range | Resolution | Good for |
|---|---|---|---|
| Dial indicator (DTI) | small (~10–25 mm) | ~µm | belt stiffness, small displacements |
| Digital calipers / steel rule + pointer | full travel | ~0.05 mm | lost-step end-position error |
| Home switch + step counting (out-and-back) | full travel | 1 step | *net* lost steps over a cycle (fast screening) |
| Accelerometer on carriage (even a phone, or a cheap ADXL345 logging fast) | , | , | ringing frequency & decay (belt resonance/damping) |
| External linear scale / optical encoder (if available) | full travel | best | everything; ideal if you can borrow one |

The home-switch out-and-back is the fastest screening method: command a long fast move
out and back to the switch; if it doesn't re-trigger the switch at the expected step
count, steps were lost that cycle.

---

## Test 1 , Geometry & ratios (trivial, do first)

**Purpose:** pulley pitch radius `r`, and therefore mm-per-motor-rev and mm-per-step.
**Method:** from the belt/pulley spec , `r = (teeth × belt_pitch) / (2π)`. E.g. a 20-tooth
GT2 (2 mm pitch) pulley → circumference 40 mm → `r = 40/(2π) = 6.366 mm`.
**Compute:** mm/rev = 2πr; mm/step = mm/rev ÷ (steps_per_rev × microstep).
**Notes:** verify by commanding a large known move and measuring with calipers , if the
measured distance disagrees with mm/step × steps, your microstepping assumption is wrong.

## Test 2 , Moving mass & inertia (mostly from CAD)

**Purpose:** carriage moving mass `m` per axis, rotor inertia `J_m`.
**Method:** `m` from the CAD mass-properties of everything that moves with that axis
(for Y: the X gantry + tool head + its own carriage). `J_m` from the motor datasheet.
**Notes:** if CAD mass-properties aren't trustworthy (materials not assigned), weigh the
moving assembly on a scale as a cross-check. Reflected inertia at the motor is
`J_m + J_pulleys + m·r²`; the **load/motor inertia ratio** `m·r² / J_m` is a headline
number , high ratio + compliant belt = resonance trouble.

## Test 3 , Friction: Coulomb + viscous

**Purpose:** `F_coulomb` (breakaway/kinetic) and `b_visc` (viscous) in
`F_friction = F_coulomb·sign(v) + b_visc·v`.
**Equipment:** force gauge OR a pulley + hanging weights; the carriage free to move.
**Procedure:**
1. **Breakaway (Coulomb):** de-energize the motor (or disconnect the belt) so the carriage
   moves freely. Pull with a force gauge, or hang increasing weight over an idler pulley,
   until the carriage *just* starts to move. `F_coulomb = m_weight · g` (or the gauge
   reading at the moment of motion).
2. **Viscous:** pull the carriage at a few *constant* speeds and record force at each:
   `F(v) = F_coulomb + b_visc·v`. Slope of F vs. v = `b_visc`.
**Notes:** viscous term is usually small on linear guides; if constant-speed pulling by
hand is impractical, it's acceptable to set `b_visc ≈ 0` initially and let the resonance
test (Test 5) refine total damping. Coulomb is the term that matters most , get it well.

## Test 4 , Belt stiffness `k` (static, and its position dependence)

**Purpose:** belt spring constant `k` (N/m) , the compliance that causes lag/windup/ringing.
**Equipment:** DTI on the carriage, force gauge or hanging weights, motor **energized**
(holding torque locks the pulley).
**Procedure:** with the motor holding, apply a known force `F` to the carriage along the
travel axis and read the deflection `Δx` on the DTI. `k = F / Δx`. Repeat at **2–3 carriage
positions** (near the motor, mid-travel, far), because the free belt span changes with
position and a longer span is softer (`k ∝ 1/L_span`).
**Notes:** report `k` at each position; the model can interpolate `k(x)`. Use several
force levels at each position and fit a line , the slope is `k`, and a non-zero intercept
flags backlash/slack to note separately.

## Test 5 , Belt resonance & damping (dynamic , the elegant one)

**Purpose:** cross-checks `k`, and gives belt damping `c` and the natural frequency `f_n`
the model must resolve. Also directly validates the resonance mechanism that triggers slip.
**Equipment:** accelerometer on the carriage (phone app or ADXL logging ≥1 kHz).
**Procedure:** with the motor holding position, give the carriage a sharp tap (or command
a tiny fast step) and record the free ringing. From the trace:
- oscillation frequency → `f_n`, so `k_eff = m·(2π f_n)²` (compare to Test 4).
- decay of successive peaks → log-decrement `δ = (1/n)·ln(A₀/Aₙ)`, damping ratio
  `ζ = δ/√(4π² + δ²)`, then `c = 2ζ·√(k_eff·m)`.
**Notes:** this sets the simulator's internal timestep , integrate at ≥ ~10× `f_n`
(typically ≥1 kHz) and interpolate the coarse position setpoints up to that rate, or the
sim will miss the ringing that causes the slip.

## Test 6 , Torque-speed pull-out curve `T_pk(ω)`

**Purpose:** the governing curve for step loss , available torque as a function of speed,
at *your* actual supply voltage, current limit, and microstepping.
**Method (datasheet anchor + empirical points):**
1. Take the motor's published pull-out torque curve as the nominal *shape*.
2. Anchor it to reality: hang a known resistive load (weight `W` over the axis → resistive
   torque `W·r`), and with **near-zero acceleration** (long slow ramp so accel torque is
   negligible) find the maximum steady speed the motor sustains before stalling. That gives
   one point `(ω_stall, T = W·r)`. Sweep `W` for several points and scale the datasheet
   curve to fit.
**Notes:** the curve shifts strongly with supply voltage and current setting, so the
empirical anchoring matters more than the datasheet shape. If you can't apply clean loads,
Test 7 substitutes for this , it maps the *usable* envelope directly under real inertia.

## Test 7 , Lost-step boundary map (master test: parameter anchor + validation set)

**Purpose:** trace, per axis, the boundary in (peak velocity, acceleration) space between
"executes cleanly" and "loses steps." This is simultaneously the best real-world anchor for
the torque envelope AND the ground-truth dataset the finished model must reproduce.
**Equipment:** calipers or home-switch out-and-back for end-position error.
**Procedure:** for a grid of (peak velocity `v`, acceleration `a`) values, command a fixed
out-and-back move N times (N ≈ 10–20 to accumulate detectable offset), then measure net
position error at the nominal home. Record pass (≈0 error) / fail (growing offset), and the
offset magnitude on fails. Do the grid for **both axes independently**, and also for a few
**diagonal** moves (both axes commanded together) since that's where your real jobs fail.
**Notes:** keep this data , model tuning succeeds when the simulator's predicted
pass/fail boundary and offset magnitudes match this map.

---

## Recommended order

1. **Test 1** (geometry) , instant, unblocks the ratios.
2. **Test 2** (mass/inertia) , mostly from CAD; can be done off-machine.
3. **Test 7** (lost-step map) , do early: it's the ground truth, and it tells you how bad
   the problem actually is and which axis/regime to focus modelling effort on.
4. **Test 5** (resonance) , one accelerometer trace gives `f_n`, `k_eff`, `c` together.
5. **Test 4** (static `k`) , confirms Test 5 and captures position dependence.
6. **Test 3** (friction) , Coulomb is the priority; viscous can start at 0.
7. **Test 6** (torque-speed) , anchor the datasheet curve; partly redundant with Test 7.

## Note on position-only setpoints (software, for the build phase)

Since setpoints are position-only, the simulator will reconstruct velocity and acceleration
by smoothing then differentiating (e.g. Savitzky–Golay), OR by fitting the trapezoid/S-curve
segments analytically if the planner's profile shape is known. Double-differentiating raw
position is noisy and noise → bad torque estimates, so the **setpoint log sample rate**
matters: log as fast as the controller allows. This is handled in code, not at the bench,
but flags a data-collection requirement now.
