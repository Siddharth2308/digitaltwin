# kerala_bot_sim

Per-axis **stepper + timing-belt** dynamics simulator for the gantry machine. Given a
commanded trajectory and the machine's physical parameters, it predicts the **actual**
executed motion , including whether the motor loses steps , so you can tell if a trajectory
plan is executable *before* running it. Pairs with the `kerala_bot_description` RViz twin.

## What it models

Each axis (independent Cartesian, open-loop stepper) is a 2-mass system:

```
motor rotor  --(belt spring + damper)-->  carriage
tau_motor = T_pk(omega) * sin( Nr * (theta_cmd - theta_m) )
```

Step loss emerges from the physics: when the load demands more torque than the speed-
dependent pull-out `T_pk(omega)`, the electrical lead angle passes 90° and the motor slips
a pole , a permanent, cumulative position error. That is the "offsets stack up at high
speed" symptom. When any axis crosses pull-out the trajectory is reported **INFEASIBLE**
at that instant (the post-slip open-loop path is not trustworthy and is not integrated
through).

## Pure-Python core (no ROS needed)

Runs anywhere with numpy/scipy/matplotlib/pyyaml:

```bash
cd colcon_ws/src/kerala_bot_sim

# feasible demo (placeholder params) -> output/overlay.png + output/sim_log.csv
python -m kerala_bot_sim.run_demo

# push it until it fails -> reports "INFEASIBLE: axis Y pull-out at t=..."
python -m kerala_bot_sim.run_demo --feed 0.5 --accel 8.0

# progressive drawing (saves output/overlay.gif, or shows live with a GUI backend)
python -m kerala_bot_sim.run_demo --animate

# use your measured numbers instead of placeholders
python -m kerala_bot_sim.run_demo --params ../../../docs/params_template.yaml
```

Output:
- `output/overlay.png` , commanded path (blue) vs predicted actual (red) on one XY canvas.
- `output/sim_log.csv` , `t, x_cmd, y_cmd, x_act, y_act`; consumed by the RViz player.
- console , FEASIBLE (with peak tracking error + permanent offset per axis) or INFEASIBLE
  (with the axis and time of pull-out).

**Every parameter still on a placeholder default is printed at startup** , nothing here is
your real machine until `docs/params_template.yaml` is filled in from the bench tests in
`docs/characterization_plan.md`.

## RViz playback (needs ROS2 Humble, WSL)

Replays a logged simulation into the RViz twin, mapping machine/bed coordinates to the URDF
joint limits via `config/axis_map.yaml`. Run this *instead of* `joint_state_publisher_gui`
(both drive `/joint_states`).

```bash
cd colcon_ws
colcon build
source install/setup.bash
ros2 launch kerala_bot_sim playback.launch.py \
    log:=$(pwd)/src/kerala_bot_sim/output/sim_log.csv rate:=1.0
```

`axis_map.yaml` sets, per machine axis, which URDF joint it drives, the bed-coordinate range,
the joint-value range, and whether to play `x_act`/`y_act` (predicted) or `x_cmd`/`y_cmd`
(intended). Adjust the ranges to your real workspace.

## Live machine state (no simulator) , `bringup.launch.py`

Drive the twin from the machine's **real** position feedback instead of the sliders. The
`state_listener` node subscribes per `config/machine_state.yaml`, scales each machine axis
from its physical workspace onto the URDF joint limits, and publishes `/joint_states`.

```bash
# edit config/machine_state.yaml: set the input mode/topic and each axis's machine_min/max
ros2 launch kerala_bot_sim bringup.launch.py            # uses the packaged config
ros2 launch kerala_bot_sim bringup.launch.py config:=/abs/path/machine_state.yaml
```

Input modes (set `input.mode`):

| mode | message | topic(s) |
|---|---|---|
| `point` | `geometry_msgs/Point` (`.x .y .z`) | one topic (`input.topic`) |
| `point_stamped` | `geometry_msgs/PointStamped` | one topic (`input.topic`) |
| `float_axes` | `std_msgs/Float64` per axis | one topic per axis (each axis's `topic`) |

`input.units` (`m`/`mm`) converts incoming values; `z` is disabled until the `z_axis` joint
exists in the URDF. Quick test without hardware:

```bash
ros2 topic pub -r 10 /machine/position geometry_msgs/msg/Point "{x: 0.6, y: 0.47, z: 0.0}"
```

> Only one publisher should own `/joint_states` at a time , run `state_listener`
> (live), `rviz_player` (sim playback), or `joint_state_publisher_gui` (manual), not two.

## Module map

| file | role |
|---|---|
| `params.py` | load `params_template.yaml`, fill blanks with flagged placeholder defaults |
| `model.py` | per-axis 2-mass ODE + stepper pole-slip torque |
| `trajectory.py` | load/synthesize position-only setpoints; reconstruct vel/accel (Savitzky-Golay) |
| `simulate.py` | integrate both axes; detect pull-out; log commanded vs actual |
| `overlay_plot.py` | XY overlay, static PNG and progressive animation |
| `mapping.py` | pure machine→joint scaling helpers (shared, unit-tested) |
| `rviz_player.py` | ROS2 node: replay sim log -> `/joint_states` with bed→joint mapping |
| `state_listener.py` | ROS2 node: live position topic(s) -> scaled `/joint_states` |
| `run_demo.py` | end-to-end glue |

## Known limitation / next fidelity step

The model is accurate in the **tracking regime** (below pull-out) and gives a clean
feasibility verdict at the boundary. It does **not** predict the exact path *after* a slip
begins , quantifying accumulated offset through repeated slips is Tier-2 work that needs the
measured torque-speed curve (Test 6) and belt stiffness/damping (Tests 4–5), validated
against the lost-step boundary map (Test 7). Until then, treat INFEASIBLE as "this plan will
lose steps here , fix the profile or the current setting," not as a precise offset figure.
