# GEM e4 Hardware Cheatsheet — `gem_ws` (ROS 2)

Workspace: `~/CS588/group9/gem_ws`
Vehicle: GEM e4 with Septentrio INS (GNSS + heading), PACMod2 drive-by-wire,
Logitech-style joystick.

This workspace is configured to drive the e4 along a real-drive GNSS
trajectory using a Stanley low-level controller. State comes from
`/navsatfix` + `/insnavgeod`; commands go out on `/pacmod/...`. All
actuation is gated behind the joystick arm sequence (LB+RB).

---

## 0. What's already in this workspace

| Piece | File | Notes |
|---|---|---|
| Reference trajectory | `src/vehicle_drivers/gem_gnss_control/waypoints/lane2_refined.csv` | 802 pts × 0.25 m, 200.25 m closed loop. Schema: `x, y, yaw_rad, s, kappa, v_ref`. Frame: local ENU about (40.092857, -88.235992) — derived from the lane2_2026-04-20 GNSS bag. |
| Controller node | `src/vehicle_drivers/gem_gnss_control/gem_gnss_control/stanley.py` | Hoffmann's Stanley law (front-axle reference) + speed PID + Werling-style sampling replanner. Same I/O surface and joystick safety as the existing `pure_pursuit` node. |
| Replanner module | `src/vehicle_drivers/gem_gnss_control/gem_gnss_control/sampling_replanner.py` | Pure-numpy Frenet sampler (225 candidates: 9 lateral × 5 speed × 5 accel) + 6-term cost evaluator. Constants tuned for the e4 (KAPPA_MAX = 0.27 1/m, V_MAX = 2.5 m/s). |
| Launch | `src/vehicle_drivers/gem_gnss_control/launch/stanley.launch.py` | Loads `${VEHICLE_NAME:-e4}_stanley.yaml`. |
| Per-vehicle config | `config/e4_stanley.yaml`, `config/e2_stanley.yaml` | Wheelbase, GPS antenna offset, Stanley gains, speed PID, ENU origin, obstacle YAML. |
| Obstacle config | `config/obstacles_lane2.yaml` | One static disc on the southern straight (see §9 for placement / GPS). |

So: arm the joystick, the launch reads GPS+INS, the replanner shifts
the path laterally to dodge any known obstacles, and Stanley tracks the
result. To use a different reference, see §5.  To add / move /
remove obstacles, see §9.

---

## 1. Build (run once, and after any code/config/csv change)

```bash
cd ~/CS588/group9/gem_ws
colcon build --symlink-install --packages-select gem_gnss_control
source install/setup.bash
```

`--symlink-install` symlinks Python source so day-to-day edits in
`src/.../stanley.py` don't need a rebuild. The cases that **do** need
a rebuild:
- Adding/removing a `console_scripts` entry point in `setup.py`
- Adding a new launch / yaml / waypoints file under a `data_files` glob
- Editing `package.xml` or any `setup.py` field

A re-source (`source install/setup.bash`) is enough for yaml-only edits.

---

## 2. Pre-flight checks

### 2.a Joystick (mandatory)
The Stanley node refuses to start if no joystick is plugged in.
```bash
python3 -c "import pygame; pygame.init(); pygame.joystick.init(); print('joysticks:', pygame.joystick.get_count())"
```
Buttons used:
- **LB (button 6) + RB (button 7) held together** = ARM (engage pacmod, drop into FORWARD)
- **LB alone** = DISARM (release pacmod)

### 2.b CAN bus + PACMod
The drive-by-wire stack needs CAN running. Use whichever script your
hardware setup uses (one of these two locations is typical):
```bash
# bundled in the workspace
bash ~/CS588/group9/gem_ws/src/utilities/can_start.sh

# OR the desktop helper used by highbay_autoware.sh
sudo bash ~/Desktop/can_start.bash
```

### 2.c GNSS heading initialised
The Septentrio's heading is sometimes unreliable on first boot until
the vehicle moves. After bringing up the GNSS launch (§3, terminal 2):
```bash
ros2 topic echo /insnavgeod --once
```
- The `heading` field should be a sensible compass bearing (0–360°)
  consistent with the vehicle's actual orientation.
- If it's stuck at 0 or junk, joystick-drive forward 5–10 s, then check
  again. If still bad, relaunch `visualization.launch.py`. Worst case,
  reboot the GNSS receiver.

### 2.d Vehicle starting position
`lane2_refined.csv` is a closed 200 m loop. The vehicle should be on or
within a few metres of the loop before arming Stanley — a large initial
offset produces a strong turn-in command. The path's first waypoint is
at hardware ENU `(27.07, -11.17)` facing yaw `-0.07 rad` (≈ -4°).

---

## 3. Run on the real vehicle — TWO EXPERIMENTS

There are two intended experiments, both running on the same workspace.
The only difference is whether the obstacle-avoidance replanner is
active.

### → Experiment 1 — pure trajectory tracking (NO obstacle)

The vehicle follows `lane2_refined.csv` with Stanley alone.  The
replanner is disabled; no obstacle in the highbay.

```bash
bash ~/CS588/group9/gem_ws/src/utilities/run_stanley.sh no_obstacle
```

That's it.  This opens the 5-terminal stack with the Stanley launch
overridden to `obstacles_yaml:=''` (replanner OFF — pure path
tracking).  After the terminals come up, arm with **LB + RB** on the
joystick.  Expected behaviour: the e4 follows the 200 m loop, brakes to
a stop in the last 5 m of the loop (the v_ref ramp at the end of the
CSV).

### → Experiment 2 — trajectory tracking WITH obstacle (replanner ON)

A real obstacle is placed in the highbay at the GPS coordinate below;
the replanner shifts the path laterally to detour around it.

**Step 1 — physically place the obstacle.**  Go to this GPS point in
the highbay (use a phone GPS, RTK, or your hand-held GNSS receiver):

| Quantity | Value |
|---|---|
| **GPS** | **lat 40.0927378° N, lon -88.2357399° W** |
| Hardware ENU `(x, y)` | `(21.4959 m, -13.2378 m)` |
| Disc radius (in `obstacles_lane2.yaml`) | 0.75 m |
| Position along reference | s ≈ 101.50 m, southern straight just past the right-hand U-turn |

A traffic cone, barrel, or 1.5 m × 1.5 m cardboard box works.  Centre
the obstacle on that GPS point — the replanner's collision check uses
its centre + 0.75 m radius.  If you can only place a smaller object,
edit `radius` in `config/obstacles_lane2.yaml` (§9) to match.

**Step 2 — bring up the stack with the replanner active.**

```bash
bash ~/CS588/group9/gem_ws/src/utilities/run_stanley.sh with_obstacle
```

(or just `bash run_stanley.sh` — `with_obstacle` is the default.)

After the terminals come up, arm with **LB + RB**.  Expected
behaviour: as the e4 approaches s ≈ 90 m, the replanner picks a
lateral offset (logged as `target(d=±0.x, ...)`), Stanley tracks the
shifted path around the cone, and the vehicle returns to the
centerline once the obstacle is past.  If the chosen candidate is
collision-free, the log line is **info** level with `ok=Y`.  If the
replanner can't find a clean detour, the log goes **warn** with `ok=N`
— the safety speed cap will brake the vehicle to 0 before the
obstacle.  Be ready to disarm.

### Arming, disarming, shutdown (same for both experiments)

- **Arm**: hold **LB + RB** simultaneously on the joystick.
- **Disarm**: release RB and keep LB held → node logs `Pacmod
  disabled by joystick` and stops publishing.
- **E-stop**: any time, hit the kill switch for a hardware cutoff.
- **Shutdown**: disarm first, then Ctrl-C each terminal in reverse
  order (Stanley → pacmod → joystick → GNSS → sensors), then stop CAN
  if you started it manually.

The Stanley node logs ~2 Hz once enabled.  Without the replanner:
```
x= 27.07 y=-11.17 yaw= -4.2° | e_fa=+0.12 ψe=+0.5° | δ=+1.2° sw= +25.0° | v=1.50 v_ref=1.50 thr=0.18
```
With the replanner active you'll also see, every ~2 s:
```
replan(sampling): N=225 best#=98 cost=89.30 [col=0 goa=15.00 cen=0.32 jrk=0.005 fea=0.05] target(d=-0.80, v=1.50, a=+0.00) max|κ|=0.092 v_min=1.18 d_obs_veh=12.4 s0=89.50 d0=+0.04 ok=Y obs:[@(+21.5,-13.2) r=0.75 v=(+0.00,+0.00)]
```

### What the launcher actually does

`run_stanley.sh` opens 5 gnome-terminals in dependency order with
small delays (~15 s total) between them, each sourcing
`install/setup.bash`.  Each terminal stays open after its launch exits
so you can see errors.

Prereqs the script does NOT do for you:
- CAN bus must already be up (§2.b).
- Joystick must be plugged in (§2.a).
- Workspace must already be built (§1).  The script aborts with a
  clear error if `install/setup.bash` is missing.

Useful env-var overrides:
```bash
WORKSPACE=/some/other/gem_ws bash run_stanley.sh no_obstacle
SLEEP_SENSORS=8 SLEEP_GNSS=8 bash run_stanley.sh with_obstacle
```

If `gnome-terminal` isn't available (non-GNOME desktop, headless box,
remote SSH session), use §3.x manual launch below.

### 3.x Manual launch (fallback for non-GNOME or fine control)

Open one terminal per launch (or a tmux pane each).  Each needs
`cd ~/CS588/group9/gem_ws && source install/setup.bash` first.

| # | Launch |
|---|---|
| 1 | `ros2 launch basic_launch sensor_init.launch.py` |
| 2 | `ros2 launch basic_launch visualization.launch.py` |
| 3 | `ros2 launch basic_launch dbw_joystick.launch.py` |
| 4 | `ros2 launch pacmod2 pacmod2.launch.xml` |
| 5a | **Experiment 1**: `ros2 launch gem_gnss_control stanley.launch.py obstacles_yaml:=''` |
| 5b | **Experiment 2**: `ros2 launch gem_gnss_control stanley.launch.py` |

---

## 4. Diagnostics

### Watch the Stanley log
```
x= xx.xx y= xx.xx yaw= xxx.x° | e_fa=+x.xx ψe=+xx.x° | δ=+xx.x° sw=+xxx.x° | v=x.xx v_ref=x.xx thr=x.xx
```
- `e_fa` (signed front-axle cross-track, m): should settle to <0.3 m on
  straights. Positive = vehicle right of path; negative = left.
- `ψe` (heading error, °): should settle to <5° on straights.
- `δ` is the front-wheel command (rad on the wire, deg in the log).
- `sw` is the steering-wheel command in degrees (after the
  `front2steer` conversion); this is the value being published as
  `angular_position` in radians on `/pacmod/steering_cmd`.

### Inspect topics live
```bash
ros2 topic hz /navsatfix                    # ~5–10 Hz
ros2 topic hz /insnavgeod                   # ~10 Hz
ros2 topic echo /insnavgeod --once          # check heading is sane
ros2 topic hz /pacmod/vehicle_speed_rpt     # confirms pacmod RX
ros2 topic echo /pacmod/enabled --once      # 'true' once armed
ros2 topic hz /pacmod/steering_cmd          # confirms Stanley TX
ros2 topic echo /pacmod/steering_cmd --once # see angular_position (rad)
```

### Record a run for offline analysis
```bash
mkdir -p ~/CS588/group9/gem_ws/logs && cd ~/CS588/group9/gem_ws/logs
ros2 bag record -o stanley_$(date +%Y%m%d_%H%M%S) \
    /navsatfix /insnavgeod \
    /pacmod/vehicle_speed_rpt /pacmod/enabled \
    /pacmod/steering_cmd /pacmod/accel_cmd /pacmod/brake_cmd
```

---

## 5. Swap or regenerate the reference trajectory

### 5.a Use a different existing CSV
Edit `config/e4_stanley.yaml` (or `e2_stanley.yaml`):
```yaml
waypoints_csv: 'lane2_refined.csv'   # → e.g. 'my_other_loop.csv'
```
Two CSV schemas are accepted automatically:
- 6 cols `x, y, yaw_rad, s, kappa, v_ref` — uses per-waypoint `v_ref`
  (capped by `desired_speed`)
- 3 cols `x, y, heading_deg` (compass; same as the existing
  `waypoints/track.csv`) — uses `desired_speed` everywhere

After editing yaml: `source install/setup.bash` and relaunch — no
rebuild needed.

### 5.b Regenerate from a fresh GNSS bag

The simulator workspace at `~/host/gem_simulation_ws/...` has the
canonical converter. From any machine with the simulator workspace
available:

```bash
cd ~/host/gem_simulation_ws/src/POLARIS_GEM_Simulator

# WGS84 → hardware ENU (origin matches e4_stanley.yaml)
python3 utils/gnss_to_reference.py \
    --gnss /path/to/your/gnss_data.csv \
    --origin 40.092857,-88.235992 \
    --output /tmp/new_path_hw.csv

# Smoothing spline + uniform 0.25 m resample + κ + v_ref
python3 utils/refine_trajectory.py \
    --input  /tmp/new_path_hw.csv \
    --output ~/CS588/group9/gem_ws/src/vehicle_drivers/gem_gnss_control/waypoints/new_path_refined.csv
```

`gnss_data.csv` must have a header with at least
`timestamp_ns, latitude_deg, longitude_deg`. To pull this from a
ros2 mcap bag, you can either replay the bag and `ros2 topic echo
/navsatfix --csv > gnss.csv`, or write a one-shot exporter — see the
lane2 dataset's `gnss/data.csv` for the exact column layout.

Then point the yaml at the new CSV (§5.a) and rebuild so the file
lands in the install share dir:
```bash
cd ~/CS588/group9/gem_ws
colcon build --symlink-install --packages-select gem_gnss_control
source install/setup.bash
```

### 5.c Important: ENU origin must match
If you regenerate waypoints with a different `--origin`, you **must**
update `origin_lat` / `origin_lon` in `config/e4_stanley.yaml` to the
same values. Otherwise the node will compute vehicle (x, y) in one
frame and search for the nearest waypoint in another — the vehicle
will think it's tens of metres from the path.

---

## 6. Tuning

All Stanley + speed-loop knobs live in `config/{e4,e2}_stanley.yaml`.
After editing yaml: `source install/setup.bash` and relaunch — no
rebuild.

| Param | Default | What it does |
|---|---|---|
| `stanley.k` | `0.45` | Cross-track gain. Higher = faster pull-in, but oscillation risk. |
| `stanley.k_soft` | `0.5` (m/s) | Numerical-stability term so `atan2(k·e, v+k_soft)` stays bounded near v=0. |
| `stanley.delta_max` | `0.61` rad (35°) | Front-wheel angle limit. |
| `stanley.delta_rate` | `1.5` rad/s | Steering slew limit (per command, not per second on the bus). |
| `desired_speed` | `2.0` m/s | Speed reference. Also caps any per-waypoint `v_ref`. |
| `max_acceleration` | `0.5` | Throttle saturation. |
| `pid.kp/ki/kd` | `0.6/0/0.1` | Speed-loop PID. |
| `wheelbase` | `2.57` (e4), `1.75` (e2) | Used for the front-axle projection. Get this right. |
| `offset` | `1.26` | GPS antenna ahead of rear axle (m). Used to convert antenna position → vehicle reference. |

Common issues:
- **Hunts / oscillates around the path** → lower `stanley.k` (try 0.3) or
  raise `stanley.k_soft` (try 1.0).
- **Cuts wide on turns** → raise `stanley.k` (try 0.6), or slow down via
  `desired_speed`.
- **Sluggish speed tracking** → raise `pid.kp` (try 1.0) or
  `max_acceleration`.

---

## 7. Switching back to pure_pursuit

The original `pure_pursuit` node is unchanged and still available:
```bash
ros2 launch gem_gnss_control pure_pursuit.launch.py
# or
ros2 run gem_gnss_control pure_pursuit
```
That uses `config/e4_pp.yaml` and `waypoints/track.csv` (which is a
**different** lane than `lane2_refined.csv` — track.csv is in the
north lane, lane2 is in the south lane).

---

## 8. Troubleshooting

### "No joystick connected" — Stanley exits immediately
Plug in the joystick; verify with the pygame check in §2.a.

### Vehicle won't move when LB+RB pressed
- `ros2 topic echo /pacmod/enabled --once` should be `true` within ~1 s
  of the arm signal — if not, pacmod isn't honouring the enable. Check
  the pacmod terminal for errors.
- Check the e-stop / kill switch is released.
- Confirm the gear is FORWARD: `ros2 topic echo /pacmod/shift_rpt --once`
  should report value 3.

### Vehicle steers in the wrong direction
- Heading is wrong. Verify `/insnavgeod`'s heading against the actual
  vehicle orientation (compass-style: 0 = N, 90 = E, 180 = S, 270 = W).
- If wrong, joystick-drive forward 5–10 s then check again. If still
  wrong, relaunch the GNSS launch.

### Vehicle pulls hard when first armed
- The closest waypoint to the vehicle is somewhere with a large heading
  offset relative to the vehicle's current pose.
- Reposition the vehicle physically near `lane2_refined.csv`'s first
  point: ENU (27.07, -11.17), facing roughly east (yaw ≈ -4°).
- Or trim the waypoints to start near where the vehicle actually is —
  use `--trim-start` / `--trim-end` on `gnss_to_reference.py` (see §5.b).

### "Pacmod disabled by joystick" loops repeatedly
- The joystick is firing LB-only intermittently. Check buttons aren't
  stuck: `jstest /dev/input/js0`.

### `e_fa` and `ψe` look correct but the vehicle still drifts
- `wheelbase` or `offset` in the yaml is wrong for this vehicle. Both
  affect where the controller thinks the front axle is.
- Confirm with a direct measurement on the vehicle.

---

## 9. Obstacle avoidance (sampling replanner)

`stanley_node` carries the same Werling-style sampling replanner the
simulator uses.  Behaviour:

1. On startup the node loads `obstacles_yaml` (default
   `config/obstacles_lane2.yaml`) — one or more static / moving disks
   in the same hardware ENU frame as `origin_lat/lon`.
2. Every `1/replan_hz` seconds (default 5 Hz), the replanner samples
   225 candidate trajectories in Frenet coordinates around the
   reference, scores them on collision + goal + centerline + speed +
   jerk + feasibility, and writes the best one back into
   `(x, y, yaw, kappa, v_ref)` in the active arc-length window.
3. Stanley tracks `self.plan` (the replanned path), which equals the
   reference outside the active window.  No interface change inside
   the controller.
4. Replan only fires when at least one obstacle is within
   `sensor_range` (default 30 m).  Outside that, Stanley tracks the
   reference unchanged.

### 9.a Pre-loaded obstacle (default)

`config/obstacles_lane2.yaml` ships with **one static disc**, derived
from the simulator's `obstacle_config.yaml` and converted into the
hardware ENU frame.  Physical placement:

| Quantity | Value |
|---|---|
| **GPS lat / lon (use a phone/GNSS to find this spot)** | **40.0927378° N, -88.2357399° W** |
| Hardware ENU `(x, y)` | (21.4959 m, -13.2378 m) |
| Disc radius | 0.75 m |
| Position along reference | s ≈ 101.50 m (waypoint 406 of 802) |
| Section of the loop | Southern straight, just past the right-hand U-turn |

Place a real obstacle (cone / barrel / cardboard box ≥ 1.5 m tall and
≥ 1.5 m wide so the lidar sees it cleanly) at that GPS point and the
replanner will detour around it.

### 9.b Adding / moving / removing obstacles

Edit `config/obstacles_lane2.yaml`:

```yaml
obstacles:
  - x: 21.4959      # hardware ENU (m)
    y: -13.2378
    radius: 0.75
  # add another disc:
  - x: -10.0
    y: -28.0
    radius: 1.0
```

Optional `vx, vy` fields (m/s in the hw ENU frame) enable the
constant-velocity prediction for moving obstacles — leave them out for
static obstacles.  After editing, `source install/setup.bash` is enough
(yaml is symlink-installed); no rebuild needed.

To compute the hw ENU position from a measured lat/lon, use the same
formula `stanley.py` uses:

```python
import pymap3d as pm
x, y, _ = pm.geodetic2enu(lat, lon, 0,
                          40.092857,    # origin_lat (e4_stanley.yaml)
                          -88.235992,   # origin_lon
                          0)
```

### 9.c Disabling the replanner (pure path tracking)

Edit `config/e4_stanley.yaml`:

```yaml
obstacles_yaml: ''     # ← empty string → replanner OFF
```

or just remove the `obstacles_lane2.yaml` file (the loader logs a
warning and continues with no obstacles).  Stanley falls back to
tracking `lane2_refined.csv` directly — same behaviour as before
obstacle support was added.

### 9.d Tuning knobs (yaml; no rebuild)

| Param | Default | What it does |
|---|---|---|
| `obstacles_yaml` | `'obstacles_lane2.yaml'` | YAML to load on startup; empty string disables the replanner |
| `replan_hz` | `5.0` | How often to call the replanner.  225 candidates × 41 timesteps × few obstacles is cheap, so 5–10 Hz is comfortable |
| `sensor_range` | `30.0` m | Replan only when an obstacle is within this distance from the rear axle |

Replanner internals — change in `gem_gnss_control/sampling_replanner.py`:
| Constant | Default | What it does |
|---|---|---|
| `KAPPA_MAX` | `0.27` 1/m | Curvature feasibility cap; matches the e4 (`tan(0.61)/2.57`).  Raise → more aggressive maneuvers, but the e4 will saturate steering |
| `V_MAX` | `2.5` m/s | Speed feasibility cap; raise to allow faster candidates |
| `DEFAULT_LANE_OFFSETS` | `[-2.4, -1.6, -0.8, -0.4, 0, 0.4, 0.8, 1.6, 2.4]` m | Lateral candidate grid; widen for bigger obstacles, tighten for narrow lanes |
| `HORIZON_S` | `4.0` s | Per-candidate planning horizon; raising plans further but slows the sampler |
| `SPHERE_RADIUS` | `0.90` m | Body inflation; raise for a wider safety margin |
| `HARD_CLEAR_M` | `0.20` m | Extra margin beyond `obstacle.r + sphere.r` |

### 9.e What the replanner log looks like

Every ~2 s `stanley_node` emits a line like:

```
replan(sampling): N=225 best#=98 cost=89.30 [col=0 goa=15.00 cen=0.32 jrk=0.005 fea=0.05]
target(d=-0.80, v=1.50, a=+0.00) max|κ|=0.092 v_min=1.18
d_obs_veh=12.4 s0=89.50 d0=+0.04 ok=Y obs:[@(+21.5,-13.2) r=0.75 v=(+0.00,+0.00)]
```

Read it as: `target(d=-0.80, ...)` means the chosen candidate offsets
the path 0.80 m to the right of the centerline (negative `d` =
right-hand offset in the path frame).  `ok=Y` means the chosen
candidate is collision-free.  `ok=N` is logged at warn level — the
plan still goes out, but the operator should be ready to disarm.

---

## Quick reference

| Need | Command |
|---|---|
| Build | `colcon build --symlink-install --packages-select gem_gnss_control` |
| Source | `source install/setup.bash` |
| **Experiment 1 — no obstacle** | `bash ~/CS588/group9/gem_ws/src/utilities/run_stanley.sh no_obstacle` |
| **Experiment 2 — with obstacle** | `bash ~/CS588/group9/gem_ws/src/utilities/run_stanley.sh with_obstacle` |
| Sensors | `ros2 launch basic_launch sensor_init.launch.py` |
| GNSS+RViz | `ros2 launch basic_launch visualization.launch.py` |
| Joystick | `ros2 launch basic_launch dbw_joystick.launch.py` |
| PACMod | `ros2 launch pacmod2 pacmod2.launch.xml` |
| Stanley (with obstacle) | `ros2 launch gem_gnss_control stanley.launch.py` |
| Stanley (no obstacle) | `ros2 launch gem_gnss_control stanley.launch.py obstacles_yaml:=''` |
| Pure pursuit | `ros2 launch gem_gnss_control pure_pursuit.launch.py` |
| Switch vehicle | `export VEHICLE_NAME=e2` (or `e4`) before launching |
| Inspect topic | `ros2 topic echo <topic> --once` |
| Topic rate | `ros2 topic hz <topic>` |
| Record bag | `ros2 bag record -o <name> <topic1> <topic2> ...` |

---

## Appendix — How state flows through Stanley

```
NavSatFix.lat,lon ─┐
                   ├─→ get_gem_state()       ─→ rear-axle (x,y,yaw)
INSNavGeod.heading ┘    pymap3d.geodetic2enu        │
                        + offset correction         │
                                                    ▼
                                            front-axle = rear + L·(cos,sin)
                                                    │
                                                    ▼
                                            argmin |path - front| → idx
                                                    │
                                                    ▼
                                            e_fa, ψe at path[idx]
                                                    │
                                                    ▼
                                            δ = ψe + atan2(k·e_fa, v+k_soft)
                                            δ ← clamp + slew-limit
                                                    │
                                                    ▼
                                            front2steer(δ) → /pacmod/steering_cmd
```

Speed control is independent: PID on `(v_ref - v)` → throttle, brake
held at zero (engine drag is enough on a flat highbay).
