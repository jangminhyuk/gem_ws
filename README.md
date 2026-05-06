# install
```bash
mkdir -p ~/CS588/<your_group_name>
cd ~/CS588/<your_group_name>
git clone https://github.com/hungdche/gem_ws.git
cd gem_ws
```

# compile workspace
```bash
colcon build --symlink-install 
```

# launch sensor
```bash
source install/setup.bash
ros2 launch basic_launch sensor_init.launch.py
```

# launch corner cameras
IMPORTANT: Because of the e4 incident, the corner cameras are not compatible with this backup PC, so corner cameras will not work.  
```bash
source install/setup.bash
ros2 launch basic_launch corner_cameras.launch.py
```

# launch gnss
```bash
source install/setup.bash
ros2 launch basic_launch visualization.launch.py
```

# launch joystick control
```bash
source install/setup.bash
ros2 launch basic_launch dbw_joystick.launch.py
```

# launch path tracking controller
**IMPORTANT**: 
- Make sure that the heading in GNSS control is correct. Relaunch GNSS or restart the machine if you have to 
- Make sure to disable to joystick control

On one terminal:
```bash
source install/setup.bash
ros2 launch pacmod2 pacmod2.launch.xml
```

On another monitor:
```bash
source install/setup.bash
ros2 run gem_gnss_control pure_pursuit
```

# launch Stanley path tracking controller (alternative to pure_pursuit)
**IMPORTANT**: same prerequisites as pure_pursuit — heading must be correct,
joystick control must be disabled, and pacmod2 must be running.

```bash
source install/setup.bash
ros2 launch gem_gnss_control stanley.launch.py
```

The Stanley node uses the same `/navsatfix`, `/insnavgeod`,
`/pacmod/vehicle_speed_rpt`, `/pacmod/enabled` inputs and the same
`/pacmod/{global,shift,brake,accel,turn,steering}_cmd` outputs as
`pure_pursuit`, plus the same LB/RB joystick arm gate (LB+RB = arm,
LB alone = disarm).

Per-vehicle config: `config/e4_stanley.yaml` and `config/e2_stanley.yaml`,
selected by the `VEHICLE_NAME` env var (default `e4`).  The waypoints
file defaults to `waypoints/lane2_refined.csv` — a 200 m loop derived
from the lane2 GNSS bag in the hardware ENU frame
(origin lat=40.092857, lon=-88.235992).  Two CSV schemas are accepted:
`(x, y, heading_deg)` like the existing `track.csv`, or the 6-column
`(x, y, yaw, s, kappa, v_ref)` produced by the simulator's
`refine_trajectory.py` (in which case `v_ref` becomes the per-waypoint
speed reference, capped at `desired_speed`).

To regenerate `lane2_refined.csv` from a different ROS 2 bag, see
section 5 of `~/gem_simulation_ws/COMMANDS.md`.

