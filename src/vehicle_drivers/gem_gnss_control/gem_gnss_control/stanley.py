#!/usr/bin/env python3
"""GNSS-anchored Stanley path-tracker for the GEM e4 (ROS 2).

Mirrors `pure_pursuit.py` in this same package: same joystick safety,
same `/pacmod/...` topics, same `/navsatfix` + `/insnavgeod` state, same
`(x, y)` waypoint frame from `pymap3d.geodetic2enu` about a configurable
origin.  The launch file and config style are also the same.

Control law — Hoffmann's Stanley (DARPA Grand Challenge):

    delta = psi_e + atan2(k * e_fa, v + k_soft)

where:
    * `e_fa` is the FRONT-AXLE signed lateral offset to the nearest path
      point (positive when the vehicle is to the RIGHT of the path,
      requiring `delta > 0` = left turn);
    * `psi_e = wrap(plan_yaw - vehicle_yaw)`;
    * `k` and `k_soft` are tuning gains (defaults from the simulator
      Stanley tuning in `mpc_controller.py`).

Waypoint CSV format — autodetected:
    * 3 cols: `x, y, heading_deg`  (compass; same as `waypoints/track.csv`)
    * >= 6 cols: `x, y, yaw_rad, s, kappa, v_ref`  (refined.csv produced
      by `utils/refine_trajectory.py`).  When `v_ref` is present it is
      used as the per-waypoint speed reference (with a config cap).

The longitudinal control mirrors `pure_pursuit.py`: a PID on speed error
with a Butterworth-filtered measurement.  Brake is held at zero — engine
drag plus accel near zero is enough to coast down on a flat highbay.
"""

import csv
import math
import os
from typing import Tuple

import numpy as np
import pygame
import pymap3d as pm
import scipy.signal as signal
import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod
from pacmod2_msgs.msg import (
    GlobalCmd, PositionWithSpeed, SystemCmdFloat, SystemCmdInt, VehicleSpeedRpt,
)

from .sampling_replanner import SamplingReplanner


# Joystick safety enable — same import-time setup as pure_pursuit.py so the
# node refuses to start without a connected joystick.
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick connected")
joystick = pygame.joystick.Joystick(0)
joystick.init()


class PID:
    def __init__(self, kp, ki, kd, wg=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.wg = wg
        self.iterm = 0.0
        self.last_e = 0.0
        self.last_t = None

    def reset(self):
        self.iterm = 0.0
        self.last_e = 0.0
        self.last_t = None

    def get_control(self, t, e):
        if self.last_t is None:
            dt = 0.0
            de = 0.0
        else:
            dt = t - self.last_t
            de = (e - self.last_e) / dt if dt > 0.0 else 0.0
        self.iterm += e * dt
        if self.wg is not None:
            self.iterm = max(min(self.iterm, self.wg), -self.wg)
        self.last_e = e
        self.last_t = t
        return self.kp * e + self.ki * self.iterm + self.kd * de


class OnlineFilter:
    def __init__(self, cutoff, fs, order):
        nyq = 0.5 * fs
        self.b, self.a = signal.butter(order, cutoff / nyq, btype='low', analog=False)
        self.z = signal.lfilter_zi(self.b, self.a)

    def get_data(self, data):
        out, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return out[0]


class Stanley(Node):
    def __init__(self):
        super().__init__('stanley_node')

        # Vehicle + tracker parameters — defaults match e4_pp.yaml so the
        # same e2/e4 yamls drop in.
        self.declare_parameter('rate_hz', 50)
        self.declare_parameter('wheelbase', 2.57)            # e4 default; e2 = 1.75
        self.declare_parameter('offset', 1.26)               # GPS antenna ahead of rear axle (m)
        self.declare_parameter('origin_lat', 40.092857)
        self.declare_parameter('origin_lon', -88.235992)
        self.declare_parameter('desired_speed', 2.0)         # m/s — also caps per-waypoint v_ref
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('vehicle_name', '')

        # Stanley gains — taken from the simulator Stanley used in
        # mpc_controller.py (already tuned for the GEM e4 plant model).
        self.declare_parameter('stanley/k', 0.45)
        self.declare_parameter('stanley/k_soft', 0.5)
        self.declare_parameter('stanley/delta_max', 0.61)    # rad — front-wheel limit
        self.declare_parameter('stanley/delta_rate', 1.5)    # rad/s — slew limit on delta cmd

        self.declare_parameter('pid/kp', 0.6)
        self.declare_parameter('pid/ki', 0.0)
        self.declare_parameter('pid/kd', 0.1)
        self.declare_parameter('pid/wg', 10.0)

        self.declare_parameter('filter/cutoff', 1.2)
        self.declare_parameter('filter/fs', 30.0)
        self.declare_parameter('filter/order', 4)

        self.declare_parameter('waypoints_csv', 'lane2_refined.csv')

        # Obstacle avoidance — leave `obstacles_yaml` empty to disable
        # the replanner entirely (Stanley falls straight back to tracking
        # `waypoints_csv`).  When set, the YAML must list one or more
        # `{x, y, radius[, vx, vy]}` entries in the same ENU frame as
        # `origin_lat/lon`.  Replanning runs at `replan_hz` and only when
        # at least one obstacle is within `sensor_range`.
        self.declare_parameter('obstacles_yaml', 'obstacles_lane2.yaml')
        self.declare_parameter('replan_hz', 5.0)
        self.declare_parameter('sensor_range', 30.0)

        gp = lambda n: self.get_parameter(n).value
        self.rate_hz       = int(gp('rate_hz'))
        self.wheelbase     = float(gp('wheelbase'))
        self.offset        = float(gp('offset'))
        self.olat          = float(gp('origin_lat'))
        self.olon          = float(gp('origin_lon'))
        self.desired_speed = min(5.0, float(gp('desired_speed')))
        self.max_accel     = min(2.0, float(gp('max_acceleration')))
        self.k_stanley     = float(gp('stanley/k'))
        self.k_soft        = float(gp('stanley/k_soft'))
        self.delta_max     = float(gp('stanley/delta_max'))
        self.delta_rate    = float(gp('stanley/delta_rate'))

        vehicle_name = gp('vehicle_name')
        if not vehicle_name:
            self.get_logger().warn(
                "No vehicle_name parameter — defaulting to e4 (L=2.57, GPS offset=1.26).")
        else:
            self.get_logger().info(
                f"Vehicle: {vehicle_name}, L={self.wheelbase}, GPS offset={self.offset}")

        self.pid_speed = PID(
            kp=gp('pid/kp'), ki=gp('pid/ki'), kd=gp('pid/kd'), wg=gp('pid/wg'))
        self.speed_filter = OnlineFilter(
            cutoff=gp('filter/cutoff'), fs=gp('filter/fs'), order=int(gp('filter/order')))

        # Subscriptions — identical to pure_pursuit.py.
        self.create_subscription(NavSatFix, '/navsatfix', self.gnss_cb, 10)
        self.create_subscription(INSNavGeod, '/insnavgeod', self.ins_cb, 10)
        self.create_subscription(Bool, '/pacmod/enabled', self.enable_cb, 10)
        self.create_subscription(VehicleSpeedRpt, '/pacmod/vehicle_speed_rpt',
                                 self.speed_cb, 10)

        # Publishers — identical to pure_pursuit.py.
        self.global_pub = self.create_publisher(GlobalCmd, '/pacmod/global_cmd', 10)
        self.gear_pub   = self.create_publisher(SystemCmdInt, '/pacmod/shift_cmd', 10)
        self.brake_pub  = self.create_publisher(SystemCmdFloat, '/pacmod/brake_cmd', 10)
        self.accel_pub  = self.create_publisher(SystemCmdFloat, '/pacmod/accel_cmd', 10)
        self.turn_pub   = self.create_publisher(SystemCmdInt, '/pacmod/turn_cmd', 10)
        self.steer_pub  = self.create_publisher(PositionWithSpeed, '/pacmod/steering_cmd', 10)

        self.global_cmd = GlobalCmd(enable=False, clear_override=True)
        self.gear_cmd   = SystemCmdInt(command=2)            # NEUTRAL on init
        self.brake_cmd  = SystemCmdFloat(command=0.0)
        self.accel_cmd  = SystemCmdFloat(command=0.0)
        self.turn_cmd   = SystemCmdInt(command=1)            # no signal
        self.steer_cmd  = PositionWithSpeed(angular_position=0.0, angular_velocity_limit=4.0)

        self.lat = 0.0
        self.lon = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.pacmod_enable = False
        self.delta_prev = 0.0

        self.path = self._load_waypoints(gp('waypoints_csv'))
        self.has_v_ref = self.path.shape[1] >= 6
        self.get_logger().info(
            f"Loaded {len(self.path)} waypoints "
            f"(v_ref column: {'yes' if self.has_v_ref else 'no'}).")

        # ─── obstacle avoidance setup ──────────────────────────────────
        # Stanley always tracks self.plan; without obstacles it's just a
        # copy of self.path, so the no-obstacle behaviour is unchanged.
        self.plan = self.path.copy()
        self.s_cur = 0.0
        self.replanner = None
        self.obstacles = []
        self._last_replan_t = 0.0
        self._replan_period = 1.0 / max(0.1, float(gp('replan_hz')))
        self._sensor_range = float(gp('sensor_range'))

        obs_path = gp('obstacles_yaml')
        if obs_path and self.has_v_ref:
            self.obstacles = self._load_obstacles(obs_path)
            if self.obstacles:
                self.replanner = SamplingReplanner(
                    self.path, sensor_range=self._sensor_range)
                obs_log = ", ".join(
                    f"({o['x']:+.2f}, {o['y']:+.2f}) r={o['r']:.2f}"
                    for o in self.obstacles)
                self.get_logger().info(
                    f"Replanner ON — {len(self.obstacles)} obstacle(s): "
                    f"{obs_log}; replan @ {gp('replan_hz')} Hz, "
                    f"sensor_range = {self._sensor_range} m")
            else:
                self.get_logger().info(
                    f"Obstacles YAML '{obs_path}' had no entries — "
                    "replanner disabled.")
        elif obs_path and not self.has_v_ref:
            self.get_logger().warn(
                "Replanner needs a 6-column waypoints CSV (x,y,yaw,s,kappa,v_ref). "
                "Disabled — falling back to direct path tracking.")
        else:
            self.get_logger().info(
                "obstacles_yaml param empty — replanner disabled.")

        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

    # ─── waypoint loader ─────────────────────────────────────────────────
    def _load_waypoints(self, csv_arg: str) -> np.ndarray:
        """Resolve CSV path; load 3-col or >=6-col schema; return float array.

        3-col `(x, y, heading_deg)` is converted to `(x, y, yaw_rad)`.  In
        that case `has_v_ref` is False and `desired_speed` is used as the
        speed reference.
        """
        path = csv_arg
        if not os.path.isabs(path):
            from ament_index_python.packages import get_package_share_directory
            try:
                share = get_package_share_directory('gem_gnss_control')
                cand = os.path.join(share, 'waypoints', path)
                if os.path.exists(cand):
                    path = cand
            except Exception:
                pass
            if not os.path.isabs(path) or not os.path.exists(path):
                # Source-tree fallback (works under colcon --symlink-install).
                here = os.path.dirname(os.path.abspath(__file__))
                path = os.path.join(here, '..', 'waypoints', csv_arg)

        with open(path, 'r') as f:
            rows = list(csv.reader(f))
        # Skip header if the first cell is non-numeric.
        try:
            float(rows[0][0])
            data = rows
        except ValueError:
            data = rows[1:]
        ncol = min(len(r) for r in data)
        arr = np.array([[float(c) for c in r[:ncol]] for r in data])
        if ncol == 3:
            # heading_deg column → yaw_rad in ENU (compass: 0=N, 90=E)
            heading = arr[:, 2]
            yaw = np.where(
                heading < 270.0,
                np.deg2rad(90.0 - heading),
                np.deg2rad(450.0 - heading),
            )
            arr = np.column_stack([arr[:, 0], arr[:, 1], yaw])
        return arr

    def _load_obstacles(self, yaml_arg: str) -> list:
        """Resolve YAML path the same way `_load_waypoints` does, then
        parse `obstacles: [...]` into the dict format the replanner
        expects: `{x, y, r, vx, vy}` (vx/vy default to 0)."""
        path = yaml_arg
        if not os.path.isabs(path):
            from ament_index_python.packages import get_package_share_directory
            try:
                share = get_package_share_directory('gem_gnss_control')
                cand = os.path.join(share, 'config', path)
                if os.path.exists(cand):
                    path = cand
            except Exception:
                pass
            if not os.path.isabs(path) or not os.path.exists(path):
                here = os.path.dirname(os.path.abspath(__file__))
                path = os.path.join(here, '..', 'config', yaml_arg)

        if not os.path.exists(path):
            self.get_logger().warn(f"Obstacles YAML not found: {path}")
            return []

        with open(path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        raw = cfg.get('obstacles', [])
        out = []
        for entry in raw:
            try:
                out.append({
                    'x':  float(entry['x']),
                    'y':  float(entry['y']),
                    'r':  float(entry.get('radius', entry.get('r', 0.5))),
                    'vx': float(entry.get('vx', 0.0)),
                    'vy': float(entry.get('vy', 0.0)),
                })
            except (KeyError, TypeError, ValueError) as e:
                self.get_logger().warn(f"Skipping bad obstacle entry {entry}: {e}")
        return out

    # ─── callbacks ───────────────────────────────────────────────────────
    def gnss_cb(self, msg: NavSatFix):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def ins_cb(self, msg: INSNavGeod):
        self.heading = msg.heading

    def speed_cb(self, msg: VehicleSpeedRpt):
        self.speed = float(self.speed_filter.get_data(msg.vehicle_speed))

    def enable_cb(self, msg: Bool):
        self.pacmod_enable = bool(msg.data)

    # ─── helpers (mirror pure_pursuit.py) ───────────────────────────────
    @staticmethod
    def _wrap(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def heading_to_yaw(self, heading_deg: float) -> float:
        if heading_deg < 270.0:
            return math.radians(90.0 - heading_deg)
        return math.radians(450.0 - heading_deg)

    def wps_to_local_xy(self, lon: float, lat: float) -> Tuple[float, float]:
        x, y, _ = pm.geodetic2enu(lat, lon, 0, self.olat, self.olon, 0)
        return x, y

    def front2steer(self, f_angle_deg: float) -> float:
        """Front-wheel angle (deg) -> steering-wheel angle (deg).

        Same coefficients as `pure_pursuit.py`: empirical second-order fit
        of the e4 steering kinematics.
        """
        f_angle = max(min(f_angle_deg, 35.0), -35.0)
        a = abs(f_angle)
        steer = -0.1084 * a * a + 21.775 * a
        return round(steer if f_angle >= 0 else -steer, 2)

    def check_joystick_enable(self) -> int:
        pygame.event.pump()
        try:
            lb = joystick.get_button(6)
            rb = joystick.get_button(7)
        except pygame.error:
            self.get_logger().warn("Joystick read failed")
            return 2
        if lb and rb:
            return 1
        if lb and not rb:
            return 0
        return 2

    def get_gem_state(self) -> Tuple[float, float, float]:
        local_x, local_y = self.wps_to_local_xy(self.lon, self.lat)
        yaw = self.heading_to_yaw(self.heading)
        # GPS antenna sits `offset` ahead of the rear axle; subtract that
        # so (x, y) refers to the rear-axle position used as the body frame.
        x = local_x - self.offset * math.cos(yaw)
        y = local_y - self.offset * math.sin(yaw)
        return x, y, yaw

    # ─── main control loop ─────────────────────────────────────────────
    def control_loop(self):
        joy = self.check_joystick_enable()

        if joy == 1 and not self.pacmod_enable:
            # Joystick arming.
            self.global_cmd.enable = True
            self.global_cmd.clear_override = True
            self.global_pub.publish(self.global_cmd)
            self.gear_cmd.command = 3
            self.gear_pub.publish(self.gear_cmd)
            self.brake_cmd.command = 0.0
            self.brake_pub.publish(self.brake_cmd)
            self.accel_cmd.command = 0.0
            self.accel_pub.publish(self.accel_cmd)
            self.turn_cmd.command = 3
            self.turn_pub.publish(self.turn_cmd)
            self.get_logger().warn('Pacmod arming: enable + forward gear')
            return

        if joy == 0 and self.pacmod_enable:
            self.global_cmd.enable = False
            self.global_pub.publish(self.global_cmd)
            self.turn_cmd.command = 1
            self.turn_pub.publish(self.turn_cmd)
            self.pid_speed.reset()
            self.delta_prev = 0.0
            self.get_logger().warn('Pacmod disabled by joystick')
            return

        if joy == 0 or not self.pacmod_enable:
            return

        # ─── Stanley step ─────────────────────────────────────────────
        rear_x, rear_y, yaw = self.get_gem_state()

        # ─── Replanner tick (rate-limited) ────────────────────────────
        # Modifies self.plan in the active window when at least one
        # obstacle is within sensor_range of the rear axle; otherwise
        # the plan stays equal to the reference path.
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self.replanner is not None
                and now - self._last_replan_t >= self._replan_period):
            try:
                result = self.replanner.replan(
                    rear_x, rear_y, yaw, self.speed,
                    self.obstacles,
                    seed_s=self.s_cur if self.s_cur > 1e-6 else None,
                )
                self.plan = result.plan
                self._last_replan_t = now
                if result.detour_ok:
                    self.get_logger().info(result.log,
                                            throttle_duration_sec=2.0)
                else:
                    self.get_logger().warn(result.log,
                                            throttle_duration_sec=1.0)
            except Exception as e:
                self.get_logger().warn(
                    f"Replanner failed: {e} — falling back to last good plan",
                    throttle_duration_sec=2.0)

        # Hoffmann references the FRONT axle, not the rear.
        fa_x = rear_x + self.wheelbase * math.cos(yaw)
        fa_y = rear_y + self.wheelbase * math.sin(yaw)

        # Nearest reference point to the front axle.  Stanley tracks
        # self.plan (= self.path when there's no active detour, or the
        # replanner's modified path inside the active window).
        d2 = (self.plan[:, 0] - fa_x) ** 2 + (self.plan[:, 1] - fa_y) ** 2
        idx = int(np.argmin(d2))
        if self.has_v_ref:
            self.s_cur = float(self.plan[idx, 3])

        px       = float(self.plan[idx, 0])
        py       = float(self.plan[idx, 1])
        plan_yaw = float(self.plan[idx, 2])

        # Cross-track error in the path frame.
        # path left-normal: n = (-sin θ, cos θ); e_left = (front - p) · n
        # e_fa = -e_left  →  positive iff front axle is RIGHT of path,
        # which calls for δ > 0 (left turn) via the atan term.
        rx = fa_x - px
        ry = fa_y - py
        e_left = -math.sin(plan_yaw) * rx + math.cos(plan_yaw) * ry
        e_fa = -e_left

        psi_e = self._wrap(plan_yaw - yaw)

        v = self.speed
        delta = psi_e + math.atan2(self.k_stanley * e_fa, v + self.k_soft)
        delta = max(-self.delta_max, min(self.delta_max, delta))

        max_step = self.delta_rate / self.rate_hz
        delta = max(self.delta_prev - max_step,
                    min(self.delta_prev + max_step, delta))
        self.delta_prev = delta

        # δ (rad) → front-wheel angle (deg) → steering-wheel angle (deg) → rad.
        sw_deg = self.front2steer(math.degrees(delta))
        self.steer_cmd.angular_position = math.radians(sw_deg)
        self.steer_pub.publish(self.steer_cmd)

        # Per-waypoint speed reference if available, capped by config.
        v_ref = float(self.plan[idx, 5]) if self.has_v_ref else self.desired_speed
        v_ref = max(0.0, min(self.desired_speed, v_ref))

        now = self.get_clock().now().nanoseconds * 1e-9
        speed_err = v_ref - v
        if abs(speed_err) < 0.05:
            speed_err = 0.0
        throttle = self.pid_speed.get_control(now, speed_err)
        throttle = max(0.0, min(throttle, self.max_accel))
        self.accel_cmd.command = throttle
        self.brake_cmd.command = 0.0
        self.accel_pub.publish(self.accel_cmd)
        self.brake_pub.publish(self.brake_cmd)
        self.global_cmd.enable = True
        self.global_pub.publish(self.global_cmd)

        # Throttled diagnostic — once every 0.5 s.
        self.get_logger().info(
            f"x={fa_x:6.2f} y={fa_y:6.2f} yaw={math.degrees(yaw):6.1f}° | "
            f"e_fa={e_fa:+.2f} ψe={math.degrees(psi_e):+5.1f}° | "
            f"δ={math.degrees(delta):+5.1f}° sw={sw_deg:+6.1f}° | "
            f"v={v:.2f} v_ref={v_ref:.2f} thr={throttle:.2f}",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = Stanley()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
