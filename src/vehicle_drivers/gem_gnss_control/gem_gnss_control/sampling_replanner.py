"""Werling-style sampling-based local replanner — hardware port.

Verbatim copy of the simulator's `utils/sampling_replanner.py` with
two constants retuned for the real GEM e4 (wheelbase 2.57 m vs the
simulator's 1.75 m):

  * KAPPA_MAX = 0.27 1/m  (tan(0.61 rad) / 2.57)
  * V_MAX     = 2.5 m/s   (matches the desired_speed cap in
                            e4_stanley.yaml + a small headroom)

Everything else — the Frenet sampler, cost terms, body-sphere geometry,
sensor range, prediction horizon — is unchanged, so behaviour matches
the simulator one-to-one.

The reference is the predefined CSV trajectory; `d = 0` means "follow
the predefined CSV exactly".  Obstacles are known disks
`(x, y, r, vx, vy)` from a YAML config (or, later, perception).  The
output is `(x, y, yaw, s, kappa, v_ref)` — same schema as the input
reference — so Stanley sees no API change.

Reference:
  Werling et al., "Optimal Trajectory Generation for Dynamic Street
  Scenarios in a Frenet Frame", ICRA 2010.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np


# ─── candidate-set tunables (sized for the GEM e4 in the highbay) ──────
HORIZON_S          = 4.0       # planning horizon per candidate (s)
NUM_SAMPLES        = 41        # ~0.1 s spacing across HORIZON_S

# Lateral offsets: d = 0 follows the CSV.  Range chosen so the body
# (∼2.5 m wide with 3-sphere model) can clear a 0.75 m obstacle plus a
# 0.20 m hard margin (1.85 m bubble) on either side of the centerline.
DEFAULT_LANE_OFFSETS = np.array(
    [-2.4, -1.6, -0.8, -0.4, 0.0, 0.4, 0.8, 1.6, 2.4], dtype=float)

# Speed deltas relative to the CSV's v_ref at the projected ego s.
# Smaller magnitudes than the highway scenario (V_MAX = 3 m/s here).
TARGET_SPEED_DELTAS = np.array(
    [-1.5, -1.0, -0.5, 0.0, 0.5], dtype=float)

# Terminal accelerations (m/s²) — sized for low-speed manoeuvring; never
# command the GEM to brake harder than a comfort-stop deceleration.
TARGET_ACCELS = np.array(
    [-2.0, -1.0, -0.5, 0.0, 0.5], dtype=float)

# Vehicle / dynamic limits used by feasibility cost.
# Hardware port: V_MAX and KAPPA_MAX retuned for the GEM e4
# (wheelbase 2.57 m, max steering 0.61 rad).
V_MAX            = 2.5       # m/s  (config caps desired_speed at 2.0; +0.5 headroom)
KAPPA_MAX        = 0.27      # 1/m  (= tan(0.61 rad) / 2.57)
A_LAT_LIMIT      = 1.8       # m/s² lateral
A_TOT_LIMIT      = 6.0       # m/s² total

# Body decomposition (must match mpc_controller.SPHERE_*).
SPHERE_OFFSETS = np.array([
    [+1.0, 0.0],   # front
    [ 0.0, 0.0],   # centre
    [-1.0, 0.0],   # rear
], dtype=float)
SPHERE_RADIUS    = 0.90
HARD_CLEAR_M     = 0.20

# Constant-velocity prediction cap on the obstacle.
PREDICT_HORIZON_S = 10.0

# Window of self.ref the replanner WRITES into.
REPLAN_HORIZON_M = 30.0
SENSOR_RANGE     = 30.0


# ─── cost weights ──────────────────────────────────────────────────────
@dataclass
class CostConfig:
    """Weights for the 6-term cost.

    The previous iteration omitted `goal_cost` on the (mistaken) belief
    that the CSV-tracking terms imply progress.  In practice they don't:
    a "stop in place" candidate has zero centerline cost and only a
    modest speed-tracking penalty (∼v_ref²), and beat dodge candidates
    that pay a lateral-offset penalty.  Adding the MP3-style goal_cost
    (penalises (s_local − s_end)) makes stalling expensive enough that
    a dodge is preferred whenever it's collision-free.
    """
    w_collision:   float = 1.0e6     # binary collision penalty — dominates
    w_goal:        float = 5.0       # remaining arc-length to local goal
    w_centerline:  float = 1.0       # mean d² + 0.5·target_offset²
    w_speed:       float = 4.0       # follow CSV v_ref schedule
    w_jerk:        float = 1.0e-2
    w_feasibility: float = 5.0


# ─── trajectory container ──────────────────────────────────────────────
@dataclass
class TrajectorySample:
    times:         np.ndarray
    s:             np.ndarray
    d:             np.ndarray
    x:             np.ndarray
    y:             np.ndarray
    yaw:           np.ndarray
    speed:         np.ndarray
    accel:         np.ndarray
    curvature:     np.ndarray
    s_jerk:        np.ndarray
    d_jerk:        np.ndarray
    target_offset: float
    target_speed:  float
    target_accel:  float


# ─── polynomials ───────────────────────────────────────────────────────
@dataclass
class QuinticPolynomial:
    coeffs: np.ndarray

    @staticmethod
    def fit(start, end, T):
        p0, v0, a0_s = start
        p1, v1, a1_e = end
        T = float(T)
        T2 = T * T;  T3 = T2 * T;  T4 = T3 * T;  T5 = T4 * T
        a0 = p0
        a1 = v0
        a2 = a0_s / 2.0
        M = np.array([
            [   T3,    T4,     T5],
            [3 * T2, 4 * T3, 5 * T4],
            [6 * T,  12 * T2, 20 * T3],
        ], dtype=float)
        r = np.array([
            p1   - a0 - a1 * T - a2 * T2,
            v1   - a1 - 2.0 * a2 * T,
            a1_e - 2.0 * a2,
        ], dtype=float)
        a3, a4, a5 = np.linalg.solve(M, r)
        return QuinticPolynomial(np.array([a0, a1, a2, a3, a4, a5], float))

    def evaluate(self, t, order=0):
        t = np.asarray(t, dtype=float)
        a0, a1, a2, a3, a4, a5 = self.coeffs
        if order == 0:
            return a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
        if order == 1:
            return a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
        if order == 2:
            return 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3
        if order == 3:
            return 6*a3 + 24*a4*t + 60*a5*t**2
        raise ValueError(f"order must be in {{0,1,2,3}}, got {order}")


@dataclass
class QuarticPolynomial:
    coeffs: np.ndarray

    @staticmethod
    def fit(start, end_speed, end_accel, T):
        p0, v0, a0_s = start
        T = float(T)
        T2 = T * T;  T3 = T2 * T
        a0 = p0
        a1 = v0
        a2 = a0_s / 2.0
        M = np.array([
            [3 * T2, 4 * T3],
            [6 * T,  12 * T2],
        ], dtype=float)
        r = np.array([
            float(end_speed) - a1 - 2.0 * a2 * T,
            float(end_accel) - 2.0 * a2,
        ], dtype=float)
        a3, a4 = np.linalg.solve(M, r)
        return QuarticPolynomial(np.array([a0, a1, a2, a3, a4], float))

    def evaluate(self, t, order=0):
        t = np.asarray(t, dtype=float)
        a0, a1, a2, a3, a4 = self.coeffs
        if order == 0:
            return a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4
        if order == 1:
            return a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3
        if order == 2:
            return 2*a2 + 6*a3*t + 12*a4*t**2
        if order == 3:
            return 6*a3 + 24*a4*t + np.zeros_like(t)
        raise ValueError(f"order must be in {{0,1,2,3}}, got {order}")


# ─── Frenet utilities for the predefined CSV reference ─────────────────
class FrenetReference:
    """Wraps the (N, 6) refined CSV path: `(x, y, yaw, s, kappa, v_ref)`.

    The CSV is already arc-length-parameterised (s monotonic, ~0.25 m
    spacing in this workspace), so projection / lookup is plain
    waypoint interpolation.
    """

    def __init__(self, ref: np.ndarray):
        self.ref = ref
        self.x   = ref[:, 0]
        self.y   = ref[:, 1]
        self.yaw = ref[:, 2]
        self.s   = ref[:, 3]
        self.kap = ref[:, 4]
        self.v   = ref[:, 5]
        self.s_min = float(self.s[0])
        self.s_max = float(self.s[-1])
        # Pre-cached cos/sin of the reference heading for spline-free
        # heading interpolation (avoids the wrap discontinuity that
        # `np.interp` would produce on yaw directly).
        self._cyaw = np.cos(self.yaw)
        self._syaw = np.sin(self.yaw)

    # signed perpendicular: d > 0 left of the path direction.
    def project(self, qx: float, qy: float,
                seed_s: Optional[float] = None,
                back_m: float = 4.0,
                fwd_m:  float = 8.0) -> tuple:
        """Project (qx, qy) onto the polyline.  Returns `(s, d, idx)`.

        Restricts the candidate range to a `[seed_s − back_m,
        seed_s + fwd_m]` window when `seed_s` is given — same pattern
        the controller uses to keep self-intersecting paths from
        snapping onto the wrong branch.
        """
        if seed_s is not None:
            mask = (self.s >= seed_s - back_m) & (self.s <= seed_s + fwd_m)
            cand = np.where(mask)[0]
        else:
            cand = np.arange(len(self.s))
        if len(cand) == 0:
            cand = np.arange(len(self.s))
        d2 = (self.x[cand] - qx) ** 2 + (self.y[cand] - qy) ** 2
        i = int(cand[int(np.argmin(d2))])
        # Refine onto adjacent segment (i-1, i) ∪ (i, i+1).
        best_s, best_signed, best_dist = float(self.s[i]), 0.0, float("inf")
        for k in (max(i - 1, 0), i):
            kn = k + 1
            if kn >= len(self.s):
                continue
            tx = self.x[kn] - self.x[k]
            ty = self.y[kn] - self.y[k]
            seg2 = tx * tx + ty * ty
            if seg2 < 1e-9:
                continue
            t = ((qx - self.x[k]) * tx + (qy - self.y[k]) * ty) / seg2
            t = float(np.clip(t, 0.0, 1.0))
            px = self.x[k] + t * tx
            py = self.y[k] + t * ty
            ds = math.hypot(px - qx, py - qy)
            if ds < best_dist:
                best_dist = ds
                # left-normal of the path direction (tx, ty).
                nx, ny = -ty, tx
                norm = math.hypot(nx, ny)
                signed = ((qx - px) * nx + (qy - py) * ny) / max(norm, 1e-9)
                best_s = float(self.s[k] + t * (self.s[kn] - self.s[k]))
                best_signed = float(signed)
        return best_s, best_signed, i

    def eval(self, s_arr: np.ndarray, d_arr: np.ndarray) -> tuple:
        """Frenet (s, d) → Cartesian xy and reference heading at s.

        Returns:
            xy:  (M, 2) array of Cartesian points.
            psi: (M,)   reference heading at each s.
        """
        s = np.clip(np.asarray(s_arr, dtype=float),
                    self.s_min, self.s_max)
        d = np.asarray(d_arr, dtype=float)
        x_c = np.interp(s, self.s, self.x)
        y_c = np.interp(s, self.s, self.y)
        c = np.interp(s, self.s, self._cyaw)
        si = np.interp(s, self.s, self._syaw)
        psi = np.arctan2(si, c)
        xy = np.empty((len(s), 2), dtype=float)
        xy[:, 0] = x_c - d * np.sin(psi)
        xy[:, 1] = y_c + d * np.cos(psi)
        return xy, psi

    def v_ref_at(self, s_arr: np.ndarray) -> np.ndarray:
        return np.interp(np.clip(s_arr, self.s_min, self.s_max),
                          self.s, self.v)


# ─── candidate generator ───────────────────────────────────────────────
def sample_trajectories(
    s0: float, d0: float, s_dot0: float, d_dot0: float,
    v_ref_now: float,
    frenet_ref: FrenetReference,
    horizon_s: float = HORIZON_S,
    num_samples: int = NUM_SAMPLES,
    target_offsets: Sequence[float] = DEFAULT_LANE_OFFSETS,
    target_speed_deltas: Sequence[float] = TARGET_SPEED_DELTAS,
    target_accels: Sequence[float] = TARGET_ACCELS,
) -> List[TrajectorySample]:
    """Werling-style Frenet candidate set.

    Cartesian product of (lane offset, target speed, target accel) →
    9 × 5 × 5 = 225 trajectories by default.  Each candidate fits an
    independent quintic for d(t) and quartic for s(t) to the supplied
    initial Frenet state and the trial terminal condition, then
    converts back to Cartesian via `frenet_ref.eval`.

    Args:
      s0, d0, s_dot0, d_dot0:   ego initial Frenet state.
      v_ref_now:                CSV-prescribed speed at `s0` — used as
                                the centre of the speed-target grid.
      frenet_ref:               wrapper over the CSV reference path.
    """
    times = np.linspace(0.0, horizon_s, num_samples)
    dt_step = float(times[1] - times[0]) if num_samples > 1 else 0.1

    # Speed grid: clamp ≥ 0 (no reversing) and ≤ V_MAX.
    speeds = np.maximum(0.0, v_ref_now + np.asarray(target_speed_deltas))
    speeds = np.minimum(speeds, V_MAX)
    accels = np.asarray(target_accels, dtype=float)
    offsets = np.asarray(target_offsets, dtype=float)

    samples: List[TrajectorySample] = []
    for d_target in offsets:
        d_poly = QuinticPolynomial.fit(
            (d0, d_dot0, 0.0), (float(d_target), 0.0, 0.0), horizon_s)
        d_vals    = d_poly.evaluate(times, 0)
        d_dot_vals = d_poly.evaluate(times, 1)
        d_jerk    = d_poly.evaluate(times, 3)

        for v_target in speeds:
            for a_target in accels:
                s_poly = QuarticPolynomial.fit(
                    (s0, s_dot0, 0.0),
                    float(v_target), float(a_target), horizon_s)
                s_vals     = s_poly.evaluate(times, 0)
                s_dot_vals = np.maximum(s_poly.evaluate(times, 1), 0.0)
                s_jerk     = s_poly.evaluate(times, 3)

                xy, psi = frenet_ref.eval(s_vals, d_vals)
                # Trajectory yaw = ref heading + atan2(d_dot, s_dot).
                yaw = np.unwrap(
                    psi + np.arctan2(d_dot_vals,
                                      np.maximum(s_dot_vals, 1e-3)))
                dx = np.gradient(xy[:, 0], dt_step)
                dy = np.gradient(xy[:, 1], dt_step)
                speed = np.hypot(dx, dy)
                accel = np.gradient(speed, dt_step)
                yaw_rate = np.gradient(yaw, dt_step)
                curvature = yaw_rate / np.maximum(speed, 1e-3)

                samples.append(TrajectorySample(
                    times=times,
                    s=s_vals, d=d_vals,
                    x=xy[:, 0], y=xy[:, 1], yaw=yaw,
                    speed=speed, accel=accel, curvature=curvature,
                    s_jerk=s_jerk, d_jerk=d_jerk,
                    target_offset=float(d_target),
                    target_speed=float(v_target),
                    target_accel=float(a_target),
                ))
    return samples


# ─── cost terms ────────────────────────────────────────────────────────
def collision_cost(traj: TrajectorySample,
                   obstacles: list,
                   sphere_offsets: np.ndarray = SPHERE_OFFSETS,
                   sphere_radius: float = SPHERE_RADIUS,
                   hard_clear: float = HARD_CLEAR_M,
                   predict_horizon: float = PREDICT_HORIZON_S) -> float:
    """Binary collision penalty (1.0 if any timestep overlaps the
    inflated obstacle disk, else 0.0).

    For each obstacle, the centre is propagated by constant-velocity
    extrapolation up to `predict_horizon`; for stationary obstacles
    (`vx == vy == 0`) this reduces to the static-disk check.
    """
    if not obstacles:
        return 0.0
    cy = np.cos(traj.yaw)
    sy = np.sin(traj.yaw)
    eta = np.minimum(traj.times, predict_horizon)
    for ox_b, oy_b in sphere_offsets:
        sx = traj.x + ox_b * cy - oy_b * sy
        sy_ = traj.y + ox_b * sy + oy_b * cy
        for ob in obstacles:
            ox = ob["x"] + ob.get("vx", 0.0) * eta
            oy = ob["y"] + ob.get("vy", 0.0) * eta
            d  = np.hypot(sx - ox, sy_ - oy)
            min_clear = ob["r"] + sphere_radius + hard_clear
            if np.any(d < min_clear):
                return 1.0
    return 0.0


def centerline_cost(traj: TrajectorySample) -> float:
    """Mean d² along the trajectory plus a 0.5-weighted penalty on the
    terminal lateral target.  The CSV is the centerline; this is the
    "stay on the predefined trajectory" term."""
    return float(np.mean(traj.d ** 2) + 0.5 * traj.target_offset ** 2)


def goal_cost(traj: TrajectorySample, frenet_ref: FrenetReference,
              local_horizon_m: float = 35.0) -> float:
    """Penalise short forward progress.

    Local goal = min(path_end_s, s_start + local_horizon_m); cost is the
    non-negative shortfall (s_local − s_end).  A trajectory that stalls
    (target_speed ≈ 0) collects ~`local_horizon_m` worth of penalty —
    enough to outweigh the centerline cost a moderate dodge incurs, so
    dodging beats stopping whenever both are feasible."""
    s_start = float(traj.s[0])
    s_end   = float(traj.s[-1])
    s_local = min(frenet_ref.s_max, s_start + local_horizon_m)
    return float(max(s_local - s_end, 0.0))


def speed_cost(traj: TrajectorySample, frenet_ref: FrenetReference) -> float:
    """Mean squared deviation of trajectory speed from the CSV's
    v_ref(s) schedule, sampled at each timestep's projected s."""
    v_des = frenet_ref.v_ref_at(traj.s)
    return float(np.mean((traj.speed - v_des) ** 2))


def jerk_cost(traj: TrajectorySample) -> float:
    return float(np.mean(traj.s_jerk ** 2 + traj.d_jerk ** 2))


def feasibility_cost(traj: TrajectorySample) -> float:
    """Soft quadratic penalties on speed / total accel / curvature
    limit violations.  Total accel uses √(a_long² + a_lat²) so the
    candidate gets pulled away from cornering hot."""
    spd_e = np.maximum(traj.speed - V_MAX, 0.0)
    a_lat = traj.speed ** 2 * traj.curvature
    a_tot = np.hypot(traj.accel, a_lat)
    acc_e = np.maximum(np.abs(a_tot) - A_TOT_LIMIT, 0.0)
    cur_e = np.maximum(np.abs(traj.curvature) - KAPPA_MAX, 0.0)
    return float(np.mean(spd_e ** 2)
                 + np.mean(acc_e ** 2)
                 + np.mean(cur_e ** 2))


def evaluate(trajectories: List[TrajectorySample],
             obstacles: list,
             frenet_ref: FrenetReference,
             cfg: Optional[CostConfig] = None) -> np.ndarray:
    """Weighted sum of the five terms for every candidate.

    Returns a `(N,)` array of scalar costs; the planner picks
    `argmin(costs)`.
    """
    if cfg is None:
        cfg = CostConfig()
    n = len(trajectories)
    costs = np.zeros(n, dtype=float)
    for i, t in enumerate(trajectories):
        costs[i] = (
            cfg.w_collision   * collision_cost(t, obstacles)
            + cfg.w_goal        * goal_cost(t, frenet_ref)
            + cfg.w_centerline  * centerline_cost(t)
            + cfg.w_speed       * speed_cost(t, frenet_ref)
            + cfg.w_jerk        * jerk_cost(t)
            + cfg.w_feasibility * feasibility_cost(t)
        )
    return costs


# ─── replanner orchestration ───────────────────────────────────────────
@dataclass
class ReplanResult:
    plan:       np.ndarray             # (N, 6) — same schema as ref
    win_idx:    np.ndarray             # indices of self.ref in horizon window
    best:       Optional[TrajectorySample]
    log:        str
    detour_ok:  bool                   # True iff best candidate is collision-free


def _smooth_kappa(xs: np.ndarray, ys: np.ndarray, baseline: int = 4) -> np.ndarray:
    """Finite-difference curvature with ±baseline-waypoint smoothing.

    Same shape and intent as the helper formerly in mpc_controller —
    used here to fill in the kappa column of the modified plan."""
    N = len(xs)
    b = baseline
    if N < 2 * b + 3:
        return np.zeros(N)
    dy = ys[2 * b:] - ys[:-2 * b]
    dx = xs[2 * b:] - xs[:-2 * b]
    yaw_c = np.unwrap(np.arctan2(dy, dx))
    seg = np.hypot(np.diff(xs), np.diff(ys))
    s_arc = np.concatenate([[0.0], np.cumsum(seg)])
    kappa = np.zeros(N)
    if len(yaw_c) >= 3:
        ds_inner = s_arc[b + 2:N - b] - s_arc[b:N - b - 2]
        kappa_inner = (yaw_c[2:] - yaw_c[:-2]) / np.maximum(ds_inner, 1e-6)
        kappa[b + 1:N - b - 1] = kappa_inner
    if N - b - 1 > b + 1:
        kappa[:b + 1] = kappa[b + 1]
        kappa[N - b - 1:] = kappa[N - b - 2]
    return kappa


class SamplingReplanner:
    """High-level orchestration: project ego → sample → score → write
    the best candidate back into the controller's plan grid.

    The plan grid retains the CSV's arc-length s column unchanged; only
    `(x, y, yaw, kappa, v_ref)` are rewritten in the active window.
    Stanley/MPC's `_project_pose_on_path` keys off s for windowing, and
    the trajectory geometry stays close enough to the nominal arc
    length that this is safe.
    """

    def __init__(self,
                 ref: np.ndarray,
                 cfg: Optional[CostConfig] = None,
                 horizon_m: float = REPLAN_HORIZON_M,
                 sensor_range: float = SENSOR_RANGE,
                 a_brake: float = 3.0,
                 stop_margin: float = HARD_CLEAR_M):
        self.ref = ref
        self.frenet = FrenetReference(ref)
        self.cfg = cfg or CostConfig()
        self.horizon_m = horizon_m
        self.sensor_range = sensor_range
        self.a_brake = a_brake
        self.stop_margin = stop_margin

    def replan(self, veh_x: float, veh_y: float, veh_yaw: float,
               veh_v: float,
               obstacles: list,
               seed_s: Optional[float] = None) -> ReplanResult:
        """Run one replan tick.  Caller passes vehicle pose, current
        obstacle list, and an optional seed s (the previous projection)
        to disambiguate self-intersecting reference paths."""

        # Sensor-range gate.
        visible = []
        d_obs_veh = float("inf")
        for ob in obstacles:
            d = math.hypot(ob["x"] - veh_x, ob["y"] - veh_y)
            if d <= self.sensor_range:
                visible.append(ob)
                d_obs_veh = min(d_obs_veh, d)

        # Frenet decomposition of the ego state.
        s0, d0, _ = self.frenet.project(veh_x, veh_y, seed_s)
        _, ref_yaw_arr = self.frenet.eval(np.array([s0]), np.array([0.0]))
        psi_ref = float(ref_yaw_arr[0])
        delta = math.atan2(math.sin(veh_yaw - psi_ref),
                           math.cos(veh_yaw - psi_ref))
        s_dot0 = veh_v * math.cos(delta)
        d_dot0 = veh_v * math.sin(delta)
        v_ref_now = float(self.frenet.v_ref_at(np.array([s0]))[0])

        # If no obstacle is in range, the d=0 / v=v_ref candidate wins
        # by construction (it is already collision-free, on the
        # centerline, and matches v_ref).  Skip the full sampler to
        # save the per-tick cost — the controller still gets a plan
        # that "starts at the vehicle".
        if not visible:
            best = self._build_centerline_candidate(s0, s_dot0, v_ref_now)
            plan, win_idx = self._write_plan(best, s0)
            return ReplanResult(
                plan=plan, win_idx=win_idx, best=best, detour_ok=True,
                log=("replan(sampling): no obstacle in range "
                     "(d_obs_veh=inf)  s0=%.2f  d0=%+.2f  "
                     "candidate=centerline" % (s0, d0)))

        samples = sample_trajectories(s0, d0, s_dot0, d_dot0,
                                      v_ref_now, self.frenet)
        costs = evaluate(samples, visible, self.frenet, self.cfg)
        i_best = int(np.argmin(costs))
        best = samples[i_best]

        # Detour OK iff the chosen candidate is itself collision-free.
        detour_ok = (collision_cost(best, visible) == 0.0)

        plan, win_idx = self._write_plan(best, s0)
        plan = self._apply_safety_speed_cap(plan, visible)

        log = self._format_log(samples, costs, i_best, best, visible,
                               d_obs_veh, s0, d0, detour_ok, self.frenet)
        return ReplanResult(plan=plan, win_idx=win_idx, best=best,
                            detour_ok=detour_ok, log=log)

    # ── helpers ────────────────────────────────────────────────────────
    def _build_centerline_candidate(
            self, s0: float, s_dot0: float,
            v_ref_now: float) -> TrajectorySample:
        times = np.linspace(0.0, HORIZON_S, NUM_SAMPLES)
        dt_step = float(times[1] - times[0])
        d_vals = np.zeros_like(times)
        d_dot_vals = np.zeros_like(times)
        d_jerk = np.zeros_like(times)
        s_poly = QuarticPolynomial.fit(
            (s0, s_dot0, 0.0), float(v_ref_now), 0.0, HORIZON_S)
        s_vals = s_poly.evaluate(times, 0)
        s_dot_vals = np.maximum(s_poly.evaluate(times, 1), 0.0)
        s_jerk = s_poly.evaluate(times, 3)
        xy, psi = self.frenet.eval(s_vals, d_vals)
        yaw = np.unwrap(psi)
        dx = np.gradient(xy[:, 0], dt_step)
        dy = np.gradient(xy[:, 1], dt_step)
        speed = np.hypot(dx, dy)
        accel = np.gradient(speed, dt_step)
        curvature = np.gradient(yaw, dt_step) / np.maximum(speed, 1e-3)
        return TrajectorySample(
            times=times, s=s_vals, d=d_vals,
            x=xy[:, 0], y=xy[:, 1], yaw=yaw,
            speed=speed, accel=accel, curvature=curvature,
            s_jerk=s_jerk, d_jerk=d_jerk,
            target_offset=0.0, target_speed=float(v_ref_now),
            target_accel=0.0,
        )

    def _write_plan(self, best: TrajectorySample, s0: float) -> tuple:
        """Resample the best Frenet candidate onto the arc-length grid
        of `self.ref` and write `(x, y, yaw, kappa, v_ref)` into the
        active replan window.  The s column is left untouched (it
        keeps the CSV's monotonic 0.25 m grid that the projection
        helpers rely on)."""
        plan = self.ref.copy()
        s_ref = self.ref[:, 3]

        # Build the window of grid stations the trajectory actually
        # covers — clamp the upper end to whichever is smaller of (the
        # trajectory's s_max, the CSV end, s_veh + horizon_m).
        s_traj_max = float(np.max(best.s))
        s_win_end = min(s_ref[-1],
                        s_traj_max,
                        s0 + self.horizon_m)
        win_idx = np.where((s_ref >= s0) & (s_ref <= s_win_end))[0]
        if len(win_idx) == 0:
            return plan, win_idx

        order = np.argsort(best.s)
        s_t = best.s[order]
        x_t = best.x[order]
        y_t = best.y[order]
        v_t = best.speed[order]

        # Strictly-monotone s_t for `np.interp` — duplicates can arise
        # if the candidate is moving very slowly; nudge them apart.
        for j in range(1, len(s_t)):
            if s_t[j] <= s_t[j - 1]:
                s_t[j] = s_t[j - 1] + 1e-6

        s_query = np.clip(s_ref[win_idx], s_t[0], s_t[-1])
        x_new = np.interp(s_query, s_t, x_t)
        y_new = np.interp(s_query, s_t, y_t)
        v_new = np.interp(s_query, s_t, v_t)

        plan[win_idx, 0] = x_new
        plan[win_idx, 1] = y_new
        # v_ref: cap to the candidate's own speed schedule but keep the
        # ramp-down-near-goal that the CSV already encodes.
        plan[win_idx, 5] = np.minimum(plan[win_idx, 5],
                                      np.maximum(v_new, 0.0))

        # Refresh yaw from the modified positions (smoothed central
        # diff over the full plan, then write back at win_idx so we
        # don't perturb yaw outside the window).
        N = len(plan)
        b = 4
        if N > 2 * b + 1:
            dy = plan[2 * b:, 1] - plan[:-2 * b, 1]
            dx = plan[2 * b:, 0] - plan[:-2 * b, 0]
            yaw_mid = np.arctan2(dy, dx)
            yaw_full = np.empty(N)
            yaw_full[b:N - b] = yaw_mid
            yaw_full[:b] = yaw_mid[0]
            yaw_full[N - b:] = yaw_mid[-1]
            plan[win_idx, 2] = yaw_full[win_idx]

        # Refresh kappa column (used downstream by v_ref capping etc).
        kappa_new = _smooth_kappa(plan[:, 0], plan[:, 1])
        plan[win_idx, 4] = kappa_new[win_idx]

        return plan, win_idx

    def _apply_safety_speed_cap(self, plan: np.ndarray,
                                obstacles: list) -> np.ndarray:
        """Distance-based braking: cap v_ref by √(2·a_brake·available)
        where `available = d_surface − stop_margin`.  Guarantees the
        plan slows down before the stop margin even when the cost
        term alone would have allowed faster passage.
        """
        if not obstacles:
            return plan
        cy = np.cos(plan[:, 2])
        sy = np.sin(plan[:, 2])
        d_surf = np.full(len(plan), np.inf)
        for ob in obstacles:
            local = np.full(len(plan), np.inf)
            for ox_b, oy_b in SPHERE_OFFSETS:
                sx = plan[:, 0] + ox_b * cy - oy_b * sy
                sy_ = plan[:, 1] + ox_b * sy + oy_b * cy
                d  = np.hypot(sx - ob["x"], sy_ - ob["y"])
                local = np.minimum(local, d)
            d_surf = np.minimum(d_surf, local - ob["r"] - SPHERE_RADIUS)
        avail = np.maximum(d_surf - self.stop_margin, 0.0)
        v_brake = np.sqrt(2.0 * self.a_brake * avail)
        plan[:, 5] = np.minimum(plan[:, 5], v_brake)
        return plan

    @staticmethod
    def _format_log(samples, costs, i_best, best, obstacles,
                    d_obs_veh, s0, d0, detour_ok,
                    frenet_ref: FrenetReference) -> str:
        # Cost breakdown for the chosen candidate (recomputed; cheap).
        c_col = collision_cost(best, obstacles)
        c_goa = goal_cost(best, frenet_ref)
        c_cen = centerline_cost(best)
        c_jrk = jerk_cost(best)
        c_fea = feasibility_cost(best)
        max_k = float(np.max(np.abs(best.curvature)))
        v_min = float(np.min(best.speed))
        obs_str = ", ".join(
            f"@({ob['x']:+.1f},{ob['y']:+.1f}) r={ob['r']:.2f}"
            f" v=({ob.get('vx', 0.0):+.2f},{ob.get('vy', 0.0):+.2f})"
            for ob in obstacles)
        return ("replan(sampling): N=%d  best#=%d  cost=%.2f "
                "[col=%.0f goa=%.2f cen=%.2f jrk=%.4f fea=%.2f]  "
                "target(d=%+.2f, v=%.2f, a=%+.2f)  "
                "max|κ|=%.3f  v_min=%.2f  d_obs_veh=%.1f  "
                "s0=%.2f d0=%+.2f  ok=%s  obs:[%s]"
                % (len(samples), i_best, costs[i_best],
                   c_col, c_goa, c_cen, c_jrk, c_fea,
                   best.target_offset, best.target_speed, best.target_accel,
                   max_k, v_min, d_obs_veh,
                   s0, d0, "Y" if detour_ok else "N",
                   obs_str))
