"""
main.py
=======
Main simulation: Franka Panda + Franka Hand + cable + MultiPriority Controller.

Operation sequence:
  1. GRASP: Robot moves gripper to cable connector on the floor and picks it up.
  2. TASK:  Robot executes arc trajectory in XY plane while maintaining cable tension.

Usage:
    python main.py                      # interactive mode (viewer)
    python main.py --headless 40.0      # headless + plots + CSV

Viewer keys:
    SPACE   — pause / resume
    R       — reset trajectory
    Q       — quit
"""

import os
import sys
import csv
import time
import datetime
from enum import Enum, auto

import math

import numpy as np
import mujoco
import mujoco.viewer

from multi_priority_controller import MultiPriorityController, get_ee_jacobian, pseudo_inverse

# ---------------------------------------------------------------------------
# Global settings
# ---------------------------------------------------------------------------

XML_PATH = "franka_cable.xml"

# Robot initial position: Panda "home"
QPOS_INIT_ROBOT = np.array([0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, -0.7853])

# Anchor
ANCHOR_POS = np.array([0.8, 0.0, 0.4])

# Cable parameters
CABLE_LENGTH = 0.50

# Trajectory parameters — same shapes as cable_grasp_orchestrator
TRAJ_SHAPE   = "circle"   # "circle" | "rectangle" | "triangle"
TRAJ_RADIUS  = 0.40       # m
TRAJ_Z_FIXED = 0.40       # m — fixed EE height during task
TRAJ_PERIOD  = 20.0       # s — one full cycle

# Controller parameters
TENSION_DESIRED = 10.0
POS_KP          = 150.0
POS_KD          = 15.0
NULL_DAMPING    = 5.0

# ---------------------------------------------------------------------------
# Grasp phase settings
# ---------------------------------------------------------------------------

CONNECTOR_POS_INIT           = np.array([0.5, 0.0, 0.025])
GRASP_APPROACH_HEIGHT_OFFSET = 0.18    # m above the connector
GRASP_HEIGHT_OFFSET          = 0.005   # m — contact height
REACH_POS_TOL                = 0.015   # m
LOWER_POS_TOL                = 0.012   # m
LIFT_HEIGHT                  = 0.40    # m (= TRAJ_Z_FIXED)
LIFT_POS_TOL                 = 0.020   # m
CLOSE_GRIPPER_DURATION       = 1.0     # s — time to close fingers
WELD_ACTIVATE_DELAY          = 0.3     # s — time for weld to settle
GRIPPER_OPEN_CTRL            = 255.0   # ctrl → fingers open
GRIPPER_CLOSE_CTRL           = 0.0     # ctrl → fingers closed
GRASP_KP                     = 200.0
GRASP_KD                     = 20.0
GRASP_NULL_DAMP              = 3.0

# Logging
LOG_EVERY_N_STEPS   = 10
PRINT_EVERY_N_STEPS = 200


# ---------------------------------------------------------------------------
# Grasp phase
# ---------------------------------------------------------------------------

class GraspPhase(Enum):
    REACH_ABOVE    = auto()
    LOWER_TO_GRASP = auto()
    CLOSE_GRIPPER  = auto()
    ACTIVATE_WELD  = auto()
    LIFT           = auto()
    TASK           = auto()


def cartesian_pd_torque(
    model: mujoco.MjModel,
    data:  mujoco.MjData,
    target_pos: np.ndarray,
    prev_ee: "np.ndarray | None" = None,
    prev_t:  "float | None"      = None,
    kp: float = GRASP_KP,
    kd: float = GRASP_KD,
    null_damp: float = GRASP_NULL_DAMP,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Simple Cartesian PD controller for the 7 arm joints.

    Returns (tau[7], ee_pos[3], pos_error_scalar).
    """
    n = 7
    ee_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    ee_pos = data.site_xpos[ee_id].copy()

    t = float(data.time)
    if prev_ee is not None and prev_t is not None and (t - prev_t) > 1e-9:
        ee_vel = (ee_pos - prev_ee) / (t - prev_t)
    else:
        ee_vel = np.zeros(3)

    pos_error_vec = target_pos - ee_pos
    pos_error     = float(np.linalg.norm(pos_error_vec))

    J_full = get_ee_jacobian(model, data)   # (6, 7)
    J3     = J_full[:3, :]                  # (3, 7) translational
    J_pinv = pseudo_inverse(J3, 5e-3)

    F_pos = kp * pos_error_vec - kd * ee_vel
    tau   = J3.T @ F_pos

    tau += data.qfrc_bias[:n]

    N = np.eye(n) - J_pinv @ J3
    tau += N @ (-null_damp * data.qvel[:n])

    max_tau = np.array([87, 87, 87, 87, 12, 12, 12], dtype=float)
    tau = np.clip(tau, -max_tau, max_tau)

    return tau, ee_pos, pos_error


class GraspStateMachine:
    """State machine for the cable pickup sequence."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.phase: GraspPhase = GraspPhase.REACH_ABOVE
        self._phase_start_time: float = 0.0

        self.weld_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_EQUALITY, "grasp_weld")
        self.connector_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "cable_connector")
        self.gripper_act_idx = 7

        self._prev_ee_pos: "np.ndarray | None" = None
        self._prev_time:   "float | None"       = None

    def reset(self, data: mujoco.MjData) -> None:
        self.phase = GraspPhase.REACH_ABOVE
        self._phase_start_time = float(data.time)
        self._prev_ee_pos = None
        self._prev_time   = None
        if self.weld_id >= 0:
            data.eq_active[self.weld_id] = False

    def _connector_pos(self, data: mujoco.MjData) -> np.ndarray:
        return data.xpos[self.connector_body_id].copy()

    def step(self, model: mujoco.MjModel,
             data: mujoco.MjData) -> tuple[np.ndarray, bool]:
        """Executes one control step. Returns (tau[7], is_done)."""
        t             = float(data.time)
        connector_pos = self._connector_pos(data)

        if self.phase == GraspPhase.REACH_ABOVE:
            target = connector_pos + np.array([0, 0, GRASP_APPROACH_HEIGHT_OFFSET])
            tau, ee_pos, err = cartesian_pd_torque(
                model, data, target,
                prev_ee=self._prev_ee_pos, prev_t=self._prev_time)
            data.ctrl[self.gripper_act_idx] = GRIPPER_OPEN_CTRL
            if err < REACH_POS_TOL:
                print(f"[GRASP] REACH_ABOVE → LOWER_TO_GRASP  (err={err:.4f}m, t={t:.1f}s)")
                self.phase = GraspPhase.LOWER_TO_GRASP
                self._phase_start_time = t

        elif self.phase == GraspPhase.LOWER_TO_GRASP:
            target = connector_pos + np.array([0, 0, GRASP_HEIGHT_OFFSET])
            tau, ee_pos, err = cartesian_pd_torque(
                model, data, target,
                prev_ee=self._prev_ee_pos, prev_t=self._prev_time)
            data.ctrl[self.gripper_act_idx] = GRIPPER_OPEN_CTRL
            if err < LOWER_POS_TOL:
                print(f"[GRASP] LOWER_TO_GRASP → CLOSE_GRIPPER  (err={err:.4f}m, t={t:.1f}s)")
                self.phase = GraspPhase.CLOSE_GRIPPER
                self._phase_start_time = t

        elif self.phase == GraspPhase.CLOSE_GRIPPER:
            target = connector_pos + np.array([0, 0, GRASP_HEIGHT_OFFSET])
            tau, ee_pos, err = cartesian_pd_torque(
                model, data, target,
                prev_ee=self._prev_ee_pos, prev_t=self._prev_time)
            data.ctrl[self.gripper_act_idx] = GRIPPER_CLOSE_CTRL
            if t - self._phase_start_time >= CLOSE_GRIPPER_DURATION:
                print(f"[GRASP] CLOSE_GRIPPER → ACTIVATE_WELD  (t={t:.1f}s)")
                self.phase = GraspPhase.ACTIVATE_WELD
                self._phase_start_time = t

        elif self.phase == GraspPhase.ACTIVATE_WELD:
            target = connector_pos + np.array([0, 0, GRASP_HEIGHT_OFFSET])
            tau, ee_pos, err = cartesian_pd_torque(
                model, data, target,
                prev_ee=self._prev_ee_pos, prev_t=self._prev_time)
            data.ctrl[self.gripper_act_idx] = GRIPPER_CLOSE_CTRL
            if self.weld_id >= 0 and not bool(data.eq_active[self.weld_id]):
                data.eq_active[self.weld_id] = True
                print(f"[GRASP] Weld ACTIVE  (t={t:.1f}s)")
            if t - self._phase_start_time >= WELD_ACTIVATE_DELAY:
                print(f"[GRASP] ACTIVATE_WELD → LIFT  (t={t:.1f}s)")
                self.phase = GraspPhase.LIFT
                self._phase_start_time = t

        elif self.phase == GraspPhase.LIFT:
            target = np.array([connector_pos[0], connector_pos[1], LIFT_HEIGHT])
            tau, ee_pos, err = cartesian_pd_torque(
                model, data, target,
                prev_ee=self._prev_ee_pos, prev_t=self._prev_time)
            data.ctrl[self.gripper_act_idx] = GRIPPER_CLOSE_CTRL
            if err < LIFT_POS_TOL:
                print(f"[GRASP] LIFT → TASK  (err={err:.4f}m, t={t:.1f}s)")
                self.phase = GraspPhase.TASK
                self._phase_start_time = t

        else:  # TASK
            ee_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
            ee_pos = data.site_xpos[ee_id].copy()
            self._prev_ee_pos = ee_pos
            self._prev_time   = t
            return np.zeros(7), True

        ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
        self._prev_ee_pos = data.site_xpos[ee_id].copy()
        self._prev_time   = t
        return tau, False


# ---------------------------------------------------------------------------
# Trajectory — same parametrization as cable_grasp_orchestrator
# ---------------------------------------------------------------------------

def _uv_unit(s: float) -> tuple[float, float]:
    """s ∈ [0,1) → (u, v) on the unit-cycle for TRAJ_SHAPE."""
    _H = math.sqrt(3.0) / 2.0
    if TRAJ_SHAPE == "circle":
        theta = 2.0 * math.pi * s
        return math.cos(theta), math.sin(theta)
    if TRAJ_SHAPE == "rectangle":
        if s < 0.125:  return 1.0, 8.0 * s
        if s < 0.375:  return 2.0 - 8.0 * s, 1.0
        if s < 0.625:  return -1.0, 4.0 - 8.0 * s
        if s < 0.875:  return 8.0 * s - 6.0, -1.0
        return 1.0, 8.0 * s - 8.0
    # triangle
    if s < 1.0 / 3.0:
        f = 3.0 * s
        return 1.0 - 1.5 * f, _H * f
    if s < 2.0 / 3.0:
        f = 3.0 * s - 1.0
        return -0.5, _H * (1.0 - 2.0 * f)
    f = 3.0 * s - 2.0
    return -0.5 + 1.5 * f, _H * (f - 1.0)


def trajectory_target(t: float, center_xy: np.ndarray) -> np.ndarray:
    """Returns target [x, y] at time t from task start.

    Identical parametrization to cable_grasp_orchestrator: trajectory
    starts at center_xy (t=0) and traces TRAJ_SHAPE with TRAJ_RADIUS.
    """
    s = (t / TRAJ_PERIOD) - math.floor(t / TRAJ_PERIOD)
    u, v = _uv_unit(s)
    return center_xy + np.array([TRAJ_RADIUS * (u - 1.0), TRAJ_RADIUS * v])


# ---------------------------------------------------------------------------
# Simulation reset
# ---------------------------------------------------------------------------

def reset_simulation(model: mujoco.MjModel, data: mujoco.MjData,
                     grasp_sm: "GraspStateMachine | None" = None) -> None:
    """Resets the simulation: arm at home, fingers open, connector on the floor."""
    mujoco.mj_resetData(model, data)

    # Arm at home
    data.qpos[:7] = QPOS_INIT_ROBOT.copy()

    # Fingers open (0.04m each)
    fj1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "finger_joint1")
    fj2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "finger_joint2")
    if fj1_id >= 0:
        data.qpos[model.jnt_qposadr[fj1_id]] = 0.04
    if fj2_id >= 0:
        data.qpos[model.jnt_qposadr[fj2_id]] = 0.04

    # Connector at initial position
    cc_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cable_connector_joint")
    if cc_jnt_id >= 0:
        adr = model.jnt_qposadr[cc_jnt_id]
        data.qpos[adr:adr+3] = CONNECTOR_POS_INIT.copy()
        data.qpos[adr+3]     = 1.0   # quat w
        data.qpos[adr+4:adr+7] = 0.0

    # Gripper open, arm with no torque
    data.ctrl[:7] = 0.0
    data.ctrl[7]  = GRIPPER_OPEN_CTRL

    # Weld inactive
    if grasp_sm is not None:
        weld_id = grasp_sm.weld_id
    else:
        weld_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "grasp_weld")
    if weld_id >= 0:
        data.eq_active[weld_id] = False

    mujoco.mj_forward(model, data)

    if grasp_sm is not None:
        grasp_sm.reset(data)


# ---------------------------------------------------------------------------
# Visual target marker
# ---------------------------------------------------------------------------

def update_target_marker(model: mujoco.MjModel, data: mujoco.MjData,
                         xy: np.ndarray) -> None:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_marker")
    if body_id < 0:
        return
    mocap_id = model.body_mocapid[body_id]
    if mocap_id >= 0:
        data.mocap_pos[mocap_id] = [xy[0], xy[1], TRAJ_Z_FIXED]


# ---------------------------------------------------------------------------
# CSV logger
# ---------------------------------------------------------------------------

class SimLogger:
    HEADER = [
        "time", "ee_x", "ee_y", "ee_z",
        "tgt_x", "tgt_y",
        "tension_N", "pos_error_m", "cable_taut",
        "cable_dist_m",
    ]

    def __init__(self, log_dir: str = "logs"):
        os.makedirs(log_dir, exist_ok=True)
        ts       = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(log_dir, f"sim_{ts}.csv")
        self._file   = open(filepath, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.HEADER)
        self._writer.writeheader()
        self.filepath = filepath
        print(f"[LOG] Saving data to: {filepath}")

    def write(self, t: float, info: dict, target_xy: np.ndarray) -> None:
        ee = info["ee_pos"]
        self._writer.writerow({
            "time":         f"{t:.4f}",
            "ee_x":         f"{ee[0]:.5f}",
            "ee_y":         f"{ee[1]:.5f}",
            "ee_z":         f"{ee[2]:.5f}",
            "tgt_x":        f"{target_xy[0]:.5f}",
            "tgt_y":        f"{target_xy[1]:.5f}",
            "tension_N":    f"{info['tension_estimated']:.4f}",
            "pos_error_m":  f"{info['pos_error']:.5f}",
            "cable_taut":   int(info["cable_taut"]),
            "cable_dist_m": f"{info['cable_dist']:.5f}",
        })

    def close(self) -> None:
        self._file.close()
        print(f"[LOG] File closed: {self.filepath}")


# ---------------------------------------------------------------------------
# Main loop — interactive mode
# ---------------------------------------------------------------------------

def run_simulation() -> None:
    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
    except Exception as e:
        print(f"[ERROR] Could not load '{XML_PATH}': {e}")
        return

    data = mujoco.MjData(model)

    grasp_sm = GraspStateMachine(model, data)
    ctrl = MultiPriorityController(
        cable_length    = CABLE_LENGTH,
        tension_desired = TENSION_DESIRED,
        pos_kp          = POS_KP,
        pos_kd          = POS_KD,
        fixed_z         = TRAJ_Z_FIXED,
        null_damping    = NULL_DAMPING,
        gravity_comp    = True,
    )

    reset_simulation(model, data, grasp_sm)
    logger = SimLogger()

    state = {
        "paused":     False,
        "reset":      False,
        "quit":       False,
        "step_count": 0,
    }

    def key_callback(keycode: int) -> None:
        if keycode == ord(' '):
            state["paused"] = not state["paused"]
            status = "PAUSED" if state["paused"] else "RUNNING"
            print(f"[INFO] Simulation {status}")
        elif keycode in (ord('R'), ord('r')):
            state["reset"] = True
        elif keycode in (ord('Q'), ord('q')):
            state["quit"] = True

    _print_header()

    with mujoco.viewer.launch_passive(model, data,
                                      key_callback=key_callback) as viewer:
        viewer.cam.distance  = 2.0
        viewer.cam.elevation = -30
        viewer.cam.azimuth   = 150

        arc_start_time:  "float | None"      = None
        traj_center_xy:  "np.ndarray | None" = None

        while viewer.is_running() and not state["quit"]:

            if state["reset"]:
                reset_simulation(model, data, grasp_sm)
                ctrl.reset()
                state["step_count"] = 0
                state["reset"]      = False
                arc_start_time      = None
                traj_center_xy      = None
                print("[INFO] Simulation reset.")

            if not state["paused"]:
                t          = float(data.time)
                step_count = state["step_count"]
                verbose    = (step_count % PRINT_EVERY_N_STEPS == 0)

                if grasp_sm.phase != GraspPhase.TASK:
                    tau, done = grasp_sm.step(model, data)
                    data.ctrl[:7] = tau
                    if verbose:
                        ee_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
                        ee_pos = data.site_xpos[ee_id]
                        print(f"  [t={t:.1f}s] GRASP: {grasp_sm.phase.name} | "
                              f"EE: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")
                else:
                    if arc_start_time is None:
                        arc_start_time = t
                        ee_id_tmp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
                        traj_center_xy = data.site_xpos[ee_id_tmp][:2].copy()
                        print(f"[INFO] Trajectory center: {traj_center_xy}  shape={TRAJ_SHAPE}  r={TRAJ_RADIUS}m")
                    arc_t     = t - arc_start_time
                    target_xy = trajectory_target(arc_t, traj_center_xy)

                    tau, info = ctrl.compute(model, data, target_xy, verbose=verbose)
                    data.ctrl[:7] = tau
                    data.ctrl[7]  = GRIPPER_CLOSE_CTRL

                    if verbose:
                        ee = info["ee_pos"]
                        print(f"  [t={t:.1f}s] TASK | Target: ({target_xy[0]:.3f}, {target_xy[1]:.3f}) | "
                              f"Error: {info['pos_error']:.4f}m | "
                              f"Tension: {info['tension_estimated']:.1f}N | "
                              f"Taut: {info['cable_taut']}")

                    update_target_marker(model, data, target_xy)

                    if step_count % LOG_EVERY_N_STEPS == 0:
                        logger.write(t, info, target_xy)

                mujoco.mj_step(model, data)
                state["step_count"] += 1

            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - 0.001))

    logger.close()
    print("[INFO] Simulation finished.")


# ---------------------------------------------------------------------------
# Headless mode — simulation + plots
# ---------------------------------------------------------------------------

def run_headless_and_plot(duration: float = 40.0) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARNING] matplotlib not installed. Install with: pip install matplotlib")
        return

    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
    except Exception as e:
        print(f"[ERROR] Could not load '{XML_PATH}': {e}")
        return

    data = mujoco.MjData(model)

    grasp_sm = GraspStateMachine(model, data)
    ctrl = MultiPriorityController(
        cable_length    = CABLE_LENGTH,
        tension_desired = TENSION_DESIRED,
        pos_kp          = POS_KP,
        pos_kd          = POS_KD,
        fixed_z         = TRAJ_Z_FIXED,
        null_damping    = NULL_DAMPING,
        gravity_comp    = True,
    )

    reset_simulation(model, data, grasp_sm)
    logger = SimLogger()

    log_t:       list[float] = []
    log_ee_x:    list[float] = []
    log_ee_y:    list[float] = []
    log_ee_z:    list[float] = []
    log_tgt_x:   list[float] = []
    log_tgt_y:   list[float] = []
    log_tension: list[float] = []
    log_error:   list[float] = []

    dt    = model.opt.timestep
    steps = int(duration / dt)
    print_interval = max(1, steps // 20)

    print(f"[INFO] Running {duration}s ({steps} steps @ {1/dt:.0f}Hz) ...")
    _print_header()

    arc_start_time:  "float | None"      = None
    traj_center_xy:  "np.ndarray | None" = None

    for i in range(steps):
        t = float(data.time)

        if grasp_sm.phase != GraspPhase.TASK:
            tau, done = grasp_sm.step(model, data)
            data.ctrl[:7] = tau
        else:
            if arc_start_time is None:
                arc_start_time = t
                ee_id_tmp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
                traj_center_xy = data.site_xpos[ee_id_tmp][:2].copy()
            arc_t     = t - arc_start_time
            target_xy = trajectory_target(arc_t, traj_center_xy)

            tau, info = ctrl.compute(model, data, target_xy)
            data.ctrl[:7] = tau
            data.ctrl[7]  = GRIPPER_CLOSE_CTRL

            if i % LOG_EVERY_N_STEPS == 0:
                logger.write(t, info, target_xy)

            if i % 50 == 0:
                ee = info["ee_pos"]
                log_t.append(t)
                log_ee_x.append(ee[0])
                log_ee_y.append(ee[1])
                log_ee_z.append(ee[2])
                log_tgt_x.append(target_xy[0])
                log_tgt_y.append(target_xy[1])
                log_tension.append(info["tension_estimated"])
                log_error.append(info["pos_error"])

            if i % print_interval == 0:
                ee = info["ee_pos"]
                print(f"  [t={t:.1f}s]  EE: ({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f}) | "
                      f"Target: ({target_xy[0]:.3f}, {target_xy[1]:.3f}) | "
                      f"Tension: {info['tension_estimated']:.1f}N | "
                      f"Err: {info['pos_error']:.4f}m")

        mujoco.mj_step(model, data)

    logger.close()

    if not log_t:
        print("[WARNING] No TASK data collected (simulation too short to complete grasp?).")
        return

    print("[INFO] Generating plots ...")

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle("Franka Panda + Cable — MultiPriority Controller\n"
                 f"P1: Tension ≈ {TENSION_DESIRED}N  |  P2: 120° Arc in XY plane",
                 fontsize=12, fontweight="bold")

    ax = axes[0, 0]
    ax.plot(log_tgt_x, log_tgt_y, "g--", linewidth=2.0, label="Target (arc)")
    ax.plot(log_ee_x,  log_ee_y,  "b-",  linewidth=1.5, label="EE actual",  alpha=0.8)
    ax.scatter([ANCHOR_POS[0]], [ANCHOR_POS[1]], c="orange", s=120,
               zorder=5, label=f"Anchor ({ANCHOR_POS[0]}, {ANCHOR_POS[1]})")
    ax.scatter([0.0], [0.0], c="red", s=80, marker="^",
               zorder=5, label="Robot base")
    theta_ref = np.linspace(ARC_ANGLE_START, ARC_ANGLE_END, 100)
    ax.plot(ARC_CENTER_X + ARC_RADIUS * np.cos(theta_ref),
            ARC_CENTER_Y + ARC_RADIUS * np.sin(theta_ref),
            "g:", linewidth=1.0, alpha=0.5)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("EE Trajectory — XY Plane")
    ax.legend(fontsize=8); ax.set_aspect("equal"); ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(log_t, log_error, "r-", linewidth=1.5)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Error (m)")
    ax.set_title("End-Effector Position Error")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(log_t, log_tension, color="darkorange", linewidth=1.5, label="Estimated tension")
    ax.axhline(TENSION_DESIRED, color="k", linestyle="--", linewidth=1.2,
               label=f"Desired tension ({TENSION_DESIRED:.0f} N)")
    ax.axhline(0.0, color="gray", linestyle=":", linewidth=0.8)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Tension (N)")
    ax.set_title("Cable Tension")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3); ax.set_ylim(bottom=-1.0)

    ax = axes[1, 1]
    ax.plot(log_t, log_ee_x,  "b-",  linewidth=1.5, label="EE X actual")
    ax.plot(log_t, log_tgt_x, "b--", linewidth=1.0, label="EE X target", alpha=0.7)
    ax.plot(log_t, log_ee_y,  "r-",  linewidth=1.5, label="EE Y actual")
    ax.plot(log_t, log_tgt_y, "r--", linewidth=1.0, label="EE Y target", alpha=0.7)
    ax.plot(log_t, log_ee_z,  "g-",  linewidth=1.0, label="EE Z actual", alpha=0.6)
    ax.axhline(TRAJ_Z_FIXED, color="g", linestyle=":", linewidth=0.8,
               label=f"Z target ({TRAJ_Z_FIXED}m)", alpha=0.6)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Position (m)")
    ax.set_title("EE Coordinates vs Time")
    ax.legend(fontsize=7, ncol=2); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = "results.png"
    plt.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.show()
    print(f"[INFO] Plot saved to '{plot_path}'")


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _print_header() -> None:
    print("=" * 65)
    print("  Franka Panda + Franka Hand + Cable — MultiPriority")
    print("=" * 65)
    print(f"  Anchor:         {ANCHOR_POS}")
    print(f"  Connector:      {CONNECTOR_POS_INIT}  (on the floor)")
    print(f"  Grasp phases:   REACH_ABOVE → LOWER → CLOSE → WELD → LIFT → TASK")
    print(f"  Trajectory:     shape={TRAJ_SHAPE}  radius={TRAJ_RADIUS}m  period={TRAJ_PERIOD}s")
    print(f"  Fixed Z:        {TRAJ_Z_FIXED}m  (center captured from EE at task start)")
    print(f"  Target tension: {TENSION_DESIRED} N")
    print("-" * 65)
    print("  SPACE → pause/resume  |  R → reset  |  Q → quit")
    print("=" * 65)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--headless":
        dur = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
        run_headless_and_plot(dur)
    else:
        run_simulation()
