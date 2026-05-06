#!/usr/bin/env python3
"""Cable grasp orchestrator (deterministic, image-free).

Sequence:
  1. MOVE_HOME_INIT  — go to predefined home pose (MoveIt).
  2. MOVE_TO_GRASP   — go to predefined grasp pose (MoveIt).
  3. CLOSE_GRIPPER   — close on the cable.
  4. MOVE_HOME_FINAL — go back to home pose with cable held (MoveIt).
  5. DERIVE_ANCHOR   — compute anchor_position from the home EE pose + cable length.
  6. SET_PARAMS      — push anchor_position and desired_tension into the controller.
  7. SWITCH_CTRL     — deactivate JTC, activate multi_priority_controller.
  8. WAIT_RAMP       — wait for the controller's gain & setpoint ramp.
  9. RUN_TRAJECTORY  — publish a circular trajectory while tension is held.

This file is the only place where we orchestrate; multi_priority_controller.cpp
is untouched.

Services:
  /cable_grasp/start  (std_srvs/Trigger)
  /cable_grasp/stop   (std_srvs/Trigger)  — abort, switch back to JTC, open gripper

Topic:
  /cable_grasp/status (std_msgs/String)
"""

import math
import signal
import threading
import time
from enum import Enum, auto
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from control_msgs.action import GripperCommand
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger

from controller_manager.controller_manager_services import switch_controllers

try:
    from pymoveit2 import MoveIt2, MoveIt2State
    from pymoveit2.robots import panda
    HAS_MOVEIT = True
except ImportError:
    HAS_MOVEIT = False
    MoveIt2State = None  # type: ignore


class State(Enum):
    IDLE = auto()
    MOVE_HOME_INIT = auto()
    MOVE_TO_GRASP = auto()
    CLOSE_GRIPPER = auto()
    MOVE_HOME_FINAL = auto()
    DERIVE_ANCHOR = auto()
    SET_PARAMS = auto()
    SWITCH_CTRL = auto()
    WAIT_RAMP = auto()
    RUN_TRAJECTORY = auto()
    ABORT = auto()
    DONE = auto()
    FAILED = auto()


# robot_mode value emitted by FrankaRobotState during normal motion
ROBOT_MODE_MOVE = 5


class CableGraspOrchestrator(Node):
    def __init__(self):
        super().__init__("cable_grasp_orchestrator")

        # ── Poses (panda_link0 frame): [x, y, z, qw, qx, qy, qz] ──────────────
        self.declare_parameter("home_pose",  [0.30, 0.00, 0.50, 1.0, 0.0, 0.0, 0.0])
        self.declare_parameter("grasp_pose", [0.40, 0.00, 0.40, 1.0, 0.0, 0.0, 0.0])

        # Cable geometry
        self.declare_parameter("cable_length", 0.7)
        self.declare_parameter("cable_dir_unit", [0.0, 0.0, -1.0])
        self.declare_parameter("cable_diameter", 0.005)

        # Gripper
        self.declare_parameter("gripper_open_pos", 0.08)
        self.declare_parameter("gripper_close_effort", 20.0)

        # Controllers
        self.declare_parameter("controller_manager_name", "controller_manager")
        self.declare_parameter("approach_controller", "joint_trajectory_controller")
        self.declare_parameter("tension_controller", "multi_priority_controller")
        self.declare_parameter("desired_tension", 10.0)

        # Wait for controller ramp (matches kSetpointRampDuration in the C++ controller, +margin)
        self.declare_parameter("ramp_wait_s", 5.5)

        # Trajectory
        self.declare_parameter("trajectory.enabled", True)
        self.declare_parameter("trajectory.radius_m", 0.05)
        self.declare_parameter("trajectory.period_s", 8.0)
        self.declare_parameter("trajectory.rate_hz", 100.0)
        self.declare_parameter("trajectory.plane", "xy")  # xy | xz | yz
        self.declare_parameter("trajectory.duration_s", 0.0)  # 0 = forever

        # MoveIt motion limits
        self.declare_parameter("moveit.max_velocity", 0.2)
        self.declare_parameter("moveit.max_acceleration", 0.2)

        # ── ROS plumbing ─────────────────────────────────────────────────────
        self._cb = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(String, "/cable_grasp/status", 10)

        self.start_srv = self.create_service(
            Trigger, "/cable_grasp/start", self._on_start, callback_group=self._cb)
        self.stop_srv = self.create_service(
            Trigger, "/cable_grasp/stop", self._on_stop, callback_group=self._cb)

        self.gripper_client = ActionClient(
            self, GripperCommand, "/panda_gripper/gripper_action", callback_group=self._cb)

        # MoveIt2 wrapper
        self.moveit2 = None
        if HAS_MOVEIT:
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=panda.joint_names(),
                base_link_name=panda.base_link_name(),
                end_effector_name=panda.end_effector_name(),
                group_name="panda_arm",
                callback_group=self._cb,
                # CRITICAL: pure-async path. Default False uses plan+execute split,
                # which calls rclpy.spin_once() internally → races with our
                # MultiThreadedExecutor and crashes with `__enter__`.
                use_move_group_action=True,
            )
            self.moveit2.max_velocity = float(self.get_parameter("moveit.max_velocity").value)
            self.moveit2.max_acceleration = float(self.get_parameter("moveit.max_acceleration").value)
        else:
            self.get_logger().error("pymoveit2 not available — orchestrator cannot move the arm")

        self._cm_name = self.get_parameter("controller_manager_name").value
        self._tension_controller = self.get_parameter("tension_controller").value
        self._approach_controller = self.get_parameter("approach_controller").value
        self.set_params_client = self.create_client(
            SetParameters, f"/{self._tension_controller}/set_parameters",
            callback_group=self._cb)

        # Publisher for the controller's desired EE pose (used by trajectory)
        self.desired_pose_pub = self.create_publisher(
            PoseStamped, f"/{self._tension_controller}/desired_ee_pose", 10)

        # Subscribers
        self.create_subscription(
            FrankaRobotState, "/franka_robot_state_broadcaster/robot_state",
            self._on_robot_state, qos_profile_sensor_data, callback_group=self._cb)

        # ── State ────────────────────────────────────────────────────────────
        self._state = State.IDLE
        self._state_start = self.get_clock().now()
        self._latest_pose_T: Optional[np.ndarray] = None
        self._latest_robot_mode: Optional[int] = None
        self._derived_anchor: Optional[List[float]] = None
        self._home_T: Optional[np.ndarray] = None  # 4×4 EE pose at moment of MPC switch
        self._fail_reason = ""

        # Async helpers
        self._async_in_flight = False
        self._async_done = False
        self._async_ok = False
        self._async_payload = None

        # Trajectory thread
        self._traj_thread: Optional[threading.Thread] = None
        self._traj_stop = threading.Event()

        # FSM tick at 20 Hz
        self._timer = self.create_timer(0.05, self._tick, callback_group=self._cb)

        self.get_logger().info("Cable grasp orchestrator ready. Call /cable_grasp/start to begin.")

    # ── Subscriptions ───────────────────────────────────────────────────────
    def _on_robot_state(self, msg: FrankaRobotState):
        self._latest_pose_T = np.array(msg.o_t_ee, dtype=float).reshape(4, 4, order="F")
        self._latest_robot_mode = int(msg.robot_mode)

    # ── Service callbacks ───────────────────────────────────────────────────
    def _on_start(self, request, response):
        if self._state not in (State.IDLE, State.DONE, State.FAILED):
            response.success = False
            response.message = f"already running ({self._state.name})"
            return response
        self.get_logger().info("/cable_grasp/start — kicking off sequence")
        self._fail_reason = ""
        self._derived_anchor = None
        self._home_T = None
        self._traj_stop.clear()
        self._set_state(State.MOVE_HOME_INIT)
        response.success = True
        response.message = "started"
        return response

    def _on_stop(self, request, response):
        self.get_logger().warn("/cable_grasp/stop — aborting")
        self._fail_reason = "stop requested"
        self._traj_stop.set()
        self._set_state(State.ABORT)
        response.success = True
        response.message = "aborting"
        return response

    # ── FSM driver ──────────────────────────────────────────────────────────
    def _set_state(self, new_state: State):
        if new_state != self._state:
            self.get_logger().info(f"FSM: {self._state.name} → {new_state.name}")
        self._state = new_state
        self._state_start = self.get_clock().now()
        self._async_in_flight = False
        self._async_done = False
        self._async_ok = False
        self._async_payload = None
        msg = String()
        msg.data = f"{new_state.name} reason={self._fail_reason}"
        self.status_pub.publish(msg)

    def _age_s(self) -> float:
        return (self.get_clock().now() - self._state_start).nanoseconds * 1e-9

    def _tick(self):
        try:
            self._tick_impl()
        except Exception as e:
            self.get_logger().error(f"FSM tick: {e}", throttle_duration_sec=1.0)
            if self._state not in (State.IDLE, State.ABORT, State.FAILED, State.DONE):
                self._fail_reason = f"exception: {e}"
                self._set_state(State.ABORT)

    def _tick_impl(self):
        s = self._state
        if s in (State.IDLE, State.DONE, State.FAILED):
            return

        # Reflex monitor while tension control is active.
        if s in (State.WAIT_RAMP, State.RUN_TRAJECTORY) and self._latest_robot_mode is not None:
            if self._latest_robot_mode != ROBOT_MODE_MOVE:
                self._fail_reason = f"robot_mode={self._latest_robot_mode} (reflex)"
                self._set_state(State.ABORT)
                return

        handler = getattr(self, f"_st_{s.name.lower()}", None)
        if handler is None:
            self._fail_reason = f"no handler for {s.name}"
            self._set_state(State.ABORT)
            return
        handler()

    # ── State handlers ──────────────────────────────────────────────────────
    def _st_move_home_init(self):
        self._move_to_pose_async(
            list(self.get_parameter("home_pose").value),
            cartesian=False,
            next_state=State.MOVE_TO_GRASP)

    def _st_move_to_grasp(self):
        self._move_to_pose_async(
            list(self.get_parameter("grasp_pose").value),
            cartesian=False,
            next_state=State.CLOSE_GRIPPER)

    def _st_close_gripper(self):
        diameter = float(self.get_parameter("cable_diameter").value)
        self._gripper_command_async(
            position=max(0.0, 0.7 * diameter),
            effort=float(self.get_parameter("gripper_close_effort").value),
            next_state=State.MOVE_HOME_FINAL)

    def _st_move_home_final(self):
        self._move_to_pose_async(
            list(self.get_parameter("home_pose").value),
            cartesian=False,
            next_state=State.DERIVE_ANCHOR)

    def _st_derive_anchor(self):
        if self._latest_pose_T is None:
            self._fail_reason = "no robot pose"
            self._set_state(State.ABORT)
            return
        self._home_T = self._latest_pose_T.copy()
        p = self._home_T[:3, 3]
        L = float(self.get_parameter("cable_length").value)
        d = np.array(self.get_parameter("cable_dir_unit").value, dtype=float)
        if np.linalg.norm(d) < 1e-6:
            d = np.array([0.0, 0.0, -1.0])
        d = d / np.linalg.norm(d)
        anchor = p + L * d
        self._derived_anchor = [float(anchor[0]), float(anchor[1]), float(anchor[2])]
        self.get_logger().info(
            f"anchor = [{anchor[0]:.3f}, {anchor[1]:.3f}, {anchor[2]:.3f}] "
            f"(p_home=[{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}], L={L}, dir={d.tolist()})")
        self._set_state(State.SET_PARAMS)

    def _st_set_params(self):
        if not self._async_in_flight:
            self._async_in_flight = True
            threading.Thread(target=self._set_params_worker, daemon=True).start()
            return
        if self._async_done:
            if self._async_ok:
                self._set_state(State.SWITCH_CTRL)
            else:
                self._fail_reason = "set_parameters failed"
                self._set_state(State.ABORT)

    def _set_params_worker(self):
        try:
            if not self.set_params_client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(f"set_parameters service /{self._tension_controller} unavailable")
            req = SetParameters.Request()

            anchor_p = Parameter(name="anchor_position",
                                 value=ParameterValue(
                                     type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                     double_array_value=self._derived_anchor))
            req.parameters.append(anchor_p)

            tension_p = Parameter(name="desired_tension",
                                  value=ParameterValue(
                                      type=ParameterType.PARAMETER_DOUBLE,
                                      double_value=float(self.get_parameter("desired_tension").value)))
            req.parameters.append(tension_p)

            fut = self.set_params_client.call_async(req)
            t0 = self.get_clock().now()
            while not fut.done() and (self.get_clock().now() - t0).nanoseconds * 1e-9 < 5.0:
                time.sleep(0.05)  # MultiThreadedExecutor spins this future for us
            if not fut.done():
                raise RuntimeError("set_parameters timed out")
            for p, r in zip(req.parameters, fut.result().results):
                if not r.successful:
                    raise RuntimeError(f"failed to set {p.name}: {r.reason}")
            self._async_ok = True
        except Exception as e:
            self.get_logger().error(f"set_params: {e}")
            self._async_ok = False
        finally:
            self._async_done = True

    def _st_switch_ctrl(self):
        if not self._async_in_flight:
            self._async_in_flight = True
            threading.Thread(target=self._switch_worker,
                             args=(self._approach_controller, self._tension_controller),
                             daemon=True).start()
            return
        if self._async_done:
            if self._async_ok:
                self._set_state(State.WAIT_RAMP)
            else:
                self._fail_reason = "switch_controller failed"
                self._set_state(State.ABORT)

    def _switch_worker(self, deactivate_name: str, activate_name: str):
        try:
            switch_controllers(
                self, self._cm_name,
                deactivate_controllers=[deactivate_name] if deactivate_name else [],
                activate_controllers=[activate_name] if activate_name else [],
                strict=False, activate_asap=False, timeout=2.0,
            )
            self._async_ok = True
        except Exception as e:
            self.get_logger().error(f"switch_worker {deactivate_name}→{activate_name}: {e}")
            self._async_ok = False
        finally:
            self._async_done = True

    def _st_wait_ramp(self):
        ramp_s = float(self.get_parameter("ramp_wait_s").value)
        if self._age_s() >= ramp_s:
            if not bool(self.get_parameter("trajectory.enabled").value):
                self.get_logger().info("Ramp finished; trajectory disabled — DONE")
                self._set_state(State.DONE)
            else:
                self._set_state(State.RUN_TRAJECTORY)

    def _st_run_trajectory(self):
        if self._traj_thread is None or not self._traj_thread.is_alive():
            self._traj_stop.clear()
            self._traj_thread = threading.Thread(target=self._trajectory_worker, daemon=True)
            self._traj_thread.start()
            return
        # Optional finite-duration trajectory
        duration = float(self.get_parameter("trajectory.duration_s").value)
        if duration > 0.0 and self._age_s() >= duration:
            self._traj_stop.set()
            self._traj_thread.join(timeout=2.0)
            self._set_state(State.DONE)

    def _trajectory_worker(self):
        if self._home_T is None:
            self.get_logger().error("traj: no home pose captured")
            return
        center = self._home_T[:3, 3].copy()
        R = self._home_T[:3, :3].copy()  # orientation kept fixed to home

        radius = float(self.get_parameter("trajectory.radius_m").value)
        period = max(0.1, float(self.get_parameter("trajectory.period_s").value))
        rate = max(1.0, float(self.get_parameter("trajectory.rate_hz").value))
        plane = str(self.get_parameter("trajectory.plane").value).lower()

        if plane == "xy":
            ax_a, ax_b = 0, 1
        elif plane == "xz":
            ax_a, ax_b = 0, 2
        elif plane == "yz":
            ax_a, ax_b = 1, 2
        else:
            self.get_logger().warn(f"traj: unknown plane '{plane}', defaulting to xy")
            ax_a, ax_b = 0, 1

        # Quaternion from R (column-major to xyzw)
        # R is row-major already (Eigen-style here is column-major in flat array, but
        # we're working with the 3x3 numpy view which behaves naturally).
        qw, qx, qy, qz = self._matrix_to_quat(R)

        self.get_logger().info(
            f"traj: circle r={radius} m, period={period} s, rate={rate} Hz, plane={plane}")

        omega = 2.0 * math.pi / period
        t0 = self.get_clock().now()
        dt = 1.0 / rate

        while rclpy.ok() and not self._traj_stop.is_set():
            now = self.get_clock().now()
            t = (now - t0).nanoseconds * 1e-9
            offset = np.zeros(3)
            offset[ax_a] = radius * math.cos(omega * t)
            offset[ax_b] = radius * math.sin(omega * t)
            # subtract starting offset so the first sample equals the home position
            if abs(t) < 1e-9:
                offset[ax_a] = radius  # cos(0)=1
                offset[ax_b] = 0.0
            target = center + (offset - np.array([radius if i == ax_a else 0.0 for i in range(3)]))

            ps = PoseStamped()
            ps.header.stamp = now.to_msg()
            ps.header.frame_id = "panda_link0"
            ps.pose.position.x = float(target[0])
            ps.pose.position.y = float(target[1])
            ps.pose.position.z = float(target[2])
            ps.pose.orientation.w = float(qw)
            ps.pose.orientation.x = float(qx)
            ps.pose.orientation.y = float(qy)
            ps.pose.orientation.z = float(qz)
            self.desired_pose_pub.publish(ps)

            self._traj_stop.wait(timeout=dt)

        self.get_logger().info("traj: stopped")

    @staticmethod
    def _matrix_to_quat(R: np.ndarray):
        # Standard rotation matrix to quaternion (w, x, y, z)
        m = R
        tr = m[0, 0] + m[1, 1] + m[2, 2]
        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * S
            qx = (m[2, 1] - m[1, 2]) / S
            qy = (m[0, 2] - m[2, 0]) / S
            qz = (m[1, 0] - m[0, 1]) / S
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            S = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            qw = (m[2, 1] - m[1, 2]) / S
            qx = 0.25 * S
            qy = (m[0, 1] + m[1, 0]) / S
            qz = (m[0, 2] + m[2, 0]) / S
        elif m[1, 1] > m[2, 2]:
            S = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            qw = (m[0, 2] - m[2, 0]) / S
            qx = (m[0, 1] + m[1, 0]) / S
            qy = 0.25 * S
            qz = (m[1, 2] + m[2, 1]) / S
        else:
            S = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            qw = (m[1, 0] - m[0, 1]) / S
            qx = (m[0, 2] + m[2, 0]) / S
            qy = (m[1, 2] + m[2, 1]) / S
            qz = 0.25 * S
        return qw, qx, qy, qz

    def _st_abort(self):
        if not self._async_in_flight:
            self._async_in_flight = True
            self._traj_stop.set()
            threading.Thread(target=self._abort_worker, daemon=True).start()
            return
        if self._async_done:
            self._set_state(State.FAILED)

    def _abort_worker(self):
        try:
            if self._traj_thread is not None and self._traj_thread.is_alive():
                self._traj_thread.join(timeout=1.5)
            try:
                switch_controllers(
                    self, self._cm_name,
                    deactivate_controllers=[self._tension_controller],
                    activate_controllers=[self._approach_controller],
                    strict=False, activate_asap=False, timeout=2.0)
            except Exception as e:
                self.get_logger().warn(f"abort: switch back failed: {e}")
            try:
                self._gripper_command_blocking(
                    position=float(self.get_parameter("gripper_open_pos").value),
                    effort=0.0)
            except Exception as e:
                self.get_logger().warn(f"abort: open gripper failed: {e}")
        finally:
            self._async_done = True

    # ── Async helpers ───────────────────────────────────────────────────────
    # MoveIt is polled directly from the FSM tick (not a worker thread) because
    # pymoveit2.wait_until_executed() calls rclpy.spin_once on this node, which
    # races with the MultiThreadedExecutor and crashes with `__enter__`.
    def _move_to_pose_async(self, pose, cartesian: bool, next_state: State):
        if self.moveit2 is None:
            self._fail_reason = "pymoveit2 not available"
            self._set_state(State.ABORT)
            return

        # Phase 1: kick off the move on the first tick of this state
        if not self._async_in_flight:
            try:
                quat_xyzw = [pose[4], pose[5], pose[6], pose[3]]  # (qx, qy, qz, qw)
                self.moveit2.move_to_pose(
                    position=pose[0:3],
                    quat_xyzw=quat_xyzw,
                    cartesian=cartesian,
                )
            except Exception as e:
                self.get_logger().error(f"move_to_pose request: {e}")
                self._fail_reason = f"move request: {e}"
                self._set_state(State.ABORT)
                return
            self._async_in_flight = True
            self._async_payload = {
                "saw_active": False,
                "deadline_s": self._age_s() + 30.0,
            }
            return

        # Phase 2: poll query_state until we observe a non-IDLE state followed by IDLE
        st = self.moveit2.query_state()
        payload = self._async_payload or {}
        self.get_logger().info(
            f"poll: state={st.name} saw_active={payload.get('saw_active', False)} age={self._age_s():.1f}s",
            throttle_duration_sec=0.5)
        if st in (MoveIt2State.REQUESTING, MoveIt2State.EXECUTING):
            payload["saw_active"] = True
            self._async_payload = payload
        elif st == MoveIt2State.IDLE and payload.get("saw_active"):
            self._set_state(next_state)
            return

        if self._age_s() > payload.get("deadline_s", 30.0):
            self._fail_reason = f"MoveIt move to {pose} timed out"
            self._set_state(State.ABORT)

    def _gripper_command_async(self, position: float, effort: float, next_state: State):
        if not self._async_in_flight:
            self._async_in_flight = True
            threading.Thread(target=self._gripper_worker,
                             args=(position, effort), daemon=True).start()
            return
        if self._async_done:
            if self._async_ok:
                self._set_state(next_state)
            else:
                self._fail_reason = f"gripper (pos={position}, effort={effort}) failed"
                self._set_state(State.ABORT)

    def _gripper_worker(self, position: float, effort: float):
        try:
            self._async_payload = self._gripper_command_blocking(position, effort)
            self._async_ok = True
        except Exception as e:
            self.get_logger().error(f"gripper: {e}")
            self._async_ok = False
        finally:
            self._async_done = True

    def _gripper_command_blocking(self, position: float, effort: float):
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("gripper action server not available")
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)
        gh_fut = self.gripper_client.send_goal_async(goal)
        t0 = self.get_clock().now()
        while not gh_fut.done() and (self.get_clock().now() - t0).nanoseconds * 1e-9 < 5.0:
            time.sleep(0.05)  # MultiThreadedExecutor advances this future
        if not gh_fut.done():
            raise RuntimeError("gripper goal accept timed out")
        gh = gh_fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("gripper goal rejected")
        res_fut = gh.get_result_async()
        t0 = self.get_clock().now()
        while not res_fut.done() and (self.get_clock().now() - t0).nanoseconds * 1e-9 < 10.0:
            time.sleep(0.05)
        if not res_fut.done():
            raise RuntimeError("gripper action timed out")
        return res_fut.result().result


def main():
    rclpy.init()
    node = CableGraspOrchestrator()

    def _sigint_handler(signum, frame):
        node.get_logger().warn("SIGINT — abort")
        node._fail_reason = "SIGINT"
        node._traj_stop.set()
        if node._state not in (State.IDLE, State.ABORT, State.FAILED, State.DONE):
            node._set_state(State.ABORT)

    signal.signal(signal.SIGINT, _sigint_handler)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
