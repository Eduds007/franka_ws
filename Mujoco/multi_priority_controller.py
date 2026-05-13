"""
multi_priority_controller.py
============================
Multi-Priority Controller for Franka Panda with cable.

Priority hierarchy (highest to lowest):
  P1 — Maintain cable tension equal to tension_desired (N)
  P2 — Track target end-effector position in XY plane (fixed Z)

Method: Null-Space Projection (Siciliano & Slotine)
  τ = J1ᵀ F1  +  (I − J1ᵀ J1#ᵀ) J2ᵀ F2  +  τ_grav  +  N1 N2 (−kd q̇)

P1 formulation — cable tension:
  The cable connects the end-effector (EE) to the fixed anchor point.
  Tension is the force the robot applies to resist the cable,
  in the direction from EE TOWARD the anchor.

  F1 = tension_desired × ĉ
  where ĉ = (anchor − ee) / ‖anchor − ee‖  (EE→anchor direction)

  J1 = ĉᵀ J_ee[:3, :]   ∈ ℝ^(1×7)
  N1 = I − J1† J1         (null-space of the tension task)

  The null-space of J1 corresponds to TANGENTIAL movements of the cable,
  which are exactly the arc movements in the XY plane — showing
  that P1 and P2 are mathematically complementary for this geometry.

Actual tension estimate:
  Read from data.qfrc_constraint[:7] (constraint forces in joint space)
  projected onto the cable direction:
    T_est = (J1 · τ_constraint) / ‖J1‖²
"""

import numpy as np
import mujoco


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def pseudo_inverse(J: np.ndarray, damping: float = 1e-3) -> np.ndarray:
    """Damped pseudo-inverse (Damped Least Squares — DLS).

    J#  =  Jᵀ (J Jᵀ + λ² I)⁻¹
    """
    JJT = J @ J.T
    n   = JJT.shape[0]
    return J.T @ np.linalg.inv(JJT + damping**2 * np.eye(n))


# ---------------------------------------------------------------------------
# End-effector Jacobian via MuJoCo
# ---------------------------------------------------------------------------

def get_ee_jacobian(model: mujoco.MjModel, data: mujoco.MjData,
                    ee_site_name: str = "end_effector",
                    n_arm_dof: int = 7) -> np.ndarray:
    """Geometric Jacobian (6 × n_arm_dof) of the end-effector.

    Rows 0:3 = translational, 3:6 = rotational.
    Returns only the n_arm_dof columns for arm joints (ignores fingers and free joints).
    """
    nv   = model.nv
    Jt   = np.zeros((3, nv))
    Jr   = np.zeros((3, nv))
    sid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_site_name)
    mujoco.mj_jacSite(model, data, Jt, Jr, sid)
    J_full = np.vstack([Jt, Jr])       # (6, nv)
    return J_full[:, :n_arm_dof]        # (6, 7) — arm joints only


# ---------------------------------------------------------------------------
# Multi-Priority Controller
# ---------------------------------------------------------------------------

class MultiPriorityController:
    """
    Two-priority controller for Franka Panda + cable.

    Priority 1 (High): Cable tension
      - Applies constant tension_desired force in cable direction (EE→anchor)
      - Null-space N1 corresponds to TANGENTIAL cable movements
      - Keeps cable taut without interfering with arc motion

    Priority 2 (Low): End-effector position in XY plane
      - 3D Cartesian PD control (X, Y and fixed Z = fixed_z)
      - Projected onto P1 null-space — arc movements are
        exactly the cable tangential movements ✓

    Geometry notes:
      - Arc: radius = cable length (0.5m), center at anchor
      - At arc position: ‖EE − anchor‖ = cable_length
      - Movements along the arc are perpendicular to cable direction
      → P1 and P2 are orthogonal in this geometry

    Parameters
    ----------
    cable_length    : total cable length [m]
    tension_desired : target tension [N] — force applied in cable direction
    pos_kp          : position proportional gain [N/m]
    pos_kd          : position derivative gain [N·s/m]
    fixed_z         : fixed Z height of end-effector during arc [m]
    damping         : DLS damping for pseudo-inverses
    null_damping    : null-space damping gain [N·m·s/rad]
    gravity_comp    : enable gravity compensation
    max_torque      : per-joint torque limits [N·m]
    """

    def __init__(
        self,
        cable_length:    float             = 0.50,
        tension_desired: float             = 10.0,
        pos_kp:          float             = 150.0,
        pos_kd:          float             = 15.0,
        fixed_z:         float             = 0.40,
        damping:         float             = 5e-3,
        null_damping:    float             = 5.0,
        gravity_comp:    bool              = True,
        max_torque:      np.ndarray | None = None,
    ):
        self.cable_length    = cable_length
        self.tension_desired = tension_desired

        self.pos_kp   = pos_kp
        self.pos_kd   = pos_kd
        self.fixed_z  = fixed_z

        self.damping      = damping
        self.null_damping = null_damping
        self.gravity_comp = gravity_comp

        if max_torque is None:
            # Panda torque limits (N·m)
            self.max_torque = np.array([87, 87, 87, 87, 12, 12, 12], dtype=float)
        else:
            self.max_torque = np.asarray(max_torque, dtype=float)

        # Internal state for finite-difference velocity
        self._prev_ee_pos: np.ndarray | None = None
        self._prev_time:   float | None       = None

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Resets internal state."""
        self._prev_ee_pos = None
        self._prev_time   = None

    # ------------------------------------------------------------------
    def compute(
        self,
        model:     mujoco.MjModel,
        data:      mujoco.MjData,
        target_xy: np.ndarray,      # [x, y] target in XY plane
        verbose:   bool = False,
    ) -> tuple[np.ndarray, dict]:
        """
        Computes the torque vector (7,) for the robot joints.

        Parameters
        ----------
        target_xy : array (2,)
            Target [x, y] position of the end-effector in the XY plane.
            The Z coordinate is fixed at self.fixed_z.

        Returns
        -------
        tau  : array (7,)   — torques to apply at the joints
        info : dict         — information for logging and debug
        """
        n = 7  # number of robot joints

        # ── Current positions ─────────────────────────────────────────────
        ee_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
        anc_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "anchor_site")

        ee_pos  = data.site_xpos[ee_id].copy()    # (3,) EE position in world
        anc_pos = data.site_xpos[anc_id].copy()   # (3,) anchor

        # Direction and distance EE → anchor
        cable_vec  = anc_pos - ee_pos             # vector from EE to anchor
        cable_dist = float(np.linalg.norm(cable_vec))

        if cable_dist > 1e-6:
            cable_dir_to_anchor = cable_vec / cable_dist   # ĉ: EE → anchor (unit)
        else:
            cable_dir_to_anchor = np.array([1., 0., 0.])  # fallback

        cable_taut = (cable_dist >= self.cable_length * 0.95)

        # ── EE Jacobian ─────────────────────────────────────────────
        J_ee_full = get_ee_jacobian(model, data)   # (6, 7)
        J2        = J_ee_full[:3, :]               # (3, 7) translational
        J2_pinv   = pseudo_inverse(J2, self.damping)

        # =================================================================
        # PRIORITY 1: Cable tension
        # =================================================================
        # Cable physics:
        #   The cable pulls EE toward the anchor (tension T).
        #   To maintain tension T_desired, the robot must resist this pull
        #   by applying an EQUAL and OPPOSITE force — i.e., pushing EE
        #   AWAY from the anchor (anchor→EE direction).
        #
        # F1 = tension_desired × (−ĉ)   where ĉ = EE→anchor direction
        #     = tension_desired × ĉ_away (EE moving away from anchor)
        #
        # J1 = ĉᵀ J2  ∈ ℝ^(1×7)  — scalar tension Jacobian (radial)
        # N1 = I − J1† J1          — null-space = tangential movements (arc)

        cable_dir_away = -cable_dir_to_anchor                        # (3,) anchor→EE
        F_tension_3d   = self.tension_desired * cable_dir_away       # (3,)

        tau_tension  = J2.T @ F_tension_3d                          # (7,)

        J1      = cable_dir_to_anchor.reshape(1, 3) @ J2            # (1, 7)
        J1_pinv = pseudo_inverse(J1, self.damping)                  # (7, 1)
        N1      = np.eye(n) - J1_pinv @ J1                          # (7, 7)

        # ── Actual tension estimate (via constraint forces) ─────────
        # tau_constraint ≈ J1ᵀ T_real  →  T_real ≈ (J1 τ_c) / ‖J1‖²
        tau_constraint = data.qfrc_constraint[:n].copy()
        J1J1T          = float(J1 @ J1.T) + 1e-9  # scalar ‖J1‖² (avoids div/0)
        T_estimated    = float(J1 @ tau_constraint) / J1J1T
        T_estimated    = max(0.0, T_estimated)   # tension ≥ 0

        # =================================================================
        # PRIORITY 2: End-effector position in XY plane (fixed Z)
        # =================================================================
        target_pos = np.array([target_xy[0], target_xy[1], self.fixed_z])
        pos_error  = target_pos - ee_pos           # (3,)

        # EE velocity via finite difference
        t = float(data.time)
        if self._prev_ee_pos is not None and self._prev_time is not None:
            dt = t - self._prev_time
            if dt > 1e-9:
                ee_vel = (ee_pos - self._prev_ee_pos) / dt
            else:
                ee_vel = np.zeros(3)
        else:
            ee_vel = np.zeros(3)

        self._prev_ee_pos = ee_pos.copy()
        self._prev_time   = t

        # 3D Cartesian PD force (includes Z component to maintain fixed height)
        F_pos   = self.pos_kp * pos_error - self.pos_kd * ee_vel    # (3,)
        tau_pos = J2.T @ F_pos                                       # (7,)

        N2 = np.eye(n) - J2_pinv @ J2                               # (7, 7)

        # =================================================================
        # Multi-Priority composition:
        #   τ = τ_P1  +  N1 · τ_P2  +  τ_grav  +  N1·N2 · τ_damp
        # =================================================================
        tau = tau_tension + N1 @ tau_pos

        # Gravity compensation
        if self.gravity_comp:
            tau += data.qfrc_bias[:n]

        # Damping in combined null-space (N1 ∩ N2)
        tau += N1 @ N2 @ (-self.null_damping * data.qvel[:n])

        # Torque saturation
        tau = np.clip(tau, -self.max_torque, self.max_torque)

        # ── Log / debug ──────────────────────────────────────────────────
        if verbose:
            print(f"  [t={t:.3f}s]")
            print(f"    EE pos:       {ee_pos.round(4)}")
            print(f"    Target:       {target_pos.round(4)}")
            print(f"    Pos error:    {np.linalg.norm(pos_error):.4f} m")
            print(f"    Dist EE→anchor: {cable_dist:.4f} m "
                  f"(cable: {self.cable_length:.3f} m, taut: {cable_taut})")
            print(f"    Desired tension:   {self.tension_desired:.1f} N")
            print(f"    Estimated tension: {T_estimated:.2f} N")
            print(f"    Torques:           {tau.round(2)}")

        info = {
            "ee_pos":            ee_pos,
            "anchor_pos":        anc_pos,
            "cable_dist":        cable_dist,
            "cable_taut":        cable_taut,
            "tension_desired":   self.tension_desired,
            "tension_estimated": T_estimated,
            "pos_error":         float(np.linalg.norm(pos_error)),
            "pos_error_vec":     pos_error,
            "target_pos":        target_pos,
        }

        return tau, info
