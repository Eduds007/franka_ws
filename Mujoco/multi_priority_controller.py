"""
multi_priority_controller.py
============================
Controlador MultiPrioridade para o Franka Panda com cabo.

Hierarquia de prioridades (da mais alta para mais baixa):
  P1 — Manter tensão no cabo igual a tension_desired (N)
  P2 — Rastrear posição alvo do end-effector no plano XY (Z fixo)

Método: Null-Space Projection (Siciliano & Slotine)
  τ = J1ᵀ F1  +  (I − J1ᵀ J1#ᵀ) J2ᵀ F2  +  τ_grav  +  N1 N2 (−kd q̇)

Formulação de P1 — tensão no cabo:
  O cabo conecta o end-effector (EE) ao ponto âncora fixo.
  A tensão é a força que o robô aplica para resistir ao cabo,
  na direção que vai do EE EM DIREÇÃO ao âncora.

  F1 = tension_desired × ĉ
  onde ĉ = (anchor − ee) / ‖anchor − ee‖  (direção EE→âncora)

  J1 = ĉᵀ J_ee[:3, :]   ∈ ℝ^(1×7)
  N1 = I − J1† J1         (null-space da tarefa de tensão)

  O null-space de J1 corresponde a movimentos TANGENCIAIS ao cabo,
  que são exatamente os movimentos do arco no plano XY — demonstrando
  que P1 e P2 são matematicamente complementares para essa geometria.

Estimativa de tensão real:
  Lida de data.qfrc_constraint[:7] (forças de constraint em espaço de joint)
  projetadas na direção do cabo:
    T_est = (J1 · τ_constraint) / ‖J1‖²
"""

import numpy as np
import mujoco


# ---------------------------------------------------------------------------
# Utilitários
# ---------------------------------------------------------------------------

def pseudo_inverse(J: np.ndarray, damping: float = 1e-3) -> np.ndarray:
    """Pseudo-inversa amortecida (Damped Least Squares — DLS).

    J#  =  Jᵀ (J Jᵀ + λ² I)⁻¹
    """
    JJT = J @ J.T
    n   = JJT.shape[0]
    return J.T @ np.linalg.inv(JJT + damping**2 * np.eye(n))


# ---------------------------------------------------------------------------
# Jacobiano do end-effector via MuJoCo
# ---------------------------------------------------------------------------

def get_ee_jacobian(model: mujoco.MjModel, data: mujoco.MjData,
                    ee_site_name: str = "end_effector") -> np.ndarray:
    """Jacobiano geométrico (6 × 7) do end-effector.

    Linhas 0:3 = translacional, 3:6 = rotacional.
    Retorna apenas as 7 colunas dos joints do robô.
    """
    nv   = model.nv
    Jt   = np.zeros((3, nv))
    Jr   = np.zeros((3, nv))
    sid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_site_name)
    mujoco.mj_jacSite(model, data, Jt, Jr, sid)
    J_full = np.vstack([Jt, Jr])   # (6, nv)
    return J_full[:, :7]            # (6, 7) — apenas joints do robô


# ---------------------------------------------------------------------------
# Controlador MultiPrioridade
# ---------------------------------------------------------------------------

class MultiPriorityController:
    """
    Controlador de duas prioridades para Franka Panda + cabo.

    Prioridade 1 (Alta): Tensão no cabo
      - Aplica force constante tension_desired na direção do cabo (EE→âncora)
      - Null-space N1 corresponde a movimentos TANGENCIAIS ao cabo
      - Mantém cabo tensionado sem interferir no movimento em arco

    Prioridade 2 (Baixa): Posição do end-effector no plano XY
      - Controle PD Cartesiano 3D (X, Y e Z fixo = fixed_z)
      - Projetado no null-space de P1 — os movimentos do arco são
        exatamente os movimentos tangenciais ao cabo ✓

    Notas sobre a geometria:
      - Arco: raio = comprimento do cabo (0.5m), centro na âncora
      - Na posição do arco: ‖EE − âncora‖ = cabo_length
      - Movimentos ao longo do arco são perpendiculares à direção do cabo
      → P1 e P2 são ortogonais nessa geometria

    Parâmetros
    ----------
    cable_length    : comprimento total do cabo [m]
    tension_desired : tensão alvo [N] — força aplicada na direção do cabo
    pos_kp          : ganho proporcional de posição [N/m]
    pos_kd          : ganho derivativo de posição [N·s/m]
    fixed_z         : altura Z fixa do end-effector durante o arco [m]
    damping         : amortecimento DLS para pseudo-inversas
    null_damping    : ganho de amortecimento no null-space [N·m·s/rad]
    gravity_comp    : ativar compensação de gravidade
    max_torque      : limites de torque por junta [N·m]
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
            # Limites de torque do Panda (N·m)
            self.max_torque = np.array([87, 87, 87, 87, 12, 12, 12], dtype=float)
        else:
            self.max_torque = np.asarray(max_torque, dtype=float)

        # Estado interno para diferença finita de velocidade
        self._prev_ee_pos: np.ndarray | None = None
        self._prev_time:   float | None       = None

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Reinicia estado interno."""
        self._prev_ee_pos = None
        self._prev_time   = None

    # ------------------------------------------------------------------
    def compute(
        self,
        model:     mujoco.MjModel,
        data:      mujoco.MjData,
        target_xy: np.ndarray,      # [x, y] alvo no plano XY
        verbose:   bool = False,
    ) -> tuple[np.ndarray, dict]:
        """
        Calcula o vetor de torques (7,) para os joints do robô.

        Parâmetros
        ----------
        target_xy : array (2,)
            Posição alvo [x, y] do end-effector no plano XY.
            A coordenada Z é fixada em self.fixed_z.

        Retorna
        -------
        tau  : array (7,)   — torques a aplicar nos joints
        info : dict         — informações para logging e debug
        """
        n = 7  # número de joints do robô

        # ── Posições atuais ─────────────────────────────────────────────
        ee_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
        anc_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "anchor_site")

        ee_pos  = data.site_xpos[ee_id].copy()    # (3,) posição EE no mundo
        anc_pos = data.site_xpos[anc_id].copy()   # (3,) âncora

        # Direção e distância EE → âncora
        cable_vec  = anc_pos - ee_pos             # vetor do EE para a âncora
        cable_dist = float(np.linalg.norm(cable_vec))

        if cable_dist > 1e-6:
            cable_dir_to_anchor = cable_vec / cable_dist   # ĉ: EE → âncora (unit)
        else:
            cable_dir_to_anchor = np.array([1., 0., 0.])  # fallback

        cable_taut = (cable_dist >= self.cable_length * 0.95)

        # ── Jacobiano do EE ─────────────────────────────────────────────
        J_ee_full = get_ee_jacobian(model, data)   # (6, 7)
        J2        = J_ee_full[:3, :]               # (3, 7) translacional
        J2_pinv   = pseudo_inverse(J2, self.damping)

        # =================================================================
        # PRIORIDADE 1: Tensão no cabo
        # =================================================================
        # Física do cabo:
        #   O cabo puxa o EE em direção à âncora (tensão T).
        #   Para o robô manter tensão T_desired, deve resistir a esse puxão
        #   aplicando força IGUAL e OPOSTA — ou seja, empurrando o EE
        #   para LONGE da âncora (direção âncora→EE).
        #
        # F1 = tension_desired × (−ĉ)   onde ĉ = direção EE→âncora
        #     = tension_desired × ĉ_away (direção EE saindo da âncora)
        #
        # J1 = ĉᵀ J2  ∈ ℝ^(1×7)  — Jacobiano escalar da tensão (radial)
        # N1 = I − J1† J1          — null-space = movimentos tangenciais (arco)

        cable_dir_away = -cable_dir_to_anchor                        # (3,) âncora→EE
        F_tension_3d   = self.tension_desired * cable_dir_away       # (3,)

        tau_tension  = J2.T @ F_tension_3d                          # (7,)

        J1      = cable_dir_to_anchor.reshape(1, 3) @ J2            # (1, 7)
        J1_pinv = pseudo_inverse(J1, self.damping)                  # (7, 1)
        N1      = np.eye(n) - J1_pinv @ J1                          # (7, 7)

        # ── Estimativa da tensão real (via forças de constraint) ─────────
        # tau_constraint ≈ J1ᵀ T_real  →  T_real ≈ (J1 τ_c) / ‖J1‖²
        tau_constraint = data.qfrc_constraint[:n].copy()
        J1J1T          = float(J1 @ J1.T) + 1e-9  # scalar ‖J1‖² (evita div/0)
        T_estimated    = float(J1 @ tau_constraint) / J1J1T
        T_estimated    = max(0.0, T_estimated)   # tensão ≥ 0

        # =================================================================
        # PRIORIDADE 2: Posição do end-effector no plano XY (Z fixo)
        # =================================================================
        target_pos = np.array([target_xy[0], target_xy[1], self.fixed_z])
        pos_error  = target_pos - ee_pos           # (3,)

        # Velocidade do EE por diferença finita
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

        # Força Cartesiana PD em 3D (inclui componente Z para manter altura fixa)
        F_pos   = self.pos_kp * pos_error - self.pos_kd * ee_vel    # (3,)
        tau_pos = J2.T @ F_pos                                       # (7,)

        N2 = np.eye(n) - J2_pinv @ J2                               # (7, 7)

        # =================================================================
        # Composição MultiPrioridade:
        #   τ = τ_P1  +  N1 · τ_P2  +  τ_grav  +  N1·N2 · τ_damp
        # =================================================================
        tau = tau_tension + N1 @ tau_pos

        # Compensação de gravidade
        if self.gravity_comp:
            tau += data.qfrc_bias[:n]

        # Amortecimento no null-space combinado (N1 ∩ N2)
        tau += N1 @ N2 @ (-self.null_damping * data.qvel[:n])

        # Saturação de torque
        tau = np.clip(tau, -self.max_torque, self.max_torque)

        # ── Log / debug ──────────────────────────────────────────────────
        if verbose:
            print(f"  [t={t:.3f}s]")
            print(f"    EE pos:       {ee_pos.round(4)}")
            print(f"    Alvo:         {target_pos.round(4)}")
            print(f"    Erro pos:     {np.linalg.norm(pos_error):.4f} m")
            print(f"    Dist EE→âncora: {cable_dist:.4f} m "
                  f"(cabo: {self.cable_length:.3f} m, tenso: {cable_taut})")
            print(f"    Tensão desejada: {self.tension_desired:.1f} N")
            print(f"    Tensão estimada: {T_estimated:.2f} N")
            print(f"    Torques:         {tau.round(2)}")

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
