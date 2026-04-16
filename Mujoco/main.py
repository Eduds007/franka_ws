"""
main.py
=======
Simulação principal: Franka Panda + cabo + Controlador MultiPrioridade.

Executa uma trajetória em arco no plano XY (Z fixo = 0.4m) enquanto
mantém o cabo tensionado em ~10N.

Geometria:
  - Robô: base em (0, 0, 0)
  - Âncora (parede): (0.8, 0, 0.4)
  - Cabo: 5 elos × 0.1m = 0.5m
  - Arco: raio 0.5m, centro na âncora, 120° de varredura no plano XY
    Percurso: (0.55, 0.43) → (0.30, 0.00) → (0.55, -0.43) @ Z=0.4m

Uso:
    python main.py                      # modo interativo (viewer)
    python main.py --headless 40.0      # headless + plots + CSV

Teclas no viewer:
    ESPAÇO  — pausar / continuar
    R       — reiniciar trajetória
    Q       — sair

Dependências:
    pip install mujoco numpy matplotlib
"""

import os
import sys
import csv
import time
import datetime
import numpy as np
import mujoco
import mujoco.viewer

from multi_priority_controller import MultiPriorityController

# ---------------------------------------------------------------------------
# Configurações globais
# ---------------------------------------------------------------------------

XML_PATH = "franka_cable.xml"

# Posição inicial do robô: "home" do Panda sem garra
# Posiciona o EE em ≈(0.307, 0, 0.487) — a ~0.5m da âncora (0.8, 0, 0.4)
QPOS_INIT_ROBOT = np.array([0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, -0.7853])

# Âncora (deve bater com o XML)
ANCHOR_POS = np.array([0.8, 0.0, 0.4])

# Parâmetros do cabo
CABLE_LENGTH = 0.50    # 5 elos × 0.1m

# Parâmetros da trajetória em arco (plano XY, Z fixo)
ARC_CENTER_X    = 0.80   # centro X = âncora X
ARC_CENTER_Y    = 0.00   # centro Y = âncora Y
ARC_RADIUS      = 0.50   # raio = comprimento do cabo
ARC_Z_FIXED     = 0.40   # altura Z fixa (= altura da âncora)
ARC_ANGLE_START = np.deg2rad(120.0)   # 120° — (0.55,  0.43)
ARC_ANGLE_END   = np.deg2rad(240.0)   # 240° — (0.55, -0.43)
ARC_PERIOD      = 20.0   # período de um ciclo (s) — movimento lento

# Parâmetros do controlador
TENSION_DESIRED = 10.0   # tensão alvo no cabo [N]
POS_KP          = 150.0  # ganho proporcional de posição
POS_KD          = 15.0   # ganho derivativo de posição
NULL_DAMPING    = 5.0    # amortecimento no null-space

# Logging
LOG_EVERY_N_STEPS = 10   # salva no CSV a cada N steps de simulação
PRINT_EVERY_N_STEPS = 200  # imprime no terminal a cada N steps


# ---------------------------------------------------------------------------
# Trajetória em arco — plano XY
# ---------------------------------------------------------------------------

def arc_trajectory(t: float) -> np.ndarray:
    """
    Retorna a posição [x, y] do alvo no plano XY para o instante t.

    O arco oscila suavemente entre ARC_ANGLE_START (120°) e ARC_ANGLE_END (240°),
    passando pelo ponto mais próximo do robô (θ=180°, EE em (0.30, 0.00, 0.40)).

    Percurso:
      t=0s:    (0.55,  0.43, 0.40)  — upper-left
      t=5s:    (0.30,  0.00, 0.40)  — centro (mais próximo do robô)
      t=10s:   (0.55, -0.43, 0.40)  — lower-left
      t=15s:   (0.30,  0.00, 0.40)  — centro
      t=20s:   (0.55,  0.43, 0.40)  — upper-left (ciclo completo)
    """
    # Interpolação sinusoidal 0→1 (suave, sem descontinuidade de velocidade)
    frac  = (np.sin(2.0 * np.pi * t / ARC_PERIOD - np.pi / 2.0) + 1.0) / 2.0
    angle = ARC_ANGLE_START + frac * (ARC_ANGLE_END - ARC_ANGLE_START)
    x = ARC_CENTER_X + ARC_RADIUS * np.cos(angle)
    y = ARC_CENTER_Y + ARC_RADIUS * np.sin(angle)
    return np.array([x, y])


# ---------------------------------------------------------------------------
# Reset da simulação
# ---------------------------------------------------------------------------

def _find_initial_qpos(model: mujoco.MjModel, data: mujoco.MjData,
                       target_ee: np.ndarray,
                       q0: np.ndarray,
                       max_iter: int = 500,
                       step: float  = 0.05,
                       tol: float   = 0.02) -> np.ndarray:
    """IK numérico (gradient descent) para posicionar o EE próximo a target_ee."""
    from multi_priority_controller import get_ee_jacobian
    q = q0.copy()
    ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    jnt_lo = model.jnt_range[:7, 0]
    jnt_hi = model.jnt_range[:7, 1]

    for _ in range(max_iter):
        data.qpos[:7] = q
        mujoco.mj_forward(model, data)
        ee = data.site_xpos[ee_id].copy()
        err = target_ee - ee
        if np.linalg.norm(err) < tol:
            break
        J = get_ee_jacobian(model, data)[:3, :]   # (3, 7)
        # Pseudo-inversa simples com amortecimento
        JJT = J @ J.T
        J_pinv = J.T @ np.linalg.inv(JJT + 1e-4 * np.eye(3))
        dq = J_pinv @ err
        q  = np.clip(q + step * dq, jnt_lo, jnt_hi)

    return q


def reset_simulation(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Reseta a simulação para a posição inicial.

    Usa IK rápido para posicionar o EE próximo ao ponto médio do arco
    (θ=180°, EE em (0.3, 0, 0.4)) antes de iniciar o controle.
    """
    mujoco.mj_resetData(model, data)

    # Ponto alvo para IK: meio do arco (ponto mais próximo do robô)
    arc_mid = np.array([
        ARC_CENTER_X + ARC_RADIUS * np.cos(np.pi),   # = 0.8 - 0.5 = 0.3
        ARC_CENTER_Y + ARC_RADIUS * np.sin(np.pi),   # = 0.0
        ARC_Z_FIXED,                                  # = 0.4
    ])

    q_init = _find_initial_qpos(model, data, arc_mid, QPOS_INIT_ROBOT.copy())
    data.qpos[:7] = q_init
    mujoco.mj_forward(model, data)


# ---------------------------------------------------------------------------
# Atualização do marcador visual do alvo
# ---------------------------------------------------------------------------

def update_target_marker(model: mujoco.MjModel, data: mujoco.MjData,
                         xy: np.ndarray) -> None:
    """Move o marcador visual verde para a posição alvo atual."""
    body_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_marker")
    if body_id < 0:
        return
    mocap_id = model.body_mocapid[body_id]
    if mocap_id >= 0:
        data.mocap_pos[mocap_id] = [xy[0], xy[1], ARC_Z_FIXED]


# ---------------------------------------------------------------------------
# Logger CSV
# ---------------------------------------------------------------------------

class SimLogger:
    """Salva dados da simulação em um arquivo CSV."""

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
        print(f"[LOG] Salvando dados em: {filepath}")

    def write(self, t: float, info: dict, target_xy: np.ndarray) -> None:
        ee = info["ee_pos"]
        self._writer.writerow({
            "time":          f"{t:.4f}",
            "ee_x":          f"{ee[0]:.5f}",
            "ee_y":          f"{ee[1]:.5f}",
            "ee_z":          f"{ee[2]:.5f}",
            "tgt_x":         f"{target_xy[0]:.5f}",
            "tgt_y":         f"{target_xy[1]:.5f}",
            "tension_N":     f"{info['tension_estimated']:.4f}",
            "pos_error_m":   f"{info['pos_error']:.5f}",
            "cable_taut":    int(info["cable_taut"]),
            "cable_dist_m":  f"{info['cable_dist']:.5f}",
        })

    def close(self) -> None:
        self._file.close()
        print(f"[LOG] Arquivo fechado: {self.filepath}")


# ---------------------------------------------------------------------------
# Loop principal — modo interativo
# ---------------------------------------------------------------------------

def run_simulation() -> None:
    """Executa a simulação com viewer interativo."""
    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
    except Exception as e:
        print(f"[ERRO] Não foi possível carregar '{XML_PATH}': {e}")
        return

    data = mujoco.MjData(model)

    ctrl = MultiPriorityController(
        cable_length    = CABLE_LENGTH,
        tension_desired = TENSION_DESIRED,
        pos_kp          = POS_KP,
        pos_kd          = POS_KD,
        fixed_z         = ARC_Z_FIXED,
        null_damping    = NULL_DAMPING,
        gravity_comp    = True,
    )

    reset_simulation(model, data)
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
            status = "PAUSADA" if state["paused"] else "RODANDO"
            print(f"[INFO] Simulação {status}")
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

        while viewer.is_running() and not state["quit"]:

            if state["reset"]:
                reset_simulation(model, data)
                ctrl.reset()
                state["step_count"] = 0
                state["reset"] = False
                print("[INFO] Simulação reiniciada.")

            if not state["paused"]:
                t          = data.time
                target_xy  = arc_trajectory(t)
                step_count = state["step_count"]

                verbose = (step_count % PRINT_EVERY_N_STEPS == 0)
                tau, info = ctrl.compute(model, data, target_xy, verbose=verbose)

                if verbose:
                    print(f"  [t={t:.1f}s] Alvo: ({target_xy[0]:.3f}, {target_xy[1]:.3f}) | "
                          f"Erro: {info['pos_error']:.4f}m | "
                          f"Tensão: {info['tension_estimated']:.1f}N | "
                          f"Tenso: {info['cable_taut']}")

                # Aplica torques e avança simulação
                data.ctrl[:7] = tau
                mujoco.mj_step(model, data)

                # Atualiza marcador visual
                update_target_marker(model, data, target_xy)

                # Logging
                if step_count % LOG_EVERY_N_STEPS == 0:
                    logger.write(t, info, target_xy)

                state["step_count"] += 1

            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - 0.001))

    logger.close()
    print("[INFO] Simulação encerrada.")


# ---------------------------------------------------------------------------
# Modo headless — simulação + plots
# ---------------------------------------------------------------------------

def run_headless_and_plot(duration: float = 40.0) -> None:
    """
    Executa a simulação sem viewer, gera plots e salva CSV.

    Uso:
        python main.py --headless 40.0
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[AVISO] matplotlib não instalado. Instale com: pip install matplotlib")
        return

    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
    except Exception as e:
        print(f"[ERRO] Não foi possível carregar '{XML_PATH}': {e}")
        return

    data = mujoco.MjData(model)

    ctrl = MultiPriorityController(
        cable_length    = CABLE_LENGTH,
        tension_desired = TENSION_DESIRED,
        pos_kp          = POS_KP,
        pos_kd          = POS_KD,
        fixed_z         = ARC_Z_FIXED,
        null_damping    = NULL_DAMPING,
        gravity_comp    = True,
    )

    reset_simulation(model, data)
    logger = SimLogger()

    # Buffers para plots
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

    print(f"[INFO] Rodando {duration}s ({steps} steps @ {1/dt:.0f}Hz) ...")
    _print_header()

    for i in range(steps):
        t         = data.time
        target_xy = arc_trajectory(t)

        tau, info = ctrl.compute(model, data, target_xy)
        data.ctrl[:7] = tau
        mujoco.mj_step(model, data)

        # Log CSV a cada LOG_EVERY_N_STEPS
        if i % LOG_EVERY_N_STEPS == 0:
            logger.write(t, info, target_xy)

        # Buffer para plots (a cada 50 steps ≈ 0.1s)
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
                  f"Alvo: ({target_xy[0]:.3f}, {target_xy[1]:.3f}) | "
                  f"Tensão: {info['tension_estimated']:.1f}N | "
                  f"Err: {info['pos_error']:.4f}m")

    logger.close()

    # ── Plots ────────────────────────────────────────────────────────────
    print("[INFO] Gerando plots ...")

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle("Franka Panda + Cabo — Controlador MultiPrioridade\n"
                 f"P1: Tensão ≈ {TENSION_DESIRED}N  |  P2: Arco 120° no plano XY",
                 fontsize=12, fontweight="bold")

    # ── Plot 1: Trajetória no plano XY ───────────────────────────────────
    ax = axes[0, 0]
    ax.plot(log_tgt_x, log_tgt_y, "g--", linewidth=2.0, label="Alvo (arco)")
    ax.plot(log_ee_x,  log_ee_y,  "b-",  linewidth=1.5, label="EE real",  alpha=0.8)
    ax.scatter([ANCHOR_POS[0]], [ANCHOR_POS[1]], c="orange", s=120,
               zorder=5, label=f"Âncora ({ANCHOR_POS[0]}, {ANCHOR_POS[1]})")
    ax.scatter([0.0], [0.0], c="red", s=80, marker="^",
               zorder=5, label="Base do robô")
    # Desenha arco de referência
    theta_ref = np.linspace(ARC_ANGLE_START, ARC_ANGLE_END, 100)
    ax.plot(ARC_CENTER_X + ARC_RADIUS * np.cos(theta_ref),
            ARC_CENTER_Y + ARC_RADIUS * np.sin(theta_ref),
            "g:", linewidth=1.0, alpha=0.5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajetória EE — Plano XY")
    ax.legend(fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # ── Plot 2: Erro de posição ───────────────────────────────────────────
    ax = axes[0, 1]
    ax.plot(log_t, log_error, "r-", linewidth=1.5)
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Erro (m)")
    ax.set_title("Erro de Posição do End-Effector")
    ax.grid(True, alpha=0.3)

    # ── Plot 3: Tensão no cabo ────────────────────────────────────────────
    ax = axes[1, 0]
    ax.plot(log_t, log_tension, color="darkorange", linewidth=1.5,
            label="Tensão estimada")
    ax.axhline(TENSION_DESIRED, color="k", linestyle="--", linewidth=1.2,
               label=f"Tensão desejada ({TENSION_DESIRED:.0f} N)")
    ax.axhline(0.0, color="gray", linestyle=":", linewidth=0.8)
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Tensão (N)")
    ax.set_title("Tensão no Cabo")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=-1.0)

    # ── Plot 4: X e Y do EE vs tempo ─────────────────────────────────────
    ax = axes[1, 1]
    ax.plot(log_t, log_ee_x,  "b-",  linewidth=1.5, label="EE X real")
    ax.plot(log_t, log_tgt_x, "b--", linewidth=1.0, label="EE X alvo", alpha=0.7)
    ax.plot(log_t, log_ee_y,  "r-",  linewidth=1.5, label="EE Y real")
    ax.plot(log_t, log_tgt_y, "r--", linewidth=1.0, label="EE Y alvo", alpha=0.7)
    ax.plot(log_t, log_ee_z,  "g-",  linewidth=1.0, label="EE Z real", alpha=0.6)
    ax.axhline(ARC_Z_FIXED, color="g", linestyle=":", linewidth=0.8,
               label=f"Z alvo ({ARC_Z_FIXED}m)", alpha=0.6)
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Posição (m)")
    ax.set_title("Coordenadas EE vs Tempo")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = "resultados.png"
    plt.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.show()
    print(f"[INFO] Plot salvo em '{plot_path}'")


# ---------------------------------------------------------------------------
# Utilitário
# ---------------------------------------------------------------------------

def _print_header() -> None:
    print("=" * 65)
    print("  Franka Panda + Cabo — Controlador MultiPrioridade (plano XY)")
    print("=" * 65)
    print(f"  Âncora:         {ANCHOR_POS}")
    print(f"  Centro arco XY: ({ARC_CENTER_X:.2f}, {ARC_CENTER_Y:.2f})  Z fixo: {ARC_Z_FIXED}m")
    print(f"  Raio arco:      {ARC_RADIUS} m")
    print(f"  Varredura:      120° (θ = 120° → 240°)")
    print(f"  Período:        {ARC_PERIOD} s")
    print(f"  Tensão alvo:    {TENSION_DESIRED} N")
    print("-" * 65)
    print("  ESPAÇO → pausar/continuar  |  R → reiniciar  |  Q → sair")
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
