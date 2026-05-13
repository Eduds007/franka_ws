import mujoco
import mujoco.viewer
import numpy as np

from robot_descriptions.loaders.mujoco import load_robot_description

model = load_robot_description("panda_mj_description")
data = mujoco.MjData(model)
SIM_GRAVITY = np.array([0.0, 0.0, -9.81])
COMPENSATE_GRAVITY = False
model.opt.gravity[:] = SIM_GRAVITY

EE_BODY_NAME = "hand"

ee_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    EE_BODY_NAME
)

if ee_id == -1:
    print(f"Error: Body '{EE_BODY_NAME}' not found!")
    exit(1)

step = 0
dt = float(model.opt.timestep)
prev_J = None
prev_q_dot = None
Kp_pos = np.diag([120.0, 120.0, 120.0])
Kd_pos = np.diag([24.0, 24.0, 24.0])
x_des = np.array([0.55, 0.5, 0.0])
F_des = np.array([0.0, 0.0, -8.0])
ENABLE_FORCE_SIM = True
F_sim = np.array([0.0, 0.0, -6.0])
FORCE_Z_REF = 0.0
FORCE_Z_DECAY = 2.0
lambda_pinv = 1e-4
Md_force = np.diag([1.5, 1.5, 1.5])
Bd_force = np.diag([80.0, 80.0, 80.0])
Kd_force = np.diag([120.0, 120.0, 120.0])
e_force = np.zeros(3)
e_dot_force = np.zeros(3)
Md_imp = 1.5 * np.eye(model.nv)
Bd_imp = 80.0 * np.eye(model.nv)
Kd_imp = 120.0 * np.eye(model.nv)


def draw_marker_sphere(viewer, pos, rgba, radius=0.02):
    if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
        return
    geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
    viewer.user_scn.ngeom += 1
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([radius, 0.0, 0.0]),
        pos.astype(float),
        np.eye(3).reshape(-1),
        np.array(rgba, dtype=float),
    )


def multi_priority_controller(model, data, ee_id, state):
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, jacr, ee_id)
    J = np.vstack((jacp, jacr))
    x = data.xpos[ee_id].copy()
    q = data.qpos.copy()
    q_dot = data.qvel.copy()
    e = state["q_d"] - q
    q_dot_d = np.zeros_like(q_dot)
    q_dot_dot_d = np.zeros_like(q_dot)
    e_dot = q_dot_d - q_dot
    x_dot = J @ q_dot
    x_dot_pos = jacp @ q_dot
    F_ext_measured = data.cfrc_ext[ee_id, 3:6].copy()
    F_sim_current = np.zeros(3)
    tau_sim = np.zeros(model.nv)
    if state["enable_force_sim"]:
        z = float(x[2])
        z_delta = max(0.0, z - state["force_z_ref"])
        force_scale = 1.0 / (1.0 + state["force_z_decay"] * z_delta)
        F_sim_current = state["F_sim_base"] * force_scale
        tau_sim = jacp.T @ F_sim_current
    F_ext = F_ext_measured + F_sim_current
    F_err = state["F_d"] - F_ext
    wrench_ext = data.cfrc_ext[ee_id].copy()
    tau_ext = J.T @ wrench_ext
    e_dot_dot_force = np.linalg.solve(
        Md_force,
        F_err - Bd_force @ state["e_dot_force"] - Kd_force @ state["e_force"],
    )
    state["e_dot_force"] = state["e_dot_force"] + e_dot_dot_force * dt
    state["e_force"] = state["e_force"] + state["e_dot_force"] * dt
    x_d = state["x_des"] + state["e_force"]
    x_dot_d = state["e_dot_force"]
    x_dot_dot_d = e_dot_dot_force

    if state["prev_J"] is None:
        J_dot = np.zeros_like(J)
    else:
        J_dot = (J - state["prev_J"]) / dt

    if state["prev_q_dot"] is None:
        q_dot_dot = np.zeros_like(q_dot)
    else:
        q_dot_dot = (q_dot - state["prev_q_dot"]) / dt

    x_dot_dot = J @ q_dot_dot + J_dot @ q_dot
    x_dot_dot_c = x_dot_dot_d + Kd_pos @ (x_dot_d - x_dot_pos) + Kp_pos @ (x_d - x)
    J_pos = jacp
    J_dot_pos = J_dot[:3, :]
    J_dot_q_dot_pos = J_dot_pos @ q_dot
    J_pos_pinv = J_pos.T @ np.linalg.solve(
        J_pos @ J_pos.T + (lambda_pinv ** 2) * np.eye(3),
        np.eye(3),
    )
    I_nv = np.eye(model.nv)
    N = I_nv - J_pos_pinv @ J_pos
    q_dot_dot_1 = J_pos_pinv @ (x_dot_dot_c - J_dot_q_dot_pos)
    q_dot_dot_imp = q_dot_dot_d + np.linalg.solve(
        Md_imp,
        Bd_imp @ e_dot + Kd_imp @ e - tau_ext,
    )
    q_dot_dot_2 = N @ q_dot_dot_imp
    q_dot_dot_cmd = q_dot_dot_1 + q_dot_dot_2
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, M, data.qM)
    bias = data.qfrc_bias.copy()
    if state["compensate_gravity"]:
        tau = M @ q_dot_dot_cmd + bias + tau_sim
    else:
        tau = M @ q_dot_dot_cmd + tau_sim

    if model.nu > 0:
        data.ctrl[:] = tau[:model.nu]
    else:
        data.qfrc_applied[:] = tau

    print(
        f"Passo {state['step']} | x_dot = {np.round(x_dot, 4)} | "
        f"x_dot_dot = {np.round(x_dot_dot, 4)} | "
        f"x_dot_dot_c = {np.round(x_dot_dot_c, 4)} | "
        f"q_dot_dot_1 = {np.round(q_dot_dot_1, 4)} | "
        f"q_dot_dot_imp = {np.round(q_dot_dot_imp, 4)} | "
        f"q_dot_dot_2 = {np.round(q_dot_dot_2, 4)} | "
        f"q_dot_dot = {np.round(q_dot_dot_cmd, 4)} | "
        f"|tau| = {np.linalg.norm(tau):.4f} | "
        f"g_comp = {state['compensate_gravity']} | "
        f"F_ext = {np.round(F_ext, 4)} | "
        f"F_sim = {np.round(F_sim_current, 4)} | "
        f"F_err = {np.round(F_err, 4)} | "
        f"|x-x_des| = {np.linalg.norm(x - state['x_des']):.4f} | "
        f"||N|| = {np.linalg.norm(N):.4f} | "
        f"||e|| = {np.linalg.norm(e):.4f}"
    )

    state["prev_J"] = J.copy()
    state["prev_q_dot"] = q_dot.copy()
    state["step"] += 1

with mujoco.viewer.launch_passive(model, data) as viewer:
    mujoco.mj_forward(model, data)
    controller_state = {
        "step": step,
        "prev_J": prev_J,
        "prev_q_dot": prev_q_dot,
        "q_d": data.qpos.copy(),
        "x_des": x_des.copy(),
        "F_d": F_des.copy(),
        "enable_force_sim": ENABLE_FORCE_SIM,
        "compensate_gravity": COMPENSATE_GRAVITY,
        "F_sim_base": F_sim.copy(),
        "force_z_ref": FORCE_Z_REF,
        "force_z_decay": FORCE_Z_DECAY,
        "e_force": e_force.copy(),
        "e_dot_force": e_dot_force.copy(),
    }

    while viewer.is_running():
        multi_priority_controller(model, data, ee_id, controller_state)

        viewer.user_scn.ngeom = 0
        draw_marker_sphere(viewer, controller_state["x_des"], [1.0, 0.1, 0.1, 0.9], radius=0.02)

        mujoco.mj_step(model, data)

        viewer.sync()