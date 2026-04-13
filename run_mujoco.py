import mujoco
import mujoco.viewer
import numpy as np

from robot_descriptions.loaders.mujoco import load_robot_description

# =========================
# 1. Carregar robô
# =========================
# model: descrição da geometria, dinâmica e restrições do robô
# data: estado atual da simulação (posições, velocidades, forças)
model = load_robot_description("panda_mj_description")
data = mujoco.MjData(model)

# =========================
# 2. Identificar end-effector
# =========================
EE_BODY_NAME = "hand"  # end-effector do Panda

ee_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    EE_BODY_NAME
)

if ee_id == -1:
    print(f"Erro: Body '{EE_BODY_NAME}' não encontrado!")
    exit(1)

# =========================
# 3. Parâmetros
# =========================

# posição desejada (XYZ controlado)
# Unidades: [m, m, m] (metros)
x_des = np.array([-0.5, -0.5, 0.5])  # Posição mais realista

# força desejada (empurrar pra baixo)
# Unidades: [N, N, N] (Newtons)
F_des = np.array([0, 0, 0])

# ganhos de controle - AJUSTADOS para estabilidade
# Kp: [N/m] - converte erro de posição em força
# Kd: [N·s/m] - converte velocidade em força (amortecimento)
# Kf: [1/1] - adimensional - escala de compensação de força
Kp = 100  # Aumentado para melhor resposta (força/metro)
Kd = 20   # Aumentado para amortecimento adequado
Kf = 0.1  # Reduzido para evitar instabilidade

# matriz de seleção: posição em XYZ (PID puro)
# [1, 1, 1] significa: controlar posição em X,Y,Z
S = np.diag([1, 1, 1])  # Mudado para controle de posição puro
I = np.eye(3)

# damping pseudo-inversa
# Unidades: [1/(N²)] - previne singularidades no Jacobiano
lambda_ = 1e-3  # Aumentado para melhor estabilidade numérica

# =========================
# 4. Função de controle
# =========================
def controller(model, data):
    # ----- estado EE -----
    # x: posição do end-effector [m, m, m] (metros em X, Y, Z)
    x = data.xpos[ee_id]  # posição do body
    
    # ----- Jacobiano -----
    # jacp: Jacobiano linear (posição) [m/rad] 
    #       shape: (3, nv) - mapeia velocidade das juntas para velocidade linear
    # jacr: Jacobiano angular (rotação) [rad/rad]
    #       shape: (3, nv) - mapeia velocidade das juntas para velocidade angular
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, jacr, ee_id)
    J = jacp
    
    # dx: velocidade linear do end-effector [m/s]
    #     calculada como: J @ qvel onde qvel tem unidades [rad/s]
    #     resultado: [m/s]
    dx = J @ data.qvel

    # ----- controle de posição PID simples -----
    # F_pos: força de controle proporcional-derivativo [N, N, N]
    # Kp * (x_des - x): [N/m] * [m] = [N]
    # Kd * dx: [N·s/m] * [m/s] = [N]
    F_pos = Kp * (x_des - x) - Kd * dx
    
    # Print do erro de posição [m]
    print(f"Erro de posição (x - x_des): {x - x_des} [m]")
    print(f"Força PID: {F_pos} [N]")

    # ----- torque final (sem estimação de força externa) -----
    # J.T @ F_pos: transposto do Jacobiano (3x9) @ força (3)
    # resultado: torque das juntas [N·m]
    tau_cmd = J.T @ F_pos

    # ajustar tamanho
    # model.nu: número de atuadores
    tau_cmd = tau_cmd[:model.nu]

    # limitar torque para segurança
    # Unidades: [N·m] - limitado entre -100 e 100 N·m
    tau_cmd = np.clip(tau_cmd, -100, 100)

    # aplicar comando de torque aos atuadores
    # data.ctrl: sinal de controle [N·m]
    data.ctrl[:] = tau_cmd

# =========================
# 5. Simulação
# =========================
# Loop de simulação em tempo real
# - controller(): calcula comando de torque baseado no estado atual
# - mujoco.mj_step(): integra as equações dinâmicas (passo de tempo: padrão 0.002s)
# - viewer.sync(): renderiza a posição atual do robô
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():

        controller(model, data)

        # Integra um passo de simulação [s]
        # Tempo de integração: typically 0.002s (2ms)
        mujoco.mj_step(model, data)

        viewer.sync()