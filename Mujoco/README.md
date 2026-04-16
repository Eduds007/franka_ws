# Franka Panda + Cabo — Ambiente MuJoCo com Controlador MultiPrioridade

## Estrutura dos arquivos

```
franka_cable.xml              ← Modelo MuJoCo (robô + cabo + âncora)
multi_priority_controller.py  ← Controlador de duas prioridades
main.py                       ← Loop de simulação + trajetória em arco
```

---

## Instalação

```bash
pip install mujoco numpy matplotlib
```

> Requer Python 3.8+ e MuJoCo ≥ 3.0

---

## Como rodar

### 1. Com visualização (viewer interativo)

```bash
python main.py
```

Abre a janela 3D do MuJoCo com:
- Robô Franka Panda (links geométricos simplificados)
- Cabo articulado laranja (7 segmentos) ligado ao end-effector
- Âncora azul fixa no espaço
- Marcador verde = posição alvo atual

**Teclas:**
| Tecla | Ação |
|-------|------|
| `ESPAÇO` | Pausar / continuar |
| `R` | Reiniciar simulação |
| `Q` | Sair |

### 2. Headless + gráficos (sem viewer)

```bash
python main.py --headless 10.0
```

Roda por 10 segundos e salva `resultados.png` com 4 gráficos:
- Trajetória XZ do end-effector vs alvo
- Erro de posição ao longo do tempo
- Tensão no cabo ao longo do tempo
- Coordenadas X e Z vs tempo

---

## Descrição do ambiente

### Robô
- **Franka Emika Panda** com 7 graus de liberdade
- Geometrias simplificadas (capsules/cylinders) — não requer arquivos de mesh
- Atuadores torque direto nos 7 joints

### Cabo
- Modelado como **cadeia de 7 corpos rígidos** (capsules laranja)
- Cada segmento tem **2 joints tipo hinge** (eixo X e eixo Z) com mola e amortecimento
- **Comprimento total:** 7 × 0.08 m = 0.56 m (+ folga → 0.64 m no controlador)
- **Conectado ao end-effector** pelo topo
- **Conectado à âncora** pela ponta via constraint de igualdade (`<connect>`)

### Âncora
- Corpo esférico fixo em `[0.6, 0.0, 0.9]` m
- O cabo é preso a ela via constraint `cable_anchor_weld`

---

## Controlador MultiPrioridade

Implementa **null-space projection** (Siciliano & Slotine):

```
τ = J₁ᵀ F₁  +  (I - J₁# J₁) J₂ᵀ F₂
```

### Prioridade 1 (alta) — Tensão no cabo
- Mede a distância entre a **ponta do cabo** e a **âncora**
- Se o cabo estiver frouxo (dist < comprimento), aplica força proporcional ao longo do cabo
- Parâmetros: `tension_kp`, `tension_desired`

### Prioridade 2 (baixa) — Posição do end-effector
- Controle **PD** no espaço Cartesiano
- Move o EE para o alvo `[x, z]` no plano XZ
- Projetado no **null-space de P1** → não perturba a tensão
- Parâmetros: `pos_kp`, `pos_kd`

---

## Trajetória em arco

A trajetória padrão oscila em arco no plano XZ:

```python
ARC_CENTER_X  = 0.45   # centro X
ARC_CENTER_Z  = 0.65   # centro Z
ARC_RADIUS    = 0.18   # raio
ARC_ANGLE_START = -60° # ângulo inicial
ARC_ANGLE_END   = +60° # ângulo final
ARC_PERIOD    = 8.0    # período em segundos
```

Para modificar, edite `main.py` → seção "Configurações Globais".

---

## Ajuste de parâmetros

| Parâmetro | Arquivo | Efeito |
|-----------|---------|--------|
| `tension_desired` | `main.py` | Tensão alvo no cabo (N) |
| `tension_kp` | `main.py` | Responsividade da tensão |
| `pos_kp / pos_kd` | `main.py` | Ganhos do rastreamento de posição |
| `cable_length` | `main.py` | Comprimento nominal do cabo |
| `ANCHOR_POS` | `main.py` | Posição da âncora (deve bater com o XML) |
| `ARC_*` | `main.py` | Parâmetros da trajetória |

---

## Integração com seu próprio controlador

Para substituir o controlador de exemplo pelo seu `MultiPriorityController`:

1. Implemente uma classe com método `compute(model, data, target_xz) -> (tau, info)`
2. Substitua a importação em `main.py`:
   ```python
   from meu_controlador import MeuControlador as MultiPriorityController
   ```
3. O ambiente (`franka_cable.xml`) permanece o mesmo

### API do ambiente (dados disponíveis)

```python
# Posição do end-effector
ee_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
ee_pos = data.site_xpos[ee_id]    # array (3,)

# Posição da ponta do cabo
tip_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cable_tip_site")
tip_pos = data.site_xpos[tip_id]   # array (3,)

# Posição da âncora
anc_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "anchor_site")
anc_pos = data.site_xpos[anc_id]   # array (3,)

# Jacobiano do end-effector
J = np.zeros((6, model.nv))
mujoco.mj_jacSite(model, data, J[:3], J[3:], ee_id)
J_robot = J[:, :7]   # apenas os 7 joints do robô

# Estado dos joints
q    = data.qpos[:7]   # posições (rad)
qdot = data.qvel[:7]   # velocidades (rad/s)

# Compensação de gravidade
g_comp = data.qfrc_bias[:7]

# Aplicar torques
data.ctrl[:7] = tau   # array (7,)
```

---

## Notas técnicas

- O MuJoCo resolve a constraint da âncora internamente → o cabo **não atravessa objetos**
- A stiffness dos joints do cabo (`stiffness="0.5"`) simula a rigidez do cabo
- Para cabo mais rígido: aumente `stiffness` e reduza `damping` nos joints do cabo no XML
- Para cabo mais flexível/pesado: reduza `stiffness` e adicione `density` às geoms do cabo
- O timestep é `0.002s` (500 Hz) — adequado para a dinâmica do sistema
