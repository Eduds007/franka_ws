# Franka Panda + Cable — MuJoCo Environment with MultiPriority Controller

## Demo

[![MuJoCo cable tension demo](https://img.youtube.com/vi/Eb4ri3VTJAs/0.jpg)](https://youtu.be/Eb4ri3VTJAs)

Watch on YouTube: <https://youtu.be/Eb4ri3VTJAs> — circle trajectory in the YZ plane while the multi-priority controller regulates cable tension at 10 N.

## File structure

```
franka_cable.xml              ← MuJoCo model (robot + cable + anchor)
multi_priority_controller.py  ← Two-priority controller
main.py                       ← Simulation loop + arc trajectory
```

---

## Installation

```bash
pip install mujoco numpy matplotlib
```

> Requires Python 3.8+ and MuJoCo ≥ 3.0

---

## How to run

### 1. With visualization (interactive viewer)

```bash
python main.py
```

Opens the MuJoCo 3D window with:
- Franka Panda robot (simplified geometric links)
- Orange articulated cable (7 segments) attached to the end-effector
- Blue fixed anchor in space
- Green marker = current target position

**Keys:**
| Key | Action |
|-----|--------|
| `SPACE` | Pause / resume |
| `R` | Reset simulation |
| `Q` | Quit |

### 2. Headless + plots (no viewer)

```bash
python main.py --headless 10.0
```

Runs for 10 seconds and saves `results.png` with 4 plots:
- EE trajectory XZ vs target
- Position error over time
- Cable tension over time
- X and Z coordinates vs time

---

## Environment description

### Robot
- **Franka Emika Panda** with 7 degrees of freedom
- Simplified geometries (capsules/cylinders) — no mesh files required
- Direct torque actuators on all 7 joints

### Cable
- Modeled as a **chain of 7 rigid bodies** (orange capsules)
- Each segment has **2 hinge joints** (X and Z axes) with spring and damping
- **Total length:** 7 × 0.08 m = 0.56 m (+ slack → 0.64 m in controller)
- **Connected to the end-effector** at the top
- **Connected to the anchor** at the tip via equality constraint (`<connect>`)

### Anchor
- Fixed spherical body at `[0.6, 0.0, 0.9]` m
- Cable is attached via `cable_anchor_weld` constraint

---

## MultiPriority Controller

Implements **null-space projection** (Siciliano & Slotine):

```
τ = J₁ᵀ F₁  +  (I - J₁# J₁) J₂ᵀ F₂
```

### Priority 1 (high) — Cable tension
- Measures distance between **cable tip** and **anchor**
- If cable is slack (dist < length), applies proportional force along the cable
- Parameters: `tension_kp`, `tension_desired`

### Priority 2 (low) — End-effector position
- **PD** control in Cartesian space
- Moves EE to target `[x, z]` in XZ plane
- Projected onto **P1 null-space** → does not disturb tension
- Parameters: `pos_kp`, `pos_kd`

---

## Arc trajectory

The default trajectory oscillates in an arc in the XZ plane:

```python
ARC_CENTER_X  = 0.45   # center X
ARC_CENTER_Z  = 0.65   # center Z
ARC_RADIUS    = 0.18   # radius
ARC_ANGLE_START = -60° # start angle
ARC_ANGLE_END   = +60° # end angle
ARC_PERIOD    = 8.0    # period in seconds
```

To modify, edit `main.py` → "Global settings" section.

---

## Parameter tuning

| Parameter | File | Effect |
|-----------|------|--------|
| `tension_desired` | `main.py` | Target cable tension (N) |
| `tension_kp` | `main.py` | Tension responsiveness |
| `pos_kp / pos_kd` | `main.py` | Position tracking gains |
| `cable_length` | `main.py` | Nominal cable length |
| `ANCHOR_POS` | `main.py` | Anchor position (must match XML) |
| `ARC_*` | `main.py` | Trajectory parameters |

---

## Integrating your own controller

To replace the example controller with your `MultiPriorityController`:

1. Implement a class with method `compute(model, data, target_xz) -> (tau, info)`
2. Replace the import in `main.py`:
   ```python
   from my_controller import MyController as MultiPriorityController
   ```
3. The environment (`franka_cable.xml`) remains the same

### Environment API (available data)

```python
# End-effector position
ee_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
ee_pos = data.site_xpos[ee_id]    # array (3,)

# Cable tip position
tip_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cable_tip_site")
tip_pos = data.site_xpos[tip_id]   # array (3,)

# Anchor position
anc_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "anchor_site")
anc_pos = data.site_xpos[anc_id]   # array (3,)

# End-effector Jacobian
J = np.zeros((6, model.nv))
mujoco.mj_jacSite(model, data, J[:3], J[3:], ee_id)
J_robot = J[:, :7]   # robot joints only

# Joint state
q    = data.qpos[:7]   # positions (rad)
qdot = data.qvel[:7]   # velocities (rad/s)

# Gravity compensation
g_comp = data.qfrc_bias[:7]

# Apply torques
data.ctrl[:7] = tau   # array (7,)
```

---

## Technical notes

- MuJoCo resolves the anchor constraint internally → cable **does not pass through objects**
- Cable joint stiffness (`stiffness="0.5"`) simulates cable rigidity
- For stiffer cable: increase `stiffness` and reduce `damping` in cable joints in XML
- For more flexible/heavier cable: reduce `stiffness` and add `density` to cable geoms
- Timestep is `0.002s` (500 Hz) — appropriate for the system dynamics
