# Franka Cable Manipulation Workspace

Research workspace for cable manipulation with a **Franka Panda 7-DOF arm**. Combines real-time robot control via ROS2/ros2_control, tactile force sensing from GelSight cameras (FEATS neural network), and MuJoCo physics simulation for offline validation of multi-priority control strategies.

## Architecture Overview

```
GelSight Cameras
      │
      ▼
FEATS UNet (PyTorch)          ←── tactile_cameras / tension_estimator
      │ force estimates
      ▼
Multi-Priority Controller     ←── tension_control (C++ plugin, 1 kHz)
      │ joint torques
      ▼
FrankaHardwareInterface       ←── franka_ros2 / libfranka
      │
      ▼
Franka Panda Robot
```

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Middleware | ROS2 Humble, ros2_control, MoveIt2 |
| Real-time control | C++17, libfranka 0.12.1 |
| Research / ML | Python 3, PyTorch |
| Simulation | MuJoCo 3.x |
| Build | colcon, ament_cmake, ament_python |

## Repository Structure

```
franka_ws/
├── src/
│   ├── franka_ros2/            # Official Franka ROS2 packages (submodule)
│   ├── libfranka/              # Low-level Franka FCI C++ library (submodule)
│   ├── tension_control/        # Custom admittance & multi-priority controllers
│   ├── tension_estimator/      # GelSight → FEATS inference → force estimates
│   ├── tactile_cameras/        # GelSight drivers, marker tracking, cable pose
│   ├── feats/                  # FEATS UNet model weights and utilities
│   └── franka_moveit_example/  # MoveIt2 Python scripting examples
├── Mujoco/                     # MuJoCo simulation assets
├── scripts/                    # Motion primitive scripts (detect cable, circular motion)
├── run_mujoco.py               # Standalone MuJoCo simulation entry point
├── interactive_joint_commander.py  # Interactive CLI for manual joint control
├── publish_trajectory.py       # Trajectory publisher utility
├── publish_joint_command.py    # Joint command publisher utility
└── cable_pose_estimation_demo.ipynb  # Cable pose estimation demo notebook
```

### Key Packages

**`tension_control`** — Core custom controller package.
- `admittance_controller.cpp` — Impedance/admittance control plugin (125 Hz)
- `multi_priority_controller.cpp` — Multi-task priority framework (cable tension + EE pose)
- `tactile_detection_service.py` — Cable detection state machine via GelSight

**`tactile_cameras`** — GelSight camera integration and cable pose estimation.
- `cable_pose_estimator.py` — Cable pose from tactile contact geometry
- `gelsight_object_detection.py` — Object detection on GelSight images
- `gelsight_marker_detector.py` — Marker-based deformation tracking

**`tension_estimator`** — ROS2 node that pipes GelSight images through the FEATS UNet to produce per-finger force estimates.

## Prerequisites

- Ubuntu 22.04 with ROS2 Humble
- Franka robot (or use `use_fake_hardware:=true` for simulation)
- GelSight cameras (or use dummy camera mode)
- Python packages: `torch`, `numpy`, `opencv-python`, `mujoco`, `robot_descriptions`

## Build

```bash
source /opt/ros/humble/setup.bash
git submodule update --init --recursive
colcon build
source install/setup.bash
```

Build a single package:

```bash
colcon build --packages-select tension_control
```

## Running

### Real Robot Bringup

```bash
# Full bringup with RViz
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.3 use_rviz:=true

# With MoveIt2
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.3
```

### Fake Hardware (no robot required)

```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

### Tension / Admittance Controller

```bash
ros2 launch tension_control admittance_controller.launch.py
# or
ros2 launch tension_control multi_priority_controller.launch.py
```

### Force Estimation

```bash
ros2 launch tension_estimator force_estimator.launch.py
```

### Cable Pose Estimation

```bash
ros2 launch tactile_cameras cable_pose_estimator_config.launch.py
```

### MuJoCo Simulation

```bash
python run_mujoco.py
```

### GelSight Cameras

```bash
bash scripts/demo_cameras.sh          # real USB cameras
bash scripts/demo_cameras.sh --dummy  # dummy mode
```

### Interactive Joint Control

```bash
python interactive_joint_commander.py
```

Provides predefined positions (home, ready, up, down, left, right) and custom joint entry.

### Motion Primitives

```bash
python scripts/primitive_1_detect_cable.py   # home → close gripper → detect cable
python scripts/primitive_2_circular_motion.py # circular trajectory with cable monitoring
```

## Testing

```bash
colcon test
colcon test-result --verbose
```

CI runs `colcon build` with clang-tidy (`-DCHECK_TIDY=ON`) then `colcon test` (clang-format, cppcheck, flake8, xmllint, copyright checks).

## Configuration

| File | Purpose |
|------|---------|
| `src/tension_control/config/admittance_params.yaml` | Mass, damping, stiffness gains |
| `src/tension_control/config/multi_priority_params.yaml` | Multi-task control weights and gains |
| `src/franka_ros2/franka_bringup/config/controllers.yaml` | Controller manager configuration |

## Documentation

- [`CLAUDE.md`](CLAUDE.md) — Codebase overview and essential commands
- [`src/tension_control/README_tactile_detection.md`](src/tension_control/README_tactile_detection.md) — Tactile detection architecture and ROS2 topics
- [`src/tactile_cameras/README.md`](src/tactile_cameras/README.md) — GelSight camera nodes and launch examples
- [`cable_pose_estimation_demo.ipynb`](cable_pose_estimation_demo.ipynb) — Jupyter demo of the cable pose estimation pipeline
- `.claude/docs/architectural_patterns.md` — ros2_control plugin pattern, real-time control loop, multi-priority control math, ML inference pipeline
