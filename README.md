# Franka Cable Manipulation Workspace

Research workspace for cable manipulation with a **Franka Panda 7-DOF arm**. Combines real-time robot control via ROS2/ros2_control, tactile force sensing from GelSight cameras (FEATS neural network), and MuJoCo physics simulation for offline validation of multi-priority control strategies.

## Architecture Overview

```
GelSight Cameras
      │
      ▼
cable_pose_estimator      ←── tactile_cameras
      │ pose / contact
      ▼
Multi-Priority Controller ←── tension_control (C++ plugin, 1 kHz)
      │ joint torques
      ▼
FrankaHardwareInterface   ←── franka_ros2 / libfranka
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
│   ├── tactile_cameras/        # GelSight drivers and cable pose estimation
│   └── feats/                  # FEATS UNet model weights and utilities
├── Mujoco/                     # MuJoCo simulation assets
├── scripts/                    # Utility and bringup helper scripts
└── cable_pose_estimation_demo.ipynb  # Cable pose estimation demo notebook
```

### Key Packages

**`tension_control`** — Core custom controller package.
- `admittance_controller.cpp` — Impedance/admittance control plugin (125 Hz)
- `multi_priority_controller.cpp` — Multi-task priority framework (cable tension + EE pose)
- `tactile_detection_service.py` — Cable detection state machine via GelSight
- `cable_grasp_orchestrator.py` — FSM orchestrator for the full cable pickup pipeline

**`tactile_cameras`** — GelSight camera integration and cable pose estimation.
- `camera_publisher.py` — Publishes raw GelSight images to ROS2 topics
- `cable_pose_estimator.py` — Cable pose from tactile contact geometry

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

## Quick Start — Cable Pickup Pipeline

### Terminal 1 — Launch

```bash
source /opt/ros/humble/setup.bash
source ~/franka_ws/install/setup.bash
export LD_LIBRARY_PATH=/home/nuc_6g_life_3/franka_ws/src/install/libfranka/lib:$LD_LIBRARY_PATH
ros2 launch tension_control cable_pickup_and_control.launch.py robot_ip:=172.16.0.3 use_rviz:=false
```

Wait for the log line: `Cable grasp orchestrator ready`

### Terminal 2 — Control

```bash
source /opt/ros/humble/setup.bash
source ~/franka_ws/install/setup.bash

# Start the grasp sequence
ros2 service call /cable_grasp/start std_srvs/srv/Trigger "{}"

# Stop
ros2 service call /cable_grasp/stop std_srvs/srv/Trigger "{}"

# Monitor FSM state
ros2 topic echo /cable_grasp/status
```

### Manual Gripper Commands

```bash
# Open
ros2 action send_goal /panda_gripper/gripper_action control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 0.0}}"

# Close (firm)
ros2 action send_goal /panda_gripper/gripper_action control_msgs/action/GripperCommand \
  "{command: {position: 0.001, max_effort: 30.0}}"
```

### Rebuild After Code Changes

```bash
cd ~/franka_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select tension_control
```

## Other Launch Options

### Real Robot Bringup (franka_bringup only)

```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.3 use_rviz:=true
```

### Fake Hardware (no robot required)

```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

### MuJoCo Simulation

```bash
python run_mujoco.py
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

## Results

- [Frank Research 3: Testing Multi Priority controller limits with tug of war](https://youtu.be/nled1iDKwn8)
- [Franka Research 3: Cable being gripped](https://youtube.com/shorts/g390GOfP2Ak?feature=share)
- [Franka Research 3: Multi Priority Controller](https://youtube.com/shorts/DwMnadb-N0o?feature=share)
- [Franka Research 3: Admittance controller](https://youtube.com/shorts/B2Tznev1DlI?feature=share)
