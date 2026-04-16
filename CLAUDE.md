# Franka Workspace

## Project Overview
Research workspace for cable manipulation with a Franka Panda 7-DOF arm. Combines real-time robot control via ROS2/ros2_control, tactile force sensing from GelSight cameras (FEATS neural network), and MuJoCo physics simulation for offline validation of multi-priority control strategies.

## Tech Stack
- **ROS2 Humble** — middleware, ros2_control, MoveIt2
- **C++17** — hardware interface, real-time controllers (1 kHz)
- **Python 3** — research nodes, ML inference, simulation scripts
- **libfranka 0.12.1** — low-level Franka FCI protocol (compiled submodule)
- **MuJoCo 3.x** — physics simulation
- **PyTorch** — FEATS UNet force estimation model
- **colcon / ament_cmake / ament_python** — build system

## Key Directories

| Path | Purpose |
|------|---------|
| `src/franka_ros2/` | Official Franka ROS2 packages: hardware interface, controllers, msgs, gripper, MoveIt config, bringup |
| `src/libfranka/` | Low-level Franka C++ library (git submodule, compiled in Docker) |
| `src/tension_control/` | Custom cable tension admittance controller (C++ plugin + Python nodes) |
| `src/tension_estimator/` | ROS2 node: GelSight images → FEATS inference → force estimates |
| `src/tactile_cameras/` | GelSight camera drivers, marker tracking, pose estimation |
| `src/feats/` | FEATS UNet model weights and inference utilities |
| `src/franka_moveit_example/` | MoveIt2 Python scripting examples |
| `Mujoco/` | Standalone MuJoCo simulation (multi-priority controller, cable model) |
| `scripts/` | Utility and bringup helper scripts |

### Key files inside `src/franka_ros2/`
| Subpackage | Purpose |
|------------|---------|
| `franka_hardware/` | `FrankaHardwareInterface` — SystemInterface implementation |
| `franka_example_controllers/` | Reference controller plugins (gravity comp, impedance, velocity…) |
| `franka_msgs/` | Custom msgs/srvs: `FrankaRobotState`, `SetJointStiffness`, etc. |
| `franka_bringup/` | Launch files and `config/controllers.yaml` |
| `franka_description/` | URDF, collision meshes |
| `franka_moveit_config/` | MoveIt2 SRDF, planning config |

## Essential Commands

```bash
# Build
source /opt/ros/humble/setup.bash
colcon build
colcon build --packages-select <package_name>

# Source after build
source install/setup.bash

# Test
colcon test
colcon test-result --verbose

# Bringup (real robot)
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.3 use_rviz:=true

# Bringup (fake hardware / no robot)
ros2 launch franka_bringup franka.launch.py robot_ip:=dont-care use_fake_hardware:=true

# MuJoCo simulation
cd Mujoco && python main.py
```

## CI/CD
Docker-based pipeline at `src/franka_ros2/.github/workflows/ci.yml`.  
Runs `colcon build` with clang-tidy (`-DCHECK_TIDY=ON`) then `colcon test` (clang-format, cppcheck, flake8, xmllint, copyright).

## Additional Documentation
- [`.claude/docs/architectural_patterns.md`](.claude/docs/architectural_patterns.md) — ros2_control plugin pattern, real-time control loop, hardware abstraction layers, multi-priority control math, ML inference pipeline, ROS2 action pattern
