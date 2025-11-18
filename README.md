# Franka ROS2 Workspace - Compatible Version

A working Franka ROS2 workspace with compatibility fixes for older Franka robot firmware (server version 6).

## 🎯 Overview

This repository contains a fully functional Franka ROS2 setup that has been tested and verified to work with:
- **Robot Server Version**: 6/3
- **Ubuntu**: 22.04 LTS
- **ROS2**: Humble
- **libfranka**: 0.13.2 (compatible with server version 6)
- **franka_ros2**: 0.1.7

## ✅ Verified Features

- ✅ Robot state communication at 1000 Hz
- ✅ Joint position, velocity, and effort feedback
- ✅ Torque control interface
- ✅ Joint velocity control interface  
- ✅ Joint position control interface
- ✅ Cartesian velocity control interface
- ✅ Real-time robot state broadcasting
- ✅ All ROS2 controllers and examples

## 🔧 Key Fixes Applied

### 1. **libfranka Version Compatibility**
- Downgraded from libfranka 0.15.0 to 0.13.2
- Compatible with robot server version 6/3 as per [Franka compatibility table](https://frankaemika.github.io/docs/compatibility.html)

### 2. **Type System Compatibility**  
- Fixed `std::unique_ptr<franka::ActiveControl>` → `std::unique_ptr<franka::ActiveControlBase>`
- Resolved compilation errors in `robot.hpp`

### 3. **Dependencies**
- Updated pinocchio libraries to compatible versions
- Resolved linking issues

## 🚀 Quick Start

### Prerequisites
```bash
# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install required dependencies
sudo apt install -y build-essential cmake git libpoco-dev libeigen3-dev
sudo apt install -y ros-humble-pinocchio ros-humble-eigenpy
```

### Clone and Build
```bash
# Clone the repository
git clone https://github.com/eduds007/franka-ros2-workspace.git
cd franka-ros2-workspace

# Install compatible libfranka (0.13.2)
# Note: The compatible libfranka should be built from source as shown in the setup

# Build the workspace
colcon build --cmake-args -DCMAKE_PREFIX_PATH=/usr/local -DBUILD_TESTING=OFF

# Source the workspace
source install/setup.bash
```

### Connect to Robot
```bash
# Replace 172.16.0.3 with your robot's IP address
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.3 use_rviz:=false
```

## 🔍 Testing Communication

Test scripts are included to verify robot communication:

```bash
# Test basic network connectivity
python3 test_basic_connection.py

# Test direct libfranka connection  
python3 test_libfranka_direct.py

# Test ROS2 communication
python3 test_robot_communication.py
```

## 📋 Robot Compatibility

| Robot System Version | libfranka Version | Robot/Gripper Server | Ubuntu | franka_ros2 (Humble) | Status |
|----------------------|-------------------|---------------------|--------|---------------------|---------|
| >= 5.2.0 | >= 0.10.0 to < 0.13.3 | 6/3 | 22.04 | v0.1.0 - v0.1.8 | ✅ **This Repo** |
| >= 5.5.0 | >= 0.13.3 to < 0.14.1 | 7/3 | 22.04 | v0.1.15 < v2.0.0 | ❌ |
| >= 5.7.0 | >= 0.14.1 to 0.15.0 | 8/3 | 22.04 | v0.1.15 < v2.0.0 | ❌ |

## 🛠️ Example Controllers

Launch files for various control modes:

```bash
# Gravity compensation (safest for testing)
ros2 launch franka_bringup gravity_compensation_example_controller.launch.py robot_ip:=<YOUR_ROBOT_IP>

# Joint velocity control
ros2 launch franka_bringup joint_velocity_example_controller.launch.py robot_ip:=<YOUR_ROBOT_IP>

# Cartesian velocity control  
ros2 launch franka_bringup cartesian_velocity_example_controller.launch.py robot_ip:=<YOUR_ROBOT_IP>

# Joint position control
ros2 launch franka_bringup joint_position_example_controller.launch.py robot_ip:=<YOUR_ROBOT_IP>
```

## 📊 Available Topics

When the robot is connected, these topics are available:

```bash
/franka_robot_state_broadcaster/robot_state  # Franka-specific robot state
/franka/joint_states                         # Joint positions, velocities, efforts  
/joint_states                                # Standard ROS joint states
/tf                                          # Transform tree
/tf_static                                   # Static transforms
```

## ⚠️ Known Issues

1. **Gripper FCI**: Gripper requires FCI mode to be enabled in Franka Desk
2. **Web Interface**: Robot web interface (port 8080) may not be accessible remotely
3. **Real-time Kernel**: Recommended but not required for basic functionality

## 🔧 Troubleshooting

### Version Mismatch Error
```
libfranka: Incompatible library version (server version: 6, library version: 9)
```
**Solution**: This repository already includes the fix with libfranka 0.13.2

### FCI Connection Refused
```
libfranka: Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk.
```
**Solution**: Enable FCI mode in Franka Desk (affects gripper only, robot arm works without it)

### Compilation Errors
```
error: no match for 'operator=' (operand types are 'std::unique_ptr<franka::ActiveControl>' and 'std::unique_ptr<franka::ActiveControlBase>')
```
**Solution**: This repository already includes the type system fix

## 📄 License

This project is licensed under the Apache 2.0 License - see the original Franka Emika license files for details.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some Amazing Feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📞 Support

- **Robot Communication Issues**: Check the compatibility table and ensure correct libfranka version
- **Build Issues**: Verify all dependencies are installed
- **Runtime Issues**: Check robot IP address and network connectivity

## 🎉 Success Story

This workspace was successfully tested with:
- ✅ Real robot at IP 172.16.0.3
- ✅ 1000 Hz control loop
- ✅ All control interfaces working
- ✅ Real-time joint state feedback
- ✅ Successful connection and control

---

**Made with ❤️ for the robotics community**
