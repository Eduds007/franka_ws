# libfranka 0.13.2 Installation Guide

This guide explains how to install the compatible libfranka version (0.13.2) for robot server version 6.

## Why libfranka 0.13.2?

According to the [Franka compatibility table](https://frankaemika.github.io/docs/compatibility.html), robot server version 6/3 requires libfranka version >= 0.10.0 to < 0.13.3.

## Installation Steps

### 1. Remove Incompatible Versions
```bash
sudo apt remove ros-humble-libfranka libfranka -y
```

### 2. Install Dependencies
```bash
sudo apt install -y build-essential cmake git libpoco-dev libeigen3-dev
sudo apt install -y ros-humble-pinocchio ros-humble-eigenpy
```

### 3. Clone and Build libfranka 0.13.2
```bash
# Clone the repository
cd /tmp
git clone --recursive https://github.com/frankaemika/libfranka.git
cd libfranka

# Checkout the compatible version
git checkout 0.13.2
git submodule update --init --recursive

# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc)

# Install
sudo make install
sudo ldconfig
```

### 4. Verify Installation
```bash
# Test basic connection (replace IP with your robot's IP)
cat << 'EOF' > /tmp/test_connection.cpp
#include <iostream>
#include <franka/robot.h>
#include <franka/exception.h>

int main() {
    try {
        std::cout << "Connecting to robot..." << std::endl;
        franka::Robot robot("172.16.0.3");  // Replace with your robot IP
        std::cout << "✓ Successfully connected!" << std::endl;
        
        auto state = robot.readOnce();
        std::cout << "✓ Successfully read robot state!" << std::endl;
        std::cout << "Robot mode: " << static_cast<int>(state.robot_mode) << std::endl;
        
        return 0;
    } catch (const franka::Exception& e) {
        std::cout << "✗ Franka exception: " << e.what() << std::endl;
        return 1;
    }
}
EOF

# Compile and run test
g++ -std=c++17 /tmp/test_connection.cpp -o /tmp/test_connection -lfranka
/tmp/test_connection
```

### 5. Build ROS2 Workspace
```bash
cd /path/to/your/franka_ws
colcon build --cmake-args -DCMAKE_PREFIX_PATH=/usr/local -DBUILD_TESTING=OFF
```

## Troubleshooting

### Version Check
```bash
# Check installed libfranka version
pkg-config --modversion franka
```

### Library Path Issues
If you get library loading errors:
```bash
sudo ldconfig
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### CMake Not Finding libfranka
Add to your CMake:
```cmake
set(CMAKE_PREFIX_PATH "/usr/local" ${CMAKE_PREFIX_PATH})
```

## Success Indicators

You'll know the installation is successful when:
- ✅ Test connection program runs without errors
- ✅ ROS2 workspace builds without libfranka errors
- ✅ Robot connection shows "Successfully connected to robot" in ROS2 logs
- ✅ No version mismatch errors

## Version Compatibility Reference

| Robot Server | libfranka Version | Status |
|-------------|-------------------|---------|
| 6/3 | 0.10.0 to < 0.13.3 | ✅ This guide |
| 7/3 | 0.13.3 to < 0.14.1 | ❌ |
| 8/3 | 0.14.1 to 0.15.0 | ❌ |
| 9/3 | >= 0.15.0 | ❌ |
| 10/3 | >= 0.18.0 | ❌ |
