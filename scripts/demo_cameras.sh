#!/bin/bash

echo "========================================"
echo "🎥 Simple Camera Test Demo"
echo "========================================"
echo "This demo will:"
echo "1. Start camera publisher (USB cameras or dummy)"
echo "2. Start camera viewer (OpenCV windows)"
echo ""
echo "⌨️  Controls:"
echo "   - ESC key in OpenCV window: Exit viewer"
echo "   - Ctrl+C: Stop all nodes"
echo ""

cd /home/nuc_6g_life_3/franka_ws

# Source the environment
source install/setup.bash

# Check if cameras are available
echo "🔍 Checking for USB cameras..."
if ls /dev/video* 1> /dev/null 2>&1; then
    echo "✅ USB cameras found:"
    ls -la /dev/video*
else
    echo "⚠️  No USB cameras found - will use dummy cameras"
fi
echo ""

# Launch the camera demo
echo "🚀 Starting camera demo..."
ros2 launch tension_control camera_demo.launch.xml
