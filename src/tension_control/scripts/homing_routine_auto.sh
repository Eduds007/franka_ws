#!/bin/bash
# Simple Complete Homing Routine Script (Non-interactive)
# Executes: Homing -> Cable Position -> Close Gripper

# Check if we're in the correct workspace directory
if [ ! -d "install/tension_control" ]; then
    echo "ERROR: Not in the correct workspace directory!"
    echo "Please run this script from: /home/nuc_6g_life_3/franka_ws/"
    exit 1
fi

# Source the ROS2 environment
echo "Sourcing ROS2 environment..."
source install/setup.bash

echo "Starting complete homing routine..."
echo "WARNING: Robot will move! Make sure workspace is clear!"

# Step 1: Move to Homing Position
echo "Step 1/3: Moving to HOMING position..."
ros2 run tension_control move_homing
python3 src/tension_control/scripts/gripper_control.py position 0.03
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to move to homing position!"
    exit 1
fi

# Wait between movements
sleep 2

# Step 2: Move to Cable Position
echo "Step 2/3: Moving to CABLE position..."
ros2 run tension_control move_homing cable
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to move to cable position!"
    exit 1
fi

# Wait before gripper action
sleep 2

# Step 3: Close Gripper
echo "Step 3/3: Closing gripper..."
python3 src/tension_control/scripts/gripper_control.py close
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to close gripper!"
    exit 1
fi

echo "SUCCESS: Complete homing routine finished!"
echo "✅ Robot moved to homing position"
echo "✅ Robot moved to cable position" 
echo "✅ Gripper closed"
