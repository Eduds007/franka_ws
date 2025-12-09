#!/bin/bash
# Demo script for Tactile Object Detection with Tension Control
# Usage: ./demo_tactile_detection.sh [mode]
# Modes: service, gui, integrated, test

set -e

# Configuration
WORKSPACE_DIR="/home/nuc_6g_life_3/franka_ws"
PACKAGE_NAME="tension_control"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if ROS2 is sourced
check_ros_setup() {
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 not sourced! Please run: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    
    if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        print_error "Workspace not built! Please run: cd $WORKSPACE_DIR && colcon build"
        exit 1
    fi
    
    print_info "Sourcing workspace..."
    source "$WORKSPACE_DIR/install/setup.bash"
}

# Function to check camera availability
check_cameras() {
    print_info "Checking camera availability..."
    
    # Check if camera devices exist
    if [ ! -e "/dev/video0" ] && [ ! -e "/dev/video1" ]; then
        print_warning "No camera devices found at /dev/video0 or /dev/video1"
        print_warning "The system will work but cameras need to be connected for actual detection"
    else
        print_success "Camera devices detected"
    fi
}

# Function to run tactile detection service only
run_service_mode() {
    print_info "🔍 Starting Tactile Detection Service..."
    print_info "This mode runs only the detection service without GUI"
    print_info "Press Ctrl+C to stop"
    
    ros2 launch tension_control tactile_detection.launch.xml
}

# Function to run simple demo (no cameras required)
run_simple_mode() {
    print_info "🎮 Starting Simple Tactile Demo..."
    print_info "This mode simulates detection without requiring real cameras"
    print_info "Perfect for testing the system integration"
    print_info "Press Ctrl+C to stop"
    
    ros2 launch tension_control tactile_demo.launch.xml demo_mode:=simple
}

# Function to run with GUI
run_gui_mode() {
    print_info "🖥️ Starting Tactile Detection with GUI..."
    print_info "This mode includes visual interface for monitoring"
    print_info "Press Ctrl+C to stop"
    
    ros2 launch tension_control tactile_detection.launch.xml enable_gui:=true
}

# Function to run integrated system
run_integrated_mode() {
    print_info "🤖 Starting Integrated Tension Control with Tactile Feedback..."
    print_info "This mode runs the full integrated system"
    print_info "Make sure the robot is connected and gripper action server is running"
    print_info "Press Ctrl+C to stop"
    
    # Start detection service in background
    print_info "Starting tactile detection service..."
    ros2 launch tension_control tactile_detection.launch.xml &
    DETECTION_PID=$!
    
    # Wait a moment for service to start
    sleep 3
    
    # Start integrated controller
    print_info "Starting integrated tension controller..."
    ros2 run tension_control tension_control_tactile
    
    # Cleanup
    print_info "Stopping background processes..."
    kill $DETECTION_PID 2>/dev/null || true
}

# Function to run test mode
run_test_mode() {
    print_info "🧪 Running Test Mode..."
    print_info "This mode tests the system functionality"
    
    # Test 1: Check if nodes can be imported
    print_info "Test 1: Checking Python imports..."
    python3 -c "
import sys
sys.path.append('$WORKSPACE_DIR/src/tension_control/src')
try:
    from tactile_detection_service import TactileObjectDetectionService
    print('✅ Tactile detection service imports OK')
except ImportError as e:
    print(f'❌ Import error: {e}')

try:
    from tension_control_tactile import TensionControlWithTactile
    print('✅ Tension control tactile imports OK')
except ImportError as e:
    print(f'❌ Import error: {e}')
"

    # Test 2: Check ROS2 topics (if any nodes are running)
    print_info "Test 2: Checking available ROS2 topics..."
    timeout 3 ros2 topic list | grep -E "(gelsight|tension_control)" || print_warning "No relevant topics found (nodes may not be running)"
    
    # Test 3: Test gripper control script
    print_info "Test 3: Testing gripper control script..."
    python3 "$WORKSPACE_DIR/src/tension_control/scripts/gripper_control.py" --help | head -5
    
    print_success "Basic tests completed!"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [mode]"
    echo ""
    echo "Available modes:"
    echo "  simple     - Run simple demo (no cameras required)"
    echo "  service    - Run tactile detection service only" 
    echo "  gui        - Run tactile detection with GUI"
    echo "  integrated - Run full integrated system"
    echo "  test       - Run system tests"
    echo "  help       - Show this help"
    echo ""
    echo "Examples:"
    echo "  $0 simple      # Start simple demo (best for testing)"
    echo "  $0 service     # Start detection service"
    echo "  $0 gui         # Start with visual interface"
    echo "  $0 integrated  # Start full system"
    echo "  $0 test        # Run tests"
}

# Function to show system info
show_system_info() {
    print_info "=== Tactile Object Detection System Info ==="
    echo "Workspace: $WORKSPACE_DIR"
    echo "Package: $PACKAGE_NAME"
    echo "ROS2 Distro: ${ROS_DISTRO:-Not sourced}"
    echo ""
    
    print_info "Available executables:"
    echo "  - simple_tactile_demo          (Simple demo - no cameras)"
    echo "  - tactile_detection_service    (Detection service)"
    echo "  - tactile_object_detector      (Detection with GUI)"
    echo "  - tension_control_tactile      (Integrated controller)"
    echo "  - gripper_control.py           (Manual gripper control)"
    echo ""
    
    print_info "Key topics:"
    echo "  - /tension_control/object_detected"
    echo "  - /tension_control/object_confidence"
    echo "  - /tension_control/recommended_grip_force"
    echo "  - /tension_control/detection_status"
    echo ""
    
    print_info "Services:"
    echo "  - /learn_background"
    echo "  - /emergency_stop_grip"
}

# Main script logic
main() {
    # Show header
    echo "=========================================="
    echo "🤖 Tactile Object Detection Demo"
    echo "=========================================="
    
    # Check ROS setup
    check_ros_setup
    
    # Check cameras
    check_cameras
    
    # Show system info
    show_system_info
    echo ""
    
    # Get mode from argument
    MODE=${1:-"help"}
    
    case $MODE in
        "service")
            run_service_mode
            ;;
        "simple")
            run_simple_mode
            ;;
        "gui")
            run_gui_mode
            ;;
        "integrated")
            run_integrated_mode
            ;;
        "test")
            run_test_mode
            ;;
        "help")
            show_usage
            ;;
        *)
            print_error "Unknown mode: $MODE"
            show_usage
            exit 1
            ;;
    esac
}

# Trap for cleanup
cleanup() {
    print_info "Cleaning up..."
    jobs -p | xargs -r kill 2>/dev/null || true
    exit 0
}

trap cleanup EXIT INT TERM

# Run main function
main "$@"
