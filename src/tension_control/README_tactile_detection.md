# Tactile Object Detection for Tension Control

This module integrates tactile object detection using dual GelSight cameras with the tension control system. It provides real-time object detection capabilities and adaptive grip force control.

## Architecture

### Components

1. **Tactile Detection Service** (`tactile_detection_service.py`)
   - Lightweight ROS2 service for object detection
   - Background subtraction-based detection
   - Publishes detection results and confidence
   - No GUI, optimized for integration

2. **Tactile Object Detector** (`tactile_object_detector.py`)
   - Full-featured detector with Kivy GUI
   - Visual feedback and manual controls
   - Background learning interface
   - Real-time camera visualization

3. **Tension Control with Tactile** (`tension_control_tactile.py`)
   - Main integration node
   - State machine for control logic
   - Adaptive grip force based on tactile feedback
   - Safety monitoring and emergency stop

## Topics

### Published Topics

- `/tension_control/object_detected` (Bool) - Object presence
- `/tension_control/object_confidence` (Float32) - Detection confidence (0.0-1.0)
- `/tension_control/recommended_grip_force` (Float32) - Recommended grip force (N)
- `/tension_control/detection_status` (String) - Human-readable status
- `/tension_control/object_location` (Int32) - Object location (0=none, 1=cam1, 2=cam2, 3=both)
- `/tension_control/control_state` (String) - Current control state
- `/tension_control/grip_command` (Float32) - Current grip command
- `/tension_control/tension_status` (String) - Tension control status
- `/tension_control/safety_alert` (String) - Safety alerts

### Subscribed Topics

- `/gelsight_cam1/image_raw` (Image) - Camera 1 tactile images
- `/gelsight_cam2/image_raw` (Image) - Camera 2 tactile images
- `/franka_robot_state_broadcaster/external_wrench_in_base` (Wrench) - Force/torque data
- `/joint_states` (JointState) - Robot joint states

### Services

- `/learn_background` (Empty) - Manually trigger background learning
- `/emergency_stop_grip` (Empty) - Emergency stop and release grip

## Usage

### Quick Start

1. **Launch tactile detection service only:**
```bash
ros2 launch tension_control tactile_detection.launch.xml
```

2. **Launch with GUI:**
```bash
ros2 launch tension_control tactile_detection.launch.xml enable_gui:=true
```

3. **Launch with cameras:**
```bash
ros2 launch tension_control tactile_detection.launch.xml start_cameras:=true
```

4. **Launch integrated tension control:**
```bash
ros2 run tension_control tension_control_tactile
```

### Manual Control

**Learn background:**
```bash
ros2 service call /learn_background std_srvs/srv/Empty
```

**Emergency stop:**
```bash
ros2 service call /emergency_stop_grip std_srvs/srv/Empty
```

**Monitor detection status:**
```bash
ros2 topic echo /tension_control/detection_status
```

**Control gripper manually:**
```bash
# Open gripper
python3 /home/nuc_6g_life_3/franka_ws/src/tension_control/scripts/gripper_control.py open

# Gentle grasp
python3 /home/nuc_6g_life_3/franka_ws/src/tension_control/scripts/gripper_control.py grasp 10.0

# Close with specific force
python3 /home/nuc_6g_life_3/franka_ws/src/tension_control/scripts/gripper_control.py grasp 20.0
```

## Configuration

### Parameters

Edit `config/tactile_detection_params.yaml`:

```yaml
# Detection sensitivity (lower = more sensitive)
detection_threshold: 0.3

# Number of frames for background learning
background_learn_frames: 20

# Enable adaptive grip force
adaptive_grip_enabled: true

# Grip force limits
min_grip_force: 5.0    # Minimum grip force (N)
max_grip_force: 30.0   # Maximum grip force (N)

# Camera topics
camera1_topic: "/gelsight_cam1/image_raw"
camera2_topic: "/gelsight_cam2/image_raw"
```

### Launch Parameters

```bash
# Adjust detection sensitivity
ros2 launch tension_control tactile_detection.launch.xml detection_threshold:=0.2

# Disable adaptive grip
ros2 launch tension_control tactile_detection.launch.xml adaptive_grip:=false

# Set grip force limits
ros2 launch tension_control tactile_detection.launch.xml min_grip_force:=3.0 max_grip_force:=25.0
```

## Control States

The integrated system operates in several states:

1. **IDLE** - System inactive
2. **LEARNING_BACKGROUND** - Learning tactile baseline
3. **MONITORING** - Monitoring for object presence
4. **OBJECT_DETECTED** - Object detected, analyzing
5. **ADJUSTING_GRIP** - Adjusting grip force based on tactile feedback
6. **MAINTAINING_TENSION** - Maintaining tension with tactile feedback
7. **ERROR** - Error state, emergency stop activated

## Safety Features

### Automatic Safety Checks

- **Grip force limits** - Prevents excessive grip force
- **Tactile feedback loss** - Detects loss of camera feed
- **Emergency stop** - Immediate grip release on emergency

### Manual Safety Controls

- Emergency stop service call
- Manual grip force override
- State monitoring and logging

## Integration with Existing Controllers

### Admittance Controller Integration

The tactile detection integrates with the existing admittance controller by:

1. Providing object presence feedback
2. Recommending grip force adjustments
3. Monitoring contact forces during manipulation
4. Triggering safety stops when needed

### Example Integration Code

```python
# In your admittance controller
self.object_detected_sub = self.create_subscription(
    Bool, '/tension_control/object_detected', self.handle_object_detection, 10)

def handle_object_detection(self, msg):
    if msg.data:
        # Object detected - adjust control strategy
        self.enable_gentle_mode()
    else:
        # No object - use normal control
        self.disable_gentle_mode()
```

## Troubleshooting

### Common Issues

1. **No camera feed:**
   - Check camera connections and indices
   - Verify camera topics are publishing
   - Check permissions for camera access

2. **Poor detection performance:**
   - Adjust `detection_threshold` parameter
   - Re-learn background in current lighting
   - Check camera positioning and focus

3. **Grip force issues:**
   - Verify gripper action server is running
   - Check grip force limits in configuration
   - Monitor `/tension_control/safety_alert` topic

### Debug Commands

```bash
# Check camera topics
ros2 topic list | grep gelsight

# Monitor detection confidence
ros2 topic echo /tension_control/object_confidence

# View detection status
ros2 topic echo /tension_control/detection_status

# Check control state
ros2 topic echo /tension_control/control_state

# Monitor safety alerts
ros2 topic echo /tension_control/safety_alert
```

### Performance Optimization

For better performance:

1. **Reduce image resolution** if high framerate is needed
2. **Adjust detection frequency** in parameters
3. **Use hardware acceleration** for OpenCV operations
4. **Optimize network bandwidth** for image topics

## Dependencies

### ROS2 Packages
- `rclpy`
- `sensor_msgs`
- `std_msgs`
- `geometry_msgs`
- `control_msgs`
- `cv_bridge`

### Python Packages
- `opencv-python`
- `numpy`
- `kivy` (for GUI version)

### System Requirements
- Ubuntu 20.04/22.04
- ROS2 Humble/Iron
- Python 3.8+
- OpenCV 4.5+

Install dependencies:
```bash
# ROS2 dependencies
sudo apt install ros-humble-cv-bridge ros-humble-control-msgs

# Python dependencies
pip3 install opencv-python numpy kivy
```
