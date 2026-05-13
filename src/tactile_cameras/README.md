# Tactile Cameras Package

ROS 2 package for GelSight tactile camera acquisition and processing.

## Overview

This package handles acquisition and processing of images from GelSight tactile cameras. It includes nodes for image publishing, object detection, marker analysis, and visualization.

## Nodes

### C++ Nodes

- **gelsight_camera_node**: Node for GelSight camera image acquisition

### Python Nodes

- **camera_publisher.py**: Publishes images from multiple USB cameras
- **camera_viewer.py**: Visualizes camera streams in real time
- **gelsight_object_detection.py**: Detects objects using image difference analysis
- **gelsight_detector_visual.py**: Visual detection with feedback
- **gelsight_marker_detector.py**: Detection based on marker tracking
- **gelsight_detector_debug.py**: Debug version with detailed logging

## Launch Files

- **camera_demo.launch.xml**: Basic demo with publisher and viewer
- **gelsight_camera.launch.py**: Launch for left/right GelSight cameras
- **gelsight_debug.launch.xml**: Launch with debug logging
- **gelsight_detection_test.launch.xml**: Object detection test
- **gelsight_marker_detection_test.launch.xml**: Marker-based detection test

## Configuration

GelSight camera parameters are in `config/gelsight_params.yaml`.

## Published Topics

- `/camera/camera1/image_raw` - Camera 1 image (sensor_msgs/Image)
- `/camera/camera2/image_raw` - Camera 2 image (sensor_msgs/Image)
- `/gelsight/left/image_raw` - Left GelSight image
- `/gelsight/right/image_raw` - Right GelSight image
- `/gelsight/object_detected` - Detection status (std_msgs/Bool)
- `/gelsight/deformation` - Detected deformation (geometry_msgs/Vector3)

## Usage

### View cameras:
```bash
ros2 launch tactile_cameras camera_demo.launch.xml
```

### GelSight cameras:
```bash
ros2 launch tactile_cameras gelsight_camera.launch.py
```

### Detection test:
```bash
ros2 launch tactile_cameras gelsight_detection_test.launch.xml
```

## Dependencies

- rclcpp
- rclpy
- sensor_msgs
- cv_bridge
- OpenCV
- numpy
