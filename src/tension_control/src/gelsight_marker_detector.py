#!/usr/bin/env python3
"""
GelSight Object Detection based on Marker Displacement
Detects when objects are grasped by analyzing marker displacement patterns
Similar to shear force measurement but focused on object detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32, String
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Lock
import time
from typing import Optional, Tuple


class MarkerTracker:
    """
    Simple marker tracker for GelSight tactile patterns
    Based on the utilities.marker_tracker approach
    """
    
    def __init__(self, image: np.ndarray):
        self.initial_marker_center = self._detect_initial_markers(image)
        
    def _detect_initial_markers(self, image: np.ndarray) -> np.ndarray:
        """Detect initial marker positions using blob detection"""
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor((image * 255).astype(np.uint8), cv2.COLOR_RGB2GRAY)
        else:
            gray = (image * 255).astype(np.uint8)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # Detect dark spots (markers) using HoughCircles
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=15,  # Minimum distance between circle centers
            param1=50,   # Upper threshold for edge detection
            param2=25,   # Lower threshold for center detection
            minRadius=3, # Minimum circle radius
            maxRadius=20 # Maximum circle radius
        )
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            # Convert to (y, x) format for consistency with original code
            markers = []
            for circle in circles[0, :]:
                x, y, r = circle
                markers.append([y, x])  # Note: swapped to match original format
            return np.array(markers, dtype=np.float32)
        else:
            # Fallback: return empty array
            return np.array([], dtype=np.float32).reshape(0, 2)


class ObjectDetectionCalculator:
    """
    Calculator for object detection based on marker displacement analysis.
    Similar to ShearForceCalculator but focused on detection rather than force measurement.
    """
    
    def __init__(self, detection_threshold: float = 2.0, min_displaced_markers: int = 3):
        """
        Initialize object detection calculator.
        
        Args:
            detection_threshold (float): Minimum displacement in pixels to consider significant
            min_displaced_markers (int): Minimum number of displaced markers for detection
        """
        self.detection_threshold = detection_threshold
        self.min_displaced_markers = min_displaced_markers
        self.reference_positions = None
        
    def set_reference(self, positions: np.ndarray):
        """Set reference positions for zero deformation state"""
        if len(positions) > 0:
            self.reference_positions = positions.copy()
        else:
            self.reference_positions = None
            
    def analyze_displacement(self, current_positions: np.ndarray) -> Tuple[bool, float, int, float, float]:
        """
        Analyze marker displacement to detect object presence.
        
        Args:
            current_positions: Current marker positions (N, 2)
            
        Returns:
            Tuple of (object_detected, confidence, displaced_count, avg_displacement, max_displacement)
        """
        if self.reference_positions is None or len(current_positions) == 0:
            return False, 0.0, 0, 0.0, 0.0
        
        # Match current positions to reference positions
        matched_displacements = []
        
        for ref_pos in self.reference_positions:
            # Find closest current position
            if len(current_positions) > 0:
                distances = np.sqrt(np.sum((current_positions - ref_pos)**2, axis=1))
                min_idx = np.argmin(distances)
                if distances[min_idx] < 30:  # Maximum matching distance
                    displacement = np.linalg.norm(current_positions[min_idx] - ref_pos)
                    matched_displacements.append(displacement)
        
        if len(matched_displacements) == 0:
            return False, 0.0, 0, 0.0, 0.0
        
        # Calculate displacement statistics
        displacements = np.array(matched_displacements)
        avg_displacement = np.mean(displacements)
        max_displacement = np.max(displacements)
        displaced_count = np.sum(displacements > self.detection_threshold)
        
        # Object detection logic
        object_detected = (
            displaced_count >= self.min_displaced_markers and 
            avg_displacement > self.detection_threshold * 0.5
        )
        
        # Confidence based on displacement magnitude and number of displaced markers
        confidence = min(1.0, (avg_displacement / (self.detection_threshold * 2)) * 
                        (displaced_count / max(1, len(self.reference_positions))))
        
        return object_detected, confidence, displaced_count, avg_displacement, max_displacement


class GelSightMarkerObjectDetector(Node):
    """Object detection based on GelSight marker displacement tracking"""
    
    def __init__(self):
        super().__init__('gelsight_marker_object_detector')
        
        # Parameters
        self.declare_parameter('detection_threshold', 3.0)  # pixels
        self.declare_parameter('min_displaced_markers', 3)
        self.declare_parameter('reference_frames', 15)
        self.declare_parameter('tracking_quality_threshold', 0.7)
        
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.min_displaced_markers = self.get_parameter('min_displaced_markers').value
        self.reference_frames = self.get_parameter('reference_frames').value
        self.tracking_quality = self.get_parameter('tracking_quality_threshold').value
        
        # ROS2 Setup
        self.bridge = CvBridge()
        self.frame_lock = Lock()
        
        # Image subscribers
        self.cam1_sub = self.create_subscription(
            Image, '/gelsight_cam1/image_raw', self.camera1_callback, 10)
        self.cam2_sub = self.create_subscription(
            Image, '/gelsight_cam2/image_raw', self.camera2_callback, 10)
        
        # Publishers
        self.object_detected_pub = self.create_publisher(Bool, '/tension_control/object_detected', 10)
        self.confidence_pub = self.create_publisher(Float32, '/tension_control/object_confidence', 10)
        self.displacement_pub = self.create_publisher(Float32, '/tension_control/avg_displacement', 10)
        self.status_pub = self.create_publisher(String, '/tension_control/detection_status', 10)
        self.location_pub = self.create_publisher(Int32, '/tension_control/object_location', 10)
        
        # State variables for camera 1
        self.frame1 = None
        self.initialized1 = False
        self.marker_tracker1 = None
        self.old_gray1 = None
        self.p0_1 = None
        self.detection_calc1 = ObjectDetectionCalculator(
            self.detection_threshold, self.min_displaced_markers)
        
        # State variables for camera 2  
        self.frame2 = None
        self.initialized2 = False
        self.marker_tracker2 = None
        self.old_gray2 = None
        self.p0_2 = None
        self.detection_calc2 = ObjectDetectionCalculator(
            self.detection_threshold, self.min_displaced_markers)
        
        self.frame_count = 0
        self.calibrated = False
        
        # Lucas-Kanade optical flow parameters
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        
        # Detection timer
        self.detection_timer = self.create_timer(0.033, self.process_detection)  # ~30 Hz
        
        self.get_logger().info("🔍 GelSight Marker Object Detector initialized")
        self.get_logger().info(f"📐 Parameters:")
        self.get_logger().info(f"   - Detection threshold: {self.detection_threshold} pixels")
        self.get_logger().info(f"   - Min displaced markers: {self.min_displaced_markers}")
        self.get_logger().info(f"   - Reference frames: {self.reference_frames}")
    
    def camera1_callback(self, msg):
        """Process camera 1 frames"""
        try:
            with self.frame_lock:
                self.frame1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera 1: {e}")
    
    def camera2_callback(self, msg):
        """Process camera 2 frames"""
        try:
            with self.frame_lock:
                self.frame2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera 2: {e}")
    
    def initialize_camera(self, frame: np.ndarray, camera_id: int):
        """Initialize marker tracking for a specific camera"""
        try:
            # Convert to float32 image for marker tracker
            img = np.float32(frame) / 255.0
            
            if camera_id == 1:
                self.marker_tracker1 = MarkerTracker(img)
                marker_centers = self.marker_tracker1.initial_marker_center
                
                if len(marker_centers) == 0:
                    self.get_logger().warn(f"📍 Camera 1: No markers detected!")
                    return False
                
                self.get_logger().info(f"📍 Camera 1: Found {len(marker_centers)} initial markers")
                
                # Convert to Lucas-Kanade format
                self.p0_1 = marker_centers.reshape(-1, 1, 2).astype(np.float32)
                # Swap x,y to match OpenCV format
                self.p0_1[:, 0, [0, 1]] = self.p0_1[:, 0, [1, 0]]
                
                self.old_gray1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                self.initialized1 = True
                return True
                
            elif camera_id == 2:
                self.marker_tracker2 = MarkerTracker(img)
                marker_centers = self.marker_tracker2.initial_marker_center
                
                if len(marker_centers) == 0:
                    self.get_logger().warn(f"📍 Camera 2: No markers detected!")
                    return False
                
                self.get_logger().info(f"📍 Camera 2: Found {len(marker_centers)} initial markers")
                
                # Convert to Lucas-Kanade format
                self.p0_2 = marker_centers.reshape(-1, 1, 2).astype(np.float32)
                # Swap x,y to match OpenCV format
                self.p0_2[:, 0, [0, 1]] = self.p0_2[:, 0, [1, 0]]
                
                self.old_gray2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                self.initialized2 = True
                return True
                
        except Exception as e:
            self.get_logger().error(f"Error initializing camera {camera_id}: {e}")
            return False
    
    def track_markers(self, frame: np.ndarray, camera_id: int) -> Tuple[bool, float, int, float, float]:
        """Track markers and detect object for specific camera"""
        try:
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            if camera_id == 1:
                if self.p0_1 is None or len(self.p0_1) == 0:
                    return False, 0.0, 0, 0.0, 0.0
                    
                # Calculate optical flow
                p1, st, err = cv2.calcOpticalFlowPyrLK(
                    self.old_gray1, frame_gray, self.p0_1, None, **self.lk_params
                )
                
                # Select good points
                if st is not None and len(st) > 0:
                    good_new = p1[st == 1]
                    good_old = self.p0_1[st == 1]
                    
                    tracking_quality = len(good_new) / len(self.p0_1) if len(self.p0_1) > 0 else 0.0
                    
                    if tracking_quality >= self.tracking_quality and len(good_new) >= 3:
                        self.p0_1 = good_new.reshape(-1, 1, 2)
                        self.old_gray1 = frame_gray.copy()
                        
                        # Analyze for object detection
                        current_positions = good_new.reshape(-1, 2)
                        return self.detection_calc1.analyze_displacement(current_positions)
                    else:
                        self.get_logger().debug(f"Camera 1: Poor tracking quality {tracking_quality:.2f}")
                
            elif camera_id == 2:
                if self.p0_2 is None or len(self.p0_2) == 0:
                    return False, 0.0, 0, 0.0, 0.0
                    
                # Calculate optical flow
                p1, st, err = cv2.calcOpticalFlowPyrLK(
                    self.old_gray2, frame_gray, self.p0_2, None, **self.lk_params
                )
                
                # Select good points
                if st is not None and len(st) > 0:
                    good_new = p1[st == 1]
                    good_old = self.p0_2[st == 1]
                    
                    tracking_quality = len(good_new) / len(self.p0_2) if len(self.p0_2) > 0 else 0.0
                    
                    if tracking_quality >= self.tracking_quality and len(good_new) >= 3:
                        self.p0_2 = good_new.reshape(-1, 1, 2)
                        self.old_gray2 = frame_gray.copy()
                        
                        # Analyze for object detection
                        current_positions = good_new.reshape(-1, 2)
                        return self.detection_calc2.analyze_displacement(current_positions)
                    else:
                        self.get_logger().debug(f"Camera 2: Poor tracking quality {tracking_quality:.2f}")
                        
        except Exception as e:
            self.get_logger().error(f"Error tracking markers camera {camera_id}: {e}")
        
        return False, 0.0, 0, 0.0, 0.0
    
    def process_detection(self):
        """Main detection processing loop"""
        with self.frame_lock:
            current_frame1 = self.frame1.copy() if self.frame1 is not None else None
            current_frame2 = self.frame2.copy() if self.frame2 is not None else None
        
        if current_frame1 is None and current_frame2 is None:
            return
        
        # Calibration phase
        if not self.calibrated:
            self.frame_count += 1
            
            if self.frame_count <= self.reference_frames:
                status_msg = String()
                status_msg.data = f"Calibrating markers... {self.frame_count}/{self.reference_frames}"
                self.status_pub.publish(status_msg)
                
                # Initialize cameras during calibration
                if current_frame1 is not None and not self.initialized1:
                    self.initialize_camera(current_frame1, 1)
                    
                if current_frame2 is not None and not self.initialized2:
                    self.initialize_camera(current_frame2, 2)
                
                return
            
            elif self.frame_count == self.reference_frames + 1:
                # Set reference positions
                if self.initialized1 and self.p0_1 is not None:
                    ref_pos_1 = self.p0_1.reshape(-1, 2)
                    self.detection_calc1.set_reference(ref_pos_1)
                    self.get_logger().info(f"✅ Camera 1: Reference set with {len(ref_pos_1)} markers")
                
                if self.initialized2 and self.p0_2 is not None:
                    ref_pos_2 = self.p0_2.reshape(-1, 2)
                    self.detection_calc2.set_reference(ref_pos_2)
                    self.get_logger().info(f"✅ Camera 2: Reference set with {len(ref_pos_2)} markers")
                
                self.calibrated = True
                self.get_logger().info("✅ Calibration complete - Starting object detection")
                
                status_msg = String()
                status_msg.data = "Calibrated - Ready for object detection"
                self.status_pub.publish(status_msg)
                return
        
        # Detection phase
        if not self.calibrated:
            return
        
        detection1, confidence1, displaced1, avg_disp1, max_disp1 = False, 0.0, 0, 0.0, 0.0
        detection2, confidence2, displaced2, avg_disp2, max_disp2 = False, 0.0, 0, 0.0, 0.0
        
        # Process camera 1
        if current_frame1 is not None and self.initialized1:
            detection1, confidence1, displaced1, avg_disp1, max_disp1 = \
                self.track_markers(current_frame1, 1)
        
        # Process camera 2
        if current_frame2 is not None and self.initialized2:
            detection2, confidence2, displaced2, avg_disp2, max_disp2 = \
                self.track_markers(current_frame2, 2)
        
        # Combined detection decision
        object_detected = detection1 or detection2
        combined_confidence = max(confidence1, confidence2)
        combined_displacement = max(avg_disp1, avg_disp2)
        
        # Determine location
        if detection1 and detection2:
            location = 0  # Both cameras
        elif detection1:
            location = 1  # Camera 1
        elif detection2:
            location = 2  # Camera 2
        else:
            location = 0  # Neither
        
        # Publish results
        detection_msg = Bool()
        detection_msg.data = bool(object_detected)
        self.object_detected_pub.publish(detection_msg)
        
        confidence_msg = Float32()
        confidence_msg.data = float(combined_confidence)
        self.confidence_pub.publish(confidence_msg)
        
        displacement_msg = Float32()
        displacement_msg.data = float(combined_displacement)
        self.displacement_pub.publish(displacement_msg)
        
        location_msg = Int32()
        location_msg.data = int(location)
        self.location_pub.publish(location_msg)
        
        # Status message with detailed info
        status_msg = String()
        if object_detected:
            status_msg.data = (f"OBJECT DETECTED - Confidence: {combined_confidence:.3f}, "
                             f"Cam1: {displaced1} displaced ({avg_disp1:.1f}px avg), "
                             f"Cam2: {displaced2} displaced ({avg_disp2:.1f}px avg)")
            self.get_logger().info(f"🎯 {status_msg.data}")
        else:
            status_msg.data = (f"No object - Cam1: {displaced1} displaced ({avg_disp1:.1f}px), "
                             f"Cam2: {displaced2} displaced ({avg_disp2:.1f}px)")
        
        self.status_pub.publish(status_msg)
    
    def reset_calibration(self):
        """Reset calibration for new reference"""
        self.calibrated = False
        self.frame_count = 0
        self.initialized1 = False
        self.initialized2 = False
        self.detection_calc1.reference_positions = None
        self.detection_calc2.reference_positions = None
        self.get_logger().info("🔄 Calibration reset")


def main():
    """Main function"""
    rclpy.init()
    
    try:
        detector = GelSightMarkerObjectDetector()
        
        print("\n" + "="*60)
        print("🔍 GelSight Marker-based Object Detector")
        print("="*60)
        print("📡 Listening to camera topics:")
        print("   • /gelsight_cam1/image_raw")
        print("   • /gelsight_cam2/image_raw")
        print("\n📊 Publishing detection results to:")
        print("   • /tension_control/object_detected")
        print("   • /tension_control/object_confidence")
        print("   • /tension_control/avg_displacement")
        print("   • /tension_control/detection_status")
        print("   • /tension_control/object_location")
        print("\n🎯 Detection process:")
        print("   1. Calibration phase (15 frames)")
        print("   2. Marker tracking using Lucas-Kanade")
        print("   3. Displacement analysis")
        print("   4. Object detection decision")
        print("\n⚙️  Detection criteria:")
        print("   • Displacement threshold: 3.0 pixels")
        print("   • Min displaced markers: 3")
        print("   • Tracking quality threshold: 70%")
        print("\n⌨️  Press Ctrl+C to stop")
        print("="*60)
        
        rclpy.spin(detector)
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        if 'detector' in locals():
            detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
