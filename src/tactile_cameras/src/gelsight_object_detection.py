#!/usr/bin/env python3
"""
GelSight Object Detection based on Marker Deformation
Detects when objects are grasped by analyzing deformation of circular markers
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


class GelSightObjectDetector(Node):
    """Object detection based on GelSight marker deformation"""
    
    def __init__(self):
        super().__init__('gelsight_object_detector')
        
        # Parameters
        self.declare_parameter('detection_threshold', 0.15)
        self.declare_parameter('reference_frames', 10)
        self.declare_parameter('marker_detection_threshold', 30)
        self.declare_parameter('min_marker_area', 50)
        self.declare_parameter('max_marker_area', 500)
        self.declare_parameter('deformation_threshold', 0.2)
        
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.reference_frames = self.get_parameter('reference_frames').value
        self.marker_threshold = self.get_parameter('marker_detection_threshold').value
        self.min_marker_area = self.get_parameter('min_marker_area').value
        self.max_marker_area = self.get_parameter('max_marker_area').value
        self.deformation_threshold = self.get_parameter('deformation_threshold').value
        
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
        self.deformation_pub = self.create_publisher(Float32, '/tension_control/deformation_level', 10)
        self.status_pub = self.create_publisher(String, '/tension_control/detection_status', 10)
        self.location_pub = self.create_publisher(Int32, '/tension_control/object_location', 10)
        
        # State variables
        self.frame1 = None
        self.frame2 = None
        self.reference_frame1 = None
        self.reference_frame2 = None
        self.reference_markers1 = []
        self.reference_markers2 = []
        self.frame_count = 0
        self.calibrated = False
        
        # Detection timer
        self.detection_timer = self.create_timer(0.1, self.process_detection)  # 10 Hz
        
        self.get_logger().info("🔍 GelSight Object Detector initialized")
        self.get_logger().info(f"📐 Parameters:")
        self.get_logger().info(f"   - Detection threshold: {self.detection_threshold}")
        self.get_logger().info(f"   - Reference frames: {self.reference_frames}")
        self.get_logger().info(f"   - Deformation threshold: {self.deformation_threshold}")
    
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
    
    def detect_markers(self, frame, camera_id="unknown"):
        """Detect circular markers in the GelSight surface"""
        if frame is None:
            self.get_logger().debug(f"📷 Camera {camera_id}: Frame is None")
            return []
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # Log frame info
        h, w = frame.shape[:2]
        self.get_logger().debug(f"📷 Camera {camera_id}: Processing frame {w}x{h}")
        
        # Use HoughCircles to detect circular markers
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=20,  # Minimum distance between circle centers
            param1=50,   # Upper threshold for edge detection
            param2=30,   # Accumulator threshold for center detection
            minRadius=5, # Minimum circle radius
            maxRadius=25 # Maximum circle radius
        )
        
        markers = []
        total_circles_found = 0
        filtered_out = 0
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            total_circles_found = len(circles[0, :])
            
            for circle in circles[0, :]:
                x, y, r = circle
                # Filter by area
                area = np.pi * r * r
                if self.min_marker_area <= area <= self.max_marker_area:
                    markers.append({
                        'center': (x, y),
                        'radius': r,
                        'area': area
                    })
                else:
                    filtered_out += 1
        
        self.get_logger().debug(f"🔍 Camera {camera_id}: Found {total_circles_found} circles, "
                               f"{len(markers)} valid markers, {filtered_out} filtered out")
        
        if len(markers) > 0:
            self.get_logger().debug(f"📍 Camera {camera_id}: Markers at positions: "
                                   f"{[m['center'] for m in markers[:5]]}")  # Show first 5
        
        return markers
    
    def calculate_deformation(self, current_markers, reference_markers):
        """Calculate deformation between current and reference markers"""
        if not reference_markers or not current_markers:
            return 0.0, []
        
        total_deformation = 0.0
        deformed_markers = []
        matched_count = 0
        
        # For each reference marker, find the closest current marker
        for ref_marker in reference_markers:
            ref_center = ref_marker['center']
            ref_radius = ref_marker['radius']
            
            min_distance = float('inf')
            closest_marker = None
            
            # Find closest current marker
            for curr_marker in current_markers:
                curr_center = curr_marker['center']
                distance = np.sqrt((ref_center[0] - curr_center[0])**2 + 
                                 (ref_center[1] - curr_center[1])**2)
                
                if distance < min_distance and distance < 30:  # Max displacement threshold
                    min_distance = distance
                    closest_marker = curr_marker
            
            if closest_marker:
                matched_count += 1
                curr_center = closest_marker['center']
                curr_radius = closest_marker['radius']
                
                # Calculate positional displacement
                displacement = min_distance
                
                # Calculate shape deformation (radius change)
                radius_change = abs(ref_radius - curr_radius) / ref_radius
                
                # Combined deformation metric
                marker_deformation = (displacement / 30.0) + radius_change
                total_deformation += marker_deformation
                
                if marker_deformation > self.deformation_threshold:
                    deformed_markers.append({
                        'ref_center': ref_center,
                        'curr_center': curr_center,
                        'displacement': displacement,
                        'radius_change': radius_change,
                        'deformation': marker_deformation
                    })
        
        # Average deformation
        if matched_count > 0:
            avg_deformation = total_deformation / matched_count
        else:
            avg_deformation = 0.0
        
        return avg_deformation, deformed_markers
    
    def process_detection(self):
        """Main detection processing loop"""
        with self.frame_lock:
            current_frame1 = self.frame1.copy() if self.frame1 is not None else None
            current_frame2 = self.frame2.copy() if self.frame2 is not None else None
        
        if current_frame1 is None and current_frame2 is None:
            return
        
        # Calibration phase - collect reference frames
        if not self.calibrated:
            self.frame_count += 1
            
            if self.frame_count <= self.reference_frames:
                status_msg = String()
                status_msg.data = f"Calibrating... {self.frame_count}/{self.reference_frames}"
                self.status_pub.publish(status_msg)
                
                # Store reference frame (use the last frame as reference)
                if current_frame1 is not None:
                    self.reference_frame1 = current_frame1
                if current_frame2 is not None:
                    self.reference_frame2 = current_frame2
                
                return
            
            elif self.frame_count == self.reference_frames + 1:
                # Detect reference markers
                if self.reference_frame1 is not None:
                    self.reference_markers1 = self.detect_markers(self.reference_frame1)
                    self.get_logger().info(f"📍 Camera 1: Found {len(self.reference_markers1)} reference markers")
                
                if self.reference_frame2 is not None:
                    self.reference_markers2 = self.detect_markers(self.reference_frame2)
                    self.get_logger().info(f"📍 Camera 2: Found {len(self.reference_markers2)} reference markers")
                
                self.calibrated = True
                self.get_logger().info("✅ Calibration complete - Starting object detection")
                
                status_msg = String()
                status_msg.data = "Calibrated - Ready for detection"
                self.status_pub.publish(status_msg)
                return
        
        # Detection phase
        if not self.calibrated:
            return
        
        deformation1 = 0.0
        deformation2 = 0.0
        deformed_markers1 = []
        deformed_markers2 = []
        
        # Process camera 1
        if current_frame1 is not None and self.reference_markers1:
            current_markers1 = self.detect_markers(current_frame1)
            deformation1, deformed_markers1 = self.calculate_deformation(
                current_markers1, self.reference_markers1)
        
        # Process camera 2
        if current_frame2 is not None and self.reference_markers2:
            current_markers2 = self.detect_markers(current_frame2)
            deformation2, deformed_markers2 = self.calculate_deformation(
                current_markers2, self.reference_markers2)
        
        # Combined deformation
        max_deformation = max(deformation1, deformation2)
        avg_deformation = (deformation1 + deformation2) / 2.0
        
        # Object detection decision
        object_detected = bool(max_deformation > self.detection_threshold)
        confidence = float(min(max_deformation, 1.0))
        
        # Determine location (which camera has more deformation)
        if deformation1 > deformation2:
            location = 1  # Camera 1 (left gripper finger)
        elif deformation2 > deformation1:
            location = 2  # Camera 2 (right gripper finger)
        else:
            location = 0  # Both or neither
        
        # Publish results
        detection_msg = Bool()
        detection_msg.data = object_detected
        self.object_detected_pub.publish(detection_msg)
        
        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.confidence_pub.publish(confidence_msg)
        
        deformation_msg = Float32()
        deformation_msg.data = avg_deformation
        self.deformation_pub.publish(deformation_msg)
        
        location_msg = Int32()
        location_msg.data = location
        self.location_pub.publish(location_msg)
        
        # Status message
        status_msg = String()
        if object_detected:
            status_msg.data = f"OBJECT DETECTED - Confidence: {confidence:.2f}, Deformation: {avg_deformation:.3f}"
        else:
            status_msg.data = f"No object - Deformation: {avg_deformation:.3f}"
        self.status_pub.publish(status_msg)
        
        # Debug info
        if object_detected:
            self.get_logger().info(f"🎯 Object detected! Confidence: {confidence:.2f}, "
                                 f"Cam1 deformation: {deformation1:.3f}, "
                                 f"Cam2 deformation: {deformation2:.3f}")
    
    def reset_calibration(self):
        """Reset calibration for new reference"""
        self.calibrated = False
        self.frame_count = 0
        self.reference_frame1 = None
        self.reference_frame2 = None
        self.reference_markers1 = []
        self.reference_markers2 = []
        self.get_logger().info("🔄 Calibration reset")


def main():
    """Main function"""
    rclpy.init()
    
    try:
        detector = GelSightObjectDetector()
        
        print("\n" + "="*60)
        print("🔍 GelSight Object Detector")
        print("="*60)
        print("📡 Listening to camera topics:")
        print("   • /gelsight_cam1/image_raw")
        print("   • /gelsight_cam2/image_raw")
        print("\n📊 Publishing detection results to:")
        print("   • /tension_control/object_detected")
        print("   • /tension_control/object_confidence")
        print("   • /tension_control/deformation_level")
        print("   • /tension_control/detection_status")
        print("   • /tension_control/object_location")
        print("\n🎯 Detection process:")
        print("   1. Calibration phase (10 frames)")
        print("   2. Marker detection and tracking")
        print("   3. Deformation analysis")
        print("   4. Object detection decision")
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
