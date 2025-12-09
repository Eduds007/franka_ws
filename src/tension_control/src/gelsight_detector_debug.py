#!/usr/bin/env python3
"""
GelSight Object Detection with Enhanced Logging
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


class GelSightObjectDetectorDebug(Node):
    """Enhanced object detection with detailed logging"""
    
    def __init__(self):
        super().__init__('gelsight_object_detector_debug')
        
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
        
        # Debug counters
        self.detection_cycle = 0
        self.last_log_time = time.time()
        
        # Detection timer
        self.detection_timer = self.create_timer(0.1, self.process_detection)  # 10 Hz
        
        self.get_logger().info("🔍 GelSight Object Detector (DEBUG MODE) initialized")
        self.get_logger().info(f"📐 Parameters:")
        self.get_logger().info(f"   - Detection threshold: {self.detection_threshold}")
        self.get_logger().info(f"   - Reference frames: {self.reference_frames}")
        self.get_logger().info(f"   - Deformation threshold: {self.deformation_threshold}")
        self.get_logger().info(f"   - Min marker area: {self.min_marker_area}")
        self.get_logger().info(f"   - Max marker area: {self.max_marker_area}")
    
    def camera1_callback(self, msg):
        """Process camera 1 frames"""
        try:
            with self.frame_lock:
                self.frame1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.get_logger().debug(f"📸 Camera 1: Received frame {self.frame1.shape}")
        except Exception as e:
            self.get_logger().error(f"❌ Error processing camera 1: {e}")
    
    def camera2_callback(self, msg):
        """Process camera 2 frames"""
        try:
            with self.frame_lock:
                self.frame2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.get_logger().debug(f"📸 Camera 2: Received frame {self.frame2.shape}")
        except Exception as e:
            self.get_logger().error(f"❌ Error processing camera 2: {e}")
    
    def detect_markers(self, frame, camera_name="Unknown"):
        """Detect circular markers with detailed logging"""
        if frame is None:
            self.get_logger().warn(f"🚫 [{camera_name}] Frame is None, cannot detect markers")
            return []
        
        # Log frame info
        h, w = frame.shape[:2]
        self.get_logger().debug(f"📷 [{camera_name}] Processing frame: {w}x{h}")
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Log grayscale stats
        mean_intensity = np.mean(gray)
        std_intensity = np.std(gray)
        self.get_logger().debug(f"📊 [{camera_name}] Gray stats - Mean: {mean_intensity:.1f}, Std: {std_intensity:.1f}")
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
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
        raw_circles_count = 0
        filtered_out_small = 0
        filtered_out_large = 0
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            raw_circles_count = len(circles[0, :])
            
            self.get_logger().debug(f"🔵 [{camera_name}] HoughCircles found {raw_circles_count} raw circles")
            
            for i, circle in enumerate(circles[0, :]):
                x, y, r = circle
                area = np.pi * r * r
                
                if area < self.min_marker_area:
                    filtered_out_small += 1
                    self.get_logger().debug(f"   Circle {i}: area {area:.1f} < {self.min_marker_area} (too small)")
                elif area > self.max_marker_area:
                    filtered_out_large += 1
                    self.get_logger().debug(f"   Circle {i}: area {area:.1f} > {self.max_marker_area} (too large)")
                else:
                    markers.append({
                        'center': (x, y),
                        'radius': r,
                        'area': area
                    })
                    self.get_logger().debug(f"   ✅ Circle {i}: center=({x},{y}), radius={r}, area={area:.1f}")
        else:
            self.get_logger().debug(f"⭕ [{camera_name}] HoughCircles found no circles")
        
        self.get_logger().info(f"📍 [{camera_name}] Marker detection summary:")
        self.get_logger().info(f"   Raw circles found: {raw_circles_count}")
        self.get_logger().info(f"   Valid markers: {len(markers)}")
        self.get_logger().info(f"   Filtered out (too small): {filtered_out_small}")
        self.get_logger().info(f"   Filtered out (too large): {filtered_out_large}")
        
        return markers
    
    def calculate_deformation(self, current_markers, reference_markers, camera_name="Unknown"):
        """Calculate deformation with detailed logging"""
        if not reference_markers:
            self.get_logger().warn(f"🚫 [{camera_name}] No reference markers available")
            return 0.0, []
        
        if not current_markers:
            self.get_logger().warn(f"🚫 [{camera_name}] No current markers detected")
            return 0.0, []
        
        self.get_logger().debug(f"🔍 [{camera_name}] Calculating deformation:")
        self.get_logger().debug(f"   Reference markers: {len(reference_markers)}")
        self.get_logger().debug(f"   Current markers: {len(current_markers)}")
        
        total_deformation = 0.0
        deformed_markers = []
        matched_count = 0
        unmatched_ref = 0
        
        # For each reference marker, find the closest current marker
        for i, ref_marker in enumerate(reference_markers):
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
                
                self.get_logger().debug(f"   Marker {i}: displacement={displacement:.2f}, "
                                       f"radius_change={radius_change:.3f}, "
                                       f"deformation={marker_deformation:.3f}")
                
                if marker_deformation > self.deformation_threshold:
                    deformed_markers.append({
                        'ref_center': ref_center,
                        'curr_center': curr_center,
                        'displacement': displacement,
                        'radius_change': radius_change,
                        'deformation': marker_deformation
                    })
                    self.get_logger().info(f"🔴 [{camera_name}] DEFORMED MARKER {i}: "
                                          f"moved {displacement:.2f}px, "
                                          f"radius changed {radius_change*100:.1f}%")
            else:
                unmatched_ref += 1
                self.get_logger().debug(f"   ❌ Marker {i}: No matching current marker found")
        
        # Average deformation
        if matched_count > 0:
            avg_deformation = total_deformation / matched_count
        else:
            avg_deformation = 0.0
        
        self.get_logger().info(f"📊 [{camera_name}] Deformation analysis:")
        self.get_logger().info(f"   Matched markers: {matched_count}/{len(reference_markers)}")
        self.get_logger().info(f"   Unmatched reference markers: {unmatched_ref}")
        self.get_logger().info(f"   Significantly deformed markers: {len(deformed_markers)}")
        self.get_logger().info(f"   Average deformation: {avg_deformation:.4f}")
        
        return avg_deformation, deformed_markers
    
    def process_detection(self):
        """Main detection processing loop with enhanced logging"""
        self.detection_cycle += 1
        current_time = time.time()
        
        # Log processing frequency every 5 seconds
        if current_time - self.last_log_time > 5.0:
            freq = self.detection_cycle / (current_time - self.last_log_time + 1e-6)
            self.get_logger().info(f"⏱️  Detection frequency: {freq:.1f} Hz")
            self.last_log_time = current_time
            self.detection_cycle = 0
        
        with self.frame_lock:
            current_frame1 = self.frame1.copy() if self.frame1 is not None else None
            current_frame2 = self.frame2.copy() if self.frame2 is not None else None
        
        # Log frame availability
        frame1_available = current_frame1 is not None
        frame2_available = current_frame2 is not None
        
        if not frame1_available and not frame2_available:
            if self.detection_cycle % 100 == 1:  # Log every ~10 seconds at 10Hz
                self.get_logger().warn("📭 No frames available from either camera")
            return
        
        self.get_logger().debug(f"📦 Frame availability - Cam1: {frame1_available}, Cam2: {frame2_available}")
        
        # Calibration phase
        if not self.calibrated:
            self.frame_count += 1
            
            self.get_logger().info(f"🔄 Calibration progress: {self.frame_count}/{self.reference_frames}")
            
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
                self.get_logger().info("🔍 Starting reference marker detection...")
                
                # Detect reference markers
                if self.reference_frame1 is not None:
                    self.reference_markers1 = self.detect_markers(self.reference_frame1, "Cam1_REF")
                    self.get_logger().info(f"📍 Camera 1 REFERENCE: {len(self.reference_markers1)} markers")
                
                if self.reference_frame2 is not None:
                    self.reference_markers2 = self.detect_markers(self.reference_frame2, "Cam2_REF")
                    self.get_logger().info(f"📍 Camera 2 REFERENCE: {len(self.reference_markers2)} markers")
                
                total_ref_markers = len(self.reference_markers1) + len(self.reference_markers2)
                if total_ref_markers == 0:
                    self.get_logger().error("❌ NO REFERENCE MARKERS DETECTED! "
                                           "Check camera feeds and marker visibility.")
                else:
                    self.get_logger().info(f"✅ Total reference markers: {total_ref_markers}")
                
                self.calibrated = True
                self.get_logger().info("🎯 Calibration complete - Starting object detection")
                
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
        if current_frame1 is not None:
            if self.reference_markers1:
                current_markers1 = self.detect_markers(current_frame1, "Cam1")
                deformation1, deformed_markers1 = self.calculate_deformation(
                    current_markers1, self.reference_markers1, "Cam1")
            else:
                self.get_logger().debug("⚠️  Camera 1: No reference markers available")
        
        # Process camera 2
        if current_frame2 is not None:
            if self.reference_markers2:
                current_markers2 = self.detect_markers(current_frame2, "Cam2")
                deformation2, deformed_markers2 = self.calculate_deformation(
                    current_markers2, self.reference_markers2, "Cam2")
            else:
                self.get_logger().debug("⚠️  Camera 2: No reference markers available")
        
        # Combined deformation analysis
        max_deformation = max(deformation1, deformation2)
        avg_deformation = (deformation1 + deformation2) / 2.0
        
        self.get_logger().info(f"🎯 DETECTION RESULTS:")
        self.get_logger().info(f"   Cam1 deformation: {deformation1:.4f}")
        self.get_logger().info(f"   Cam2 deformation: {deformation2:.4f}")
        self.get_logger().info(f"   Max deformation: {max_deformation:.4f}")
        self.get_logger().info(f"   Avg deformation: {avg_deformation:.4f}")
        self.get_logger().info(f"   Detection threshold: {self.detection_threshold}")
        
        # Object detection decision
        object_detected = bool(max_deformation > self.detection_threshold)
        confidence = float(min(max_deformation, 1.0))
        
        # Determine location
        if deformation1 > deformation2:
            location = 1  # Camera 1
        elif deformation2 > deformation1:
            location = 2  # Camera 2
        else:
            location = 0  # Both or neither
        
        if object_detected:
            self.get_logger().info(f"🚨 OBJECT DETECTED! Confidence: {confidence:.3f}, Location: Cam{location}")
        
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
            status_msg.data = f"OBJECT DETECTED - Conf:{confidence:.3f} Def:{avg_deformation:.4f} Loc:Cam{location}"
        else:
            status_msg.data = f"Monitoring - Deformation:{avg_deformation:.4f} (threshold:{self.detection_threshold})"
        self.status_pub.publish(status_msg)


def main():
    """Main function"""
    rclpy.init()
    
    try:
        detector = GelSightObjectDetectorDebug()
        
        print("\n" + "="*60)
        print("🔍 GelSight Object Detector - DEBUG MODE")
        print("="*60)
        print("📊 Enhanced logging enabled")
        print("📡 Listening to camera topics:")
        print("   • /gelsight_cam1/image_raw")
        print("   • /gelsight_cam2/image_raw")
        print("\n🎯 Detection process with detailed logs:")
        print("   1. Frame reception logging")
        print("   2. Marker detection with statistics")
        print("   3. Deformation calculation details")
        print("   4. Decision process transparency")
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
