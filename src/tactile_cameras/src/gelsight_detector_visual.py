#!/usr/bin/env python3
"""
GelSight Object Detection with Visual Debug
Shows marker detection and deformation visualization
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


class GelSightDetectorVisual(Node):
    """Visual debugging version of GelSight object detector"""
    
    def __init__(self):
        super().__init__('gelsight_detector_visual')
        
        # Parameters - same as main detector
        self.declare_parameter('detection_threshold', 0.15)
        self.declare_parameter('reference_frames', 10)
        self.declare_parameter('deformation_threshold', 0.2)
        
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.reference_frames = self.get_parameter('reference_frames').value
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
        self.status_pub = self.create_publisher(String, '/tension_control/detection_status', 10)
        
        # State variables
        self.frame1 = None
        self.frame2 = None
        self.reference_markers1 = []
        self.reference_markers2 = []
        self.frame_count = 0
        self.calibrated = False
        
        # OpenCV windows
        cv2.namedWindow('GelSight Detection Cam1', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('GelSight Detection Cam2', cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow('GelSight Detection Cam1', 100, 100)
        cv2.moveWindow('GelSight Detection Cam2', 700, 100)
        
        # Detection timer
        self.detection_timer = self.create_timer(0.1, self.process_detection)
        
        self.get_logger().info("🔍 GelSight Visual Detector initialized")
    
    def camera1_callback(self, msg):
        try:
            with self.frame_lock:
                self.frame1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera 1: {e}")
    
    def camera2_callback(self, msg):
        try:
            with self.frame_lock:
                self.frame2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera 2: {e}")
    
    def detect_markers(self, frame):
        """Detect circular markers"""
        if frame is None:
            return []
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=20,
            param1=50,
            param2=30,
            minRadius=5,
            maxRadius=25
        )
        
        markers = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle
                area = np.pi * r * r
                if 50 <= area <= 500:  # Filter by area
                    markers.append({
                        'center': (x, y),
                        'radius': r,
                        'area': area
                    })
        
        return markers
    
    def draw_markers(self, frame, markers, color=(0, 255, 0), thickness=2):
        """Draw markers on frame"""
        if frame is None:
            return frame
        
        display_frame = frame.copy()
        for marker in markers:
            center = marker['center']
            radius = marker['radius']
            cv2.circle(display_frame, center, radius, color, thickness)
            cv2.circle(display_frame, center, 2, color, -1)  # Center dot
        
        return display_frame
    
    def calculate_deformation(self, current_markers, reference_markers):
        """Calculate deformation with detailed info"""
        if not reference_markers or not current_markers:
            return 0.0, []
        
        total_deformation = 0.0
        deformed_markers = []
        matched_count = 0
        
        for ref_marker in reference_markers:
            ref_center = ref_marker['center']
            ref_radius = ref_marker['radius']
            
            min_distance = float('inf')
            closest_marker = None
            
            for curr_marker in current_markers:
                curr_center = curr_marker['center']
                distance = np.sqrt((ref_center[0] - curr_center[0])**2 + 
                                 (ref_center[1] - curr_center[1])**2)
                
                if distance < min_distance and distance < 30:
                    min_distance = distance
                    closest_marker = curr_marker
            
            if closest_marker:
                matched_count += 1
                curr_center = closest_marker['center']
                curr_radius = closest_marker['radius']
                
                displacement = min_distance
                radius_change = abs(ref_radius - curr_radius) / ref_radius
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
        
        avg_deformation = total_deformation / matched_count if matched_count > 0 else 0.0
        return avg_deformation, deformed_markers
    
    def draw_deformation(self, frame, deformed_markers):
        """Draw deformation visualization"""
        if frame is None:
            return frame
        
        display_frame = frame.copy()
        
        for marker in deformed_markers:
            ref_center = marker['ref_center']
            curr_center = marker['curr_center']
            
            # Draw displacement arrow
            cv2.arrowedLine(display_frame, ref_center, curr_center, (0, 0, 255), 3, tipLength=0.3)
            
            # Draw reference position (blue)
            cv2.circle(display_frame, ref_center, 8, (255, 0, 0), 2)
            
            # Draw current position (red)
            cv2.circle(display_frame, curr_center, 8, (0, 0, 255), 2)
            
            # Add deformation text
            cv2.putText(display_frame, f"{marker['deformation']:.2f}", 
                       (curr_center[0] + 10, curr_center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return display_frame
    
    def process_detection(self):
        """Main detection with visualization"""
        with self.frame_lock:
            current_frame1 = self.frame1.copy() if self.frame1 is not None else None
            current_frame2 = self.frame2.copy() if self.frame2 is not None else None
        
        if current_frame1 is None and current_frame2 is None:
            return
        
        # Calibration phase
        if not self.calibrated:
            self.frame_count += 1
            
            if self.frame_count <= self.reference_frames:
                # Show calibration progress
                if current_frame1 is not None:
                    display1 = current_frame1.copy()
                    cv2.putText(display1, f"Calibrating... {self.frame_count}/{self.reference_frames}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    cv2.imshow('GelSight Detection Cam1', cv2.resize(display1, (480, 360)))
                
                if current_frame2 is not None:
                    display2 = current_frame2.copy()
                    cv2.putText(display2, f"Calibrating... {self.frame_count}/{self.reference_frames}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    cv2.imshow('GelSight Detection Cam2', cv2.resize(display2, (480, 360)))
                
                cv2.waitKey(1)
                return
            
            elif self.frame_count == self.reference_frames + 1:
                # Detect reference markers
                if current_frame1 is not None:
                    self.reference_markers1 = self.detect_markers(current_frame1)
                    self.get_logger().info(f"📍 Camera 1: {len(self.reference_markers1)} reference markers")
                
                if current_frame2 is not None:
                    self.reference_markers2 = self.detect_markers(current_frame2)
                    self.get_logger().info(f"📍 Camera 2: {len(self.reference_markers2)} reference markers")
                
                self.calibrated = True
                self.get_logger().info("✅ Calibration complete")
                return
        
        # Detection phase with visualization
        if not self.calibrated:
            return
        
        deformation1 = 0.0
        deformation2 = 0.0
        deformed_markers1 = []
        deformed_markers2 = []
        
        # Process camera 1
        if current_frame1 is not None:
            current_markers1 = self.detect_markers(current_frame1)
            
            # Draw current markers (green)
            display1 = self.draw_markers(current_frame1, current_markers1, (0, 255, 0), 2)
            
            # Draw reference markers (blue)
            display1 = self.draw_markers(display1, self.reference_markers1, (255, 0, 0), 1)
            
            if self.reference_markers1:
                deformation1, deformed_markers1 = self.calculate_deformation(
                    current_markers1, self.reference_markers1)
                
                # Draw deformation
                display1 = self.draw_deformation(display1, deformed_markers1)
            
            # Add status text
            object_detected1 = deformation1 > self.detection_threshold
            status_color = (0, 255, 0) if not object_detected1 else (0, 0, 255)
            cv2.putText(display1, f"Cam1 - Deformation: {deformation1:.3f}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            if object_detected1:
                cv2.putText(display1, "OBJECT DETECTED!", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow('GelSight Detection Cam1', cv2.resize(display1, (480, 360)))
        
        # Process camera 2
        if current_frame2 is not None:
            current_markers2 = self.detect_markers(current_frame2)
            
            # Draw current markers (green)
            display2 = self.draw_markers(current_frame2, current_markers2, (0, 255, 0), 2)
            
            # Draw reference markers (blue)
            display2 = self.draw_markers(display2, self.reference_markers2, (255, 0, 0), 1)
            
            if self.reference_markers2:
                deformation2, deformed_markers2 = self.calculate_deformation(
                    current_markers2, self.reference_markers2)
                
                # Draw deformation
                display2 = self.draw_deformation(display2, deformed_markers2)
            
            # Add status text
            object_detected2 = deformation2 > self.detection_threshold
            status_color = (0, 255, 0) if not object_detected2 else (0, 0, 255)
            cv2.putText(display2, f"Cam2 - Deformation: {deformation2:.3f}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            if object_detected2:
                cv2.putText(display2, "OBJECT DETECTED!", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow('GelSight Detection Cam2', cv2.resize(display2, (480, 360)))
        
        cv2.waitKey(1)
        
        # Publish detection results
        max_deformation = max(deformation1, deformation2)
        object_detected = max_deformation > self.detection_threshold
        confidence = min(max_deformation, 1.0)
        
        detection_msg = Bool()
        detection_msg.data = object_detected
        self.object_detected_pub.publish(detection_msg)
        
        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.confidence_pub.publish(confidence_msg)
        
        status_msg = String()
        if object_detected:
            status_msg.data = f"OBJECT DETECTED - Confidence: {confidence:.2f}"
        else:
            status_msg.data = f"No object - Max deformation: {max_deformation:.3f}"
        self.status_pub.publish(status_msg)
    
    def destroy_node(self):
        """Clean up"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    
    try:
        detector = GelSightDetectorVisual()
        
        print("\n" + "="*60)
        print("🔍 GelSight Visual Object Detector")
        print("="*60)
        print("👁️  Visual debugging with marker tracking")
        print("🔵 Blue circles: Reference markers (baseline)")
        print("🟢 Green circles: Current markers")
        print("🔴 Red arrows: Deformation vectors")
        print("\n⌨️  Press ESC in any window or Ctrl+C to stop")
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
