#!/usr/bin/env python3
"""
Tactile Object Detection Service for Tension Control
Lightweight object detection service that integrates with the tension controller
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String, Int32
from geometry_msgs.msg import Wrench
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from threading import Lock


class TactileObjectDetectionService(Node):
    """Lightweight object detection service for tension control integration"""
    
    def __init__(self):
        super().__init__('tactile_object_detection_service')
        
        # Parameters
        self.declare_parameter('detection_threshold', 0.3)
        self.declare_parameter('background_learn_frames', 20)
        self.declare_parameter('min_grip_force', 5.0)
        self.declare_parameter('max_grip_force', 30.0)
        self.declare_parameter('adaptive_grip_enabled', True)
        self.declare_parameter('camera1_topic', '/gelsight_cam1/image_raw')
        self.declare_parameter('camera2_topic', '/gelsight_cam2/image_raw')
        
        # Get parameters
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.background_learn_frames = self.get_parameter('background_learn_frames').value
        self.min_grip_force = self.get_parameter('min_grip_force').value
        self.max_grip_force = self.get_parameter('max_grip_force').value
        self.adaptive_grip_enabled = self.get_parameter('adaptive_grip_enabled').value
        
        # ROS2 Setup
        self.bridge = CvBridge()
        self.frame_lock = Lock()
        
        # Image subscribers
        self.cam1_sub = self.create_subscription(
            Image, self.get_parameter('camera1_topic').value, 
            self.camera1_callback, 10)
        self.cam2_sub = self.create_subscription(
            Image, self.get_parameter('camera2_topic').value, 
            self.camera2_callback, 10)
        
        # Publishers for tension control integration
        self.object_detected_pub = self.create_publisher(Bool, '/tension_control/object_detected', 10)
        self.object_confidence_pub = self.create_publisher(Float32, '/tension_control/object_confidence', 10)
        self.grip_force_recommendation_pub = self.create_publisher(Float32, '/tension_control/recommended_grip_force', 10)
        self.detection_status_pub = self.create_publisher(String, '/tension_control/detection_status', 10)
        self.object_location_pub = self.create_publisher(Int32, '/tension_control/object_location', 10)  # 0=none, 1=cam1, 2=cam2, 3=both
        
        # Service for manual background learning
        from std_srvs.srv import Empty
        self.learn_bg_service = self.create_service(Empty, 'learn_background', self.learn_background_callback)
        
        # Gripper action client
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_gripper/gripper_action')
        
        # Detection state
        self.current_frame1 = None
        self.current_frame2 = None
        self.background_frames1 = []
        self.background_frames2 = []
        self.background_learned = False
        self.frame_count = 0
        
        # Detection results
        self.detection_results = {
            'cam1_detected': False,
            'cam2_detected': False,
            'cam1_confidence': 0.0,
            'cam2_confidence': 0.0,
            'overall_detected': False,
            'max_confidence': 0.0,
            'object_location': 0
        }
        
        # Processing timer
        self.detection_timer = self.create_timer(0.1, self.process_detection_cycle)
        
        # Auto-learn background on startup
        self.auto_learn_timer = self.create_timer(2.0, self.auto_learn_background)
        
        self.get_logger().info("🔍 Tactile Object Detection Service started")
        self.get_logger().info(f"📊 Parameters - Threshold: {self.detection_threshold}, "
                             f"Background frames: {self.background_learn_frames}")
    
    def camera1_callback(self, msg):
        """Process camera 1 frames"""
        try:
            with self.frame_lock:
                self.current_frame1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera 1: {e}")
    
    def camera2_callback(self, msg):
        """Process camera 2 frames"""
        try:
            with self.frame_lock:
                self.current_frame2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera 2: {e}")
    
    def auto_learn_background(self):
        """Automatically learn background on startup"""
        # Check if cameras are available
        if self.current_frame1 is None or self.current_frame2 is None:
            self.get_logger().info("⏳ Waiting for camera feeds before learning background...")
            return  # Keep trying until cameras are available
        
        self.get_logger().info("🎯 Auto-learning background...")
        self.learn_background()
        # Cancel the timer after running once
        self.auto_learn_timer.cancel()
    
    def learn_background_callback(self, request, response):
        """Service callback for learning background"""
        self.learn_background()
        return response
    
    def learn_background(self):
        """Learn background for both cameras"""
        self.get_logger().info("📚 Learning background patterns...")
        
        # Check if cameras are available
        if self.current_frame1 is None or self.current_frame2 is None:
            self.get_logger().error("❌ Cannot learn background - camera feeds not available!")
            return
        
        self.background_frames1 = []
        self.background_frames2 = []
        self.background_learned = False
        
        # Collect background frames
        collected_frames = 0
        start_time = time.time()
        timeout = 15.0  # 15 second timeout
        consecutive_failures = 0
        max_consecutive_failures = 20  # Allow 2 seconds of no frames
        
        self.get_logger().info(f"🎯 Starting to collect {self.background_learn_frames} background frames...")
        
        while collected_frames < self.background_learn_frames and (time.time() - start_time) < timeout:
            with self.frame_lock:
                if self.current_frame1 is not None and self.current_frame2 is not None:
                    # Verify frame quality
                    if self.current_frame1.shape[0] > 0 and self.current_frame1.shape[1] > 0:
                        if self.current_frame2.shape[0] > 0 and self.current_frame2.shape[1] > 0:
                            self.background_frames1.append(self.current_frame1.copy())
                            self.background_frames2.append(self.current_frame2.copy())
                            collected_frames += 1
                            consecutive_failures = 0
                            
                            if collected_frames % 5 == 0:  # Log every 5 frames
                                self.get_logger().info(f"📸 Collected {collected_frames}/{self.background_learn_frames} background frames")
                        else:
                            consecutive_failures += 1
                    else:
                        consecutive_failures += 1
                else:
                    consecutive_failures += 1
            
            if consecutive_failures > max_consecutive_failures:
                self.get_logger().error("❌ Too many consecutive failures - camera feeds may be unstable")
                break
                
            time.sleep(0.1)
        
        if collected_frames >= self.background_learn_frames:
            self.background_learned = True
            self.get_logger().info(f"✅ Background learning complete! Collected {collected_frames} frames")
            self.get_logger().info(f"📊 Camera 1 frame size: {self.background_frames1[0].shape}")
            self.get_logger().info(f"📊 Camera 2 frame size: {self.background_frames2[0].shape}")
        else:
            self.get_logger().warn(f"⚠️ Background learning incomplete. Only collected {collected_frames} frames")
            self.get_logger().warn("💡 Try manually calling the learn_background service when cameras are stable")
    
    def detect_object_in_frame(self, current_frame, background_frames):
        """Detect object presence using background subtraction"""
        if not self.background_learned or len(background_frames) == 0:
            return False, 0.0, None
        
        try:
            # Calculate average background
            bg_sum = np.zeros_like(background_frames[0], dtype=np.float64)
            for bg_frame in background_frames:
                bg_sum += bg_frame.astype(np.float64)
            background = (bg_sum / len(background_frames)).astype(np.uint8)
            
            # Background subtraction
            diff = cv2.absdiff(current_frame, background)
            gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
            
            # Adaptive thresholding
            thresh = cv2.adaptiveThreshold(gray_diff, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                         cv2.THRESH_BINARY, 11, 2)
            
            # Morphological operations
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
            
            # Calculate detection confidence
            changed_pixels = cv2.countNonZero(thresh)
            total_pixels = thresh.shape[0] * thresh.shape[1]
            confidence = changed_pixels / total_pixels
            
            # Detection based on threshold
            detected = confidence > self.detection_threshold
            
            return detected, confidence, thresh
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return False, 0.0, None
    
    def process_detection_cycle(self):
        """Main detection processing cycle"""
        if not self.background_learned:
            return
        
        with self.frame_lock:
            frame1 = self.current_frame1.copy() if self.current_frame1 is not None else None
            frame2 = self.current_frame2.copy() if self.current_frame2 is not None else None
        
        if frame1 is None or frame2 is None:
            return
        
        # Detect objects in both cameras
        detected1, conf1, _ = self.detect_object_in_frame(frame1, self.background_frames1)
        detected2, conf2, _ = self.detect_object_in_frame(frame2, self.background_frames2)
        
        # Update detection results
        self.detection_results = {
            'cam1_detected': detected1,
            'cam2_detected': detected2,
            'cam1_confidence': conf1,
            'cam2_confidence': conf2,
            'overall_detected': detected1 or detected2,
            'max_confidence': max(conf1, conf2),
            'object_location': self.get_object_location(detected1, detected2)
        }
        
        # Publish results
        self.publish_detection_results()
    
    def get_object_location(self, detected1, detected2):
        """Determine object location"""
        if detected1 and detected2:
            return 3  # Both cameras
        elif detected1:
            return 1  # Camera 1 only
        elif detected2:
            return 2  # Camera 2 only
        else:
            return 0  # No object detected
    
    def publish_detection_results(self):
        """Publish all detection results to ROS topics"""
        results = self.detection_results
        
        # Main detection result
        self.object_detected_pub.publish(Bool(data=results['overall_detected']))
        
        # Confidence
        self.object_confidence_pub.publish(Float32(data=results['max_confidence']))
        
        # Object location
        self.object_location_pub.publish(Int32(data=results['object_location']))
        
        # Recommended grip force (if adaptive grip is enabled)
        if self.adaptive_grip_enabled and results['overall_detected']:
            grip_force = self.calculate_adaptive_grip_force(results['max_confidence'])
            self.grip_force_recommendation_pub.publish(Float32(data=grip_force))
        
        # Status message
        location_str = {0: "CLEAR", 1: "CAM1", 2: "CAM2", 3: "BOTH"}[results['object_location']]
        status = f"Detection: {location_str} | Conf: {results['max_confidence']:.3f} | " \
                f"C1: {results['cam1_confidence']:.3f} | C2: {results['cam2_confidence']:.3f}"
        self.detection_status_pub.publish(String(data=status))
        
        # Debug logging (every 2 seconds)
        if self.frame_count % 20 == 0:  # Assuming 10Hz, log every 2 seconds
            self.get_logger().info(
                f"🔍 {status} | Adaptive grip: {self.adaptive_grip_enabled}")
        
        self.frame_count += 1
    
    def calculate_adaptive_grip_force(self, confidence):
        """Calculate adaptive grip force based on detection confidence"""
        # Map confidence to grip force
        # Higher confidence -> more gentle grip (object detected clearly)
        # Lower confidence -> stronger grip (uncertain detection)
        normalized_conf = max(0.0, min(1.0, confidence))
        
        # Inverse relationship: higher confidence -> gentler grip
        force_ratio = 1.0 - (normalized_conf * 0.7)  # Scale down the effect
        grip_force = self.min_grip_force + (self.max_grip_force - self.min_grip_force) * force_ratio
        
        return max(self.min_grip_force, min(self.max_grip_force, grip_force))
    
    def get_current_detection_state(self):
        """Get current detection state (for external use)"""
        return self.detection_results


def main():
    """Main function"""
    rclpy.init()
    
    try:
        detector_service = TactileObjectDetectionService()
        
        # Log startup info
        detector_service.get_logger().info("🚀 Tactile Object Detection Service running")
        detector_service.get_logger().info("📋 Available topics:")
        detector_service.get_logger().info("  - /tension_control/object_detected (Bool)")
        detector_service.get_logger().info("  - /tension_control/object_confidence (Float32)")
        detector_service.get_logger().info("  - /tension_control/recommended_grip_force (Float32)")
        detector_service.get_logger().info("  - /tension_control/detection_status (String)")
        detector_service.get_logger().info("  - /tension_control/object_location (Int32)")
        detector_service.get_logger().info("📞 Available services:")
        detector_service.get_logger().info("  - /learn_background (Empty)")
        
        rclpy.spin(detector_service)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'detector_service' in locals():
            detector_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
