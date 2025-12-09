#!/usr/bin/env python3
"""
Tactile Detection Service - Debug Version
Simplified version for testing without real cameras
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from threading import Lock


class TactileDetectionDebug(Node):
    """Debug version of tactile detection for testing"""
    
    def __init__(self):
        super().__init__('tactile_detection_debug')
        
        self.bridge = CvBridge()
        self.frame_lock = Lock()
        
        # Parameters
        self.detection_threshold = 0.3
        self.background_learn_frames = 10  # Reduced for testing
        
        # Publishers
        self.object_detected_pub = self.create_publisher(Bool, '/tension_control/object_detected', 10)
        self.object_confidence_pub = self.create_publisher(Float32, '/tension_control/object_confidence', 10)
        self.detection_status_pub = self.create_publisher(String, '/tension_control/detection_status', 10)
        
        # State
        self.background_learned = False
        self.frames_received = 0
        
        # Create fake camera feeds for testing
        self.create_fake_camera_feeds()
        
        # Processing timer
        self.detection_timer = self.create_timer(0.5, self.process_detection_debug)
        
        self.get_logger().info("🧪 Tactile Detection Debug Service started")
    
    def create_fake_camera_feeds(self):
        """Create fake camera feeds for testing"""
        # Create fake background frames
        fake_frame1 = np.ones((480, 640, 3), dtype=np.uint8) * 128  # Gray background
        fake_frame2 = np.ones((480, 640, 3), dtype=np.uint8) * 120  # Slightly different gray
        
        # Add some noise to make it more realistic
        noise1 = np.random.normal(0, 5, fake_frame1.shape).astype(np.uint8)
        noise2 = np.random.normal(0, 5, fake_frame2.shape).astype(np.uint8)
        
        self.fake_frame1 = np.clip(fake_frame1 + noise1, 0, 255)
        self.fake_frame2 = np.clip(fake_frame2 + noise2, 0, 255)
        
        # Simulate background learning
        self.learn_fake_background()
    
    def learn_fake_background(self):
        """Learn background using fake frames"""
        self.get_logger().info("📚 Learning fake background...")
        
        # Create multiple slightly different background frames
        self.background_frames1 = []
        self.background_frames2 = []
        
        for i in range(self.background_learn_frames):
            # Add slight variations to each frame
            variation1 = np.random.normal(0, 3, self.fake_frame1.shape).astype(np.int16)
            variation2 = np.random.normal(0, 3, self.fake_frame2.shape).astype(np.int16)
            
            frame1_varied = np.clip(self.fake_frame1.astype(np.int16) + variation1, 0, 255).astype(np.uint8)
            frame2_varied = np.clip(self.fake_frame2.astype(np.int16) + variation2, 0, 255).astype(np.uint8)
            
            self.background_frames1.append(frame1_varied)
            self.background_frames2.append(frame2_varied)
        
        self.background_learned = True
        self.get_logger().info(f"✅ Fake background learned with {len(self.background_frames1)} frames")
    
    def simulate_object_detection(self):
        """Simulate object detection for testing"""
        # Simulate varying confidence based on time
        current_time = time.time()
        
        # Create sine wave for confidence simulation
        confidence1 = 0.5 + 0.3 * np.sin(current_time * 0.5)  # Slow sine wave
        confidence2 = 0.4 + 0.2 * np.cos(current_time * 0.7)  # Different frequency
        
        # Add some randomness
        confidence1 += np.random.normal(0, 0.1)
        confidence2 += np.random.normal(0, 0.1)
        
        # Clamp to valid range
        confidence1 = max(0.0, min(1.0, confidence1))
        confidence2 = max(0.0, min(1.0, confidence2))
        
        # Determine detection
        detected1 = confidence1 > self.detection_threshold
        detected2 = confidence2 > self.detection_threshold
        overall_detected = detected1 or detected2
        max_confidence = max(confidence1, confidence2)
        
        return {
            'cam1_detected': detected1,
            'cam2_detected': detected2,
            'cam1_confidence': confidence1,
            'cam2_confidence': confidence2,
            'overall_detected': overall_detected,
            'max_confidence': max_confidence
        }
    
    def process_detection_debug(self):
        """Process detection in debug mode"""
        if not self.background_learned:
            return
        
        # Simulate detection
        results = self.simulate_object_detection()
        
        # Publish results
        self.object_detected_pub.publish(Bool(data=results['overall_detected']))
        self.object_confidence_pub.publish(Float32(data=results['max_confidence']))
        
        # Create status message
        status = f"DEBUG | Cam1: {'✅' if results['cam1_detected'] else '❌'} " \
                f"({results['cam1_confidence']:.3f}) | " \
                f"Cam2: {'✅' if results['cam2_detected'] else '❌'} " \
                f"({results['cam2_confidence']:.3f}) | " \
                f"Overall: {'DETECTED' if results['overall_detected'] else 'CLEAR'}"
        
        self.detection_status_pub.publish(String(data=status))
        
        # Log occasionally
        self.frames_received += 1
        if self.frames_received % 10 == 0:  # Every 5 seconds at 0.5Hz
            self.get_logger().info(f"🔍 {status}")


def main():
    """Main function"""
    rclpy.init()
    
    try:
        debug_service = TactileDetectionDebug()
        
        debug_service.get_logger().info("🚀 Debug Service running - simulating tactile detection")
        debug_service.get_logger().info("📋 Publishing to:")
        debug_service.get_logger().info("  - /tension_control/object_detected")
        debug_service.get_logger().info("  - /tension_control/object_confidence")
        debug_service.get_logger().info("  - /tension_control/detection_status")
        
        rclpy.spin(debug_service)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'debug_service' in locals():
            debug_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
