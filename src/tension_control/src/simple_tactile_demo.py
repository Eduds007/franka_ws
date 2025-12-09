#!/usr/bin/env python3
"""
Simple Tactile Object Detection Demo
Works without real cameras for testing purposes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, Int32
import cv2
import numpy as np
import time
import threading


class SimpleTactileDemo(Node):
    """Simple demo node that simulates tactile detection"""
    
    def __init__(self):
        super().__init__('simple_tactile_demo')
        
        # Publishers
        self.object_detected_pub = self.create_publisher(Bool, '/tension_control/object_detected', 10)
        self.object_confidence_pub = self.create_publisher(Float32, '/tension_control/object_confidence', 10)
        self.grip_force_pub = self.create_publisher(Float32, '/tension_control/recommended_grip_force', 10)
        self.status_pub = self.create_publisher(String, '/tension_control/detection_status', 10)
        self.location_pub = self.create_publisher(Int32, '/tension_control/object_location', 10)
        
        # Simulation parameters
        self.detection_probability = 0.02  # 2% chance per frame
        self.min_grip_force = 5.0
        self.max_grip_force = 30.0
        
        # State
        self.current_detection = False
        self.current_confidence = 0.0
        self.detection_duration = 0
        self.max_detection_duration = 50  # frames
        
        # Timer for simulation
        self.sim_timer = self.create_timer(0.1, self.simulate_detection)  # 10Hz
        
        # Background learning simulation
        self.background_learned = False
        self.learning_timer = self.create_timer(3.0, self.simulate_background_learning)
        
        self.get_logger().info("🎮 Simple Tactile Demo initialized")
        self.get_logger().info("📋 Publishing to:")
        self.get_logger().info("  - /tension_control/object_detected")
        self.get_logger().info("  - /tension_control/object_confidence")  
        self.get_logger().info("  - /tension_control/recommended_grip_force")
        self.get_logger().info("  - /tension_control/detection_status")
        self.get_logger().info("  - /tension_control/object_location")
    
    def simulate_background_learning(self):
        """Simulate background learning process"""
        if not self.background_learned:
            self.get_logger().info("📚 Simulating background learning...")
            time.sleep(2.0)  # Simulate learning time
            self.background_learned = True
            self.get_logger().info("✅ Background learning complete!")
            self.learning_timer.cancel()
    
    def simulate_detection(self):
        """Simulate object detection"""
        if not self.background_learned:
            return
        
        # Simulate detection events
        if not self.current_detection:
            # Random chance of detecting object
            if np.random.random() < self.detection_probability:
                self.current_detection = True
                self.current_confidence = np.random.uniform(0.4, 0.9)
                self.detection_duration = 0
                self.get_logger().info(f"🎯 Object detected! Confidence: {self.current_confidence:.3f}")
        else:
            # Keep detection for some time
            self.detection_duration += 1
            if self.detection_duration >= self.max_detection_duration:
                self.current_detection = False
                self.current_confidence = 0.0
                self.detection_duration = 0
                self.get_logger().info("❌ Object detection ended")
        
        # Publish results
        self.publish_results()
    
    def publish_results(self):
        """Publish detection results"""
        # Object detection
        self.object_detected_pub.publish(Bool(data=self.current_detection))
        
        # Confidence
        confidence = self.current_confidence if self.current_detection else 0.0
        self.object_confidence_pub.publish(Float32(data=confidence))
        
        # Object location (simulate camera 1)
        location = 1 if self.current_detection else 0
        self.location_pub.publish(Int32(data=location))
        
        # Grip force recommendation
        if self.current_detection:
            grip_force = self.calculate_grip_force(confidence)
            self.grip_force_pub.publish(Float32(data=grip_force))
        
        # Status message
        if self.current_detection:
            status = f"Cam1: ✅ ({confidence:.3f}) | Cam2: ❌ (0.000) | Overall: DETECTED"
        else:
            status = f"Cam1: ❌ (0.000) | Cam2: ❌ (0.000) | Overall: CLEAR"
        
        self.status_pub.publish(String(data=status))
    
    def calculate_grip_force(self, confidence):
        """Calculate recommended grip force"""
        # Higher confidence -> gentler grip
        force_ratio = 1.0 - confidence * 0.7
        grip_force = self.min_grip_force + (self.max_grip_force - self.min_grip_force) * force_ratio
        return max(self.min_grip_force, min(self.max_grip_force, grip_force))


def main():
    """Main function"""
    rclpy.init()
    
    try:
        demo = SimpleTactileDemo()
        demo.get_logger().info("🚀 Simple Tactile Demo running...")
        demo.get_logger().info("   Objects will be randomly detected every ~50 seconds")
        demo.get_logger().info("   Press Ctrl+C to stop")
        
        rclpy.spin(demo)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'demo' in locals():
            demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
