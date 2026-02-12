#!/usr/bin/env python3
"""
Test script for Tactile Contact Analyzer
Monitors contact analysis output and displays results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class ContactAnalyzerMonitor(Node):
    """Monitor for testing tactile contact analyzer."""
    
    def __init__(self):
        super().__init__('contact_analyzer_monitor')
        
        self.bridge = CvBridge()
        
        # State
        self.contact_detected = False
        self.latest_displacement = None
        self.latest_info = None
        self.message_count = 0
        self.start_time = time.time()
        
        # Subscribers
        self.viz_sub = self.create_subscription(
            Image, 
            '/tactile_contact_analyzer/visualization', 
            self.viz_callback, 
            10
        )
        
        self.contact_sub = self.create_subscription(
            Bool,
            '/tactile_contact_analyzer/contact_detected',
            self.contact_callback,
            10
        )
        
        self.displacement_sub = self.create_subscription(
            Vector3,
            '/tactile_contact_analyzer/displacement',
            self.displacement_callback,
            10
        )
        
        self.info_sub = self.create_subscription(
            Float32MultiArray,
            '/tactile_contact_analyzer/contact_info',
            self.info_callback,
            10
        )
        
        # Timer for status display
        self.status_timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info('🔍 Tactile Contact Analyzer Monitor Started')
        print("\n" + "="*70)
        print("🔍 Tactile Contact Analyzer Test Monitor")
        print("="*70)
        print("Monitoring contact analysis from tactile camera...")
        print("")
        print("📊 Topics monitored:")
        print("  • /tactile_contact_analyzer/contact_detected")
        print("  • /tactile_contact_analyzer/displacement")
        print("  • /tactile_contact_analyzer/contact_pose")
        print("  • /tactile_contact_analyzer/contact_info")
        print("  • /tactile_contact_analyzer/visualization")
        print("")
        print("🎯 Touch the tactile sensor to test contact detection")
        print("⌨️  Press 'q' in visualization window or Ctrl+C to quit")
        print("="*70)
        
    def viz_callback(self, msg: Image):
        """Display visualization in OpenCV window."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize to 240x320 (same as camera visualizer)
            resized = cv2.resize(cv_image, (640, 480), interpolation=cv2.INTER_AREA)
            
            cv2.imshow('Tactile Contact Analysis', resized)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed')
                raise KeyboardInterrupt
                
        except Exception as e:
            self.get_logger().error(f'Error displaying visualization: {str(e)}')
    
    def contact_callback(self, msg: Bool):
        """Handle contact detection updates."""
        if msg.data != self.contact_detected:
            if msg.data:
                print(f"\n✅ Contact DETECTED at {time.strftime('%H:%M:%S')}")
            else:
                print(f"\n❌ Contact LOST at {time.strftime('%H:%M:%S')}")
        
        self.contact_detected = msg.data
        self.message_count += 1
    
    def displacement_callback(self, msg: Vector3):
        """Handle displacement updates."""
        self.latest_displacement = msg
        
        if self.contact_detected and self.message_count % 30 == 0:  # Print every 30 frames
            print(f"\n📍 Displacement:")
            print(f"   X: {msg.x:.2f} mm")
            print(f"   Y: {msg.y:.2f} mm")
            print(f"   Distance: {msg.z:.2f} mm")
    
    def info_callback(self, msg: Float32MultiArray):
        """Handle detailed contact info updates."""
        self.latest_info = msg.data
        
        if self.contact_detected and self.message_count % 30 == 0:  # Print every 30 frames
            # [center_x_px, center_y_px, offset_x_mm, offset_y_mm, distance_mm, angle_deg, 
            #  width_mm, height_mm, contact_area_mm2, unified_area_mm2, num_regions]
            if len(msg.data) >= 11:
                print(f"\n📊 Contact Info:")
                print(f"   Center: ({msg.data[0]:.1f}, {msg.data[1]:.1f}) px")
                print(f"   Offset: ({msg.data[2]:.2f}, {msg.data[3]:.2f}) mm")
                print(f"   Distance: {msg.data[4]:.2f} mm")
                print(f"   Angle: {msg.data[5]:.1f}°")
                print(f"   Size: {msg.data[6]:.2f} x {msg.data[7]:.2f} mm")
                print(f"   Area: {msg.data[9]:.2f} mm²")
                print(f"   Regions: {int(msg.data[10])}")
    
    def print_status(self):
        """Print periodic status update."""
        elapsed = time.time() - self.start_time
        fps = self.message_count / elapsed if elapsed > 0 else 0
        
        status_icon = "🟢" if self.contact_detected else "🔴"
        timestamp = time.strftime('%H:%M:%S')
        
        disp_info = ""
        if self.latest_displacement is not None:
            disp_info = f" | Disp: ({self.latest_displacement.x:.1f}, {self.latest_displacement.y:.1f}) mm"
        
        status_line = (f"\r[{timestamp}] {status_icon} Contact: {self.contact_detected} | "
                      f"Messages: {self.message_count} | FPS: {fps:.1f}{disp_info}")
        
        print(status_line, end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    
    monitor = None
    try:
        monitor = ContactAnalyzerMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n🛑 Monitor stopped")
    finally:
        cv2.destroyAllWindows()
        if monitor is not None:
            monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
