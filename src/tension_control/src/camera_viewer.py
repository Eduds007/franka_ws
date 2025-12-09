#!/usr/bin/env python3
"""
Simple Camera Viewer for GelSight Cameras
Basic ROS2 node to view dual camera feeds
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading


class CameraViewerNode(Node):
    """Simple camera viewer node"""
    
    def __init__(self):
        super().__init__('camera_viewer')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Current frames
        self.frame1 = None
        self.frame2 = None
        self.frame_lock = threading.Lock()
        
        # Image subscribers
        self.cam1_sub = self.create_subscription(
            Image, '/gelsight_cam1/image_raw', self.camera1_callback, 10)
        self.cam2_sub = self.create_subscription(
            Image, '/gelsight_cam2/image_raw', self.camera2_callback, 10)
        
        self.get_logger().info("🎥 Camera Viewer Node initialized")
        self.get_logger().info("📡 Listening to topics:")
        self.get_logger().info("   - /gelsight_cam1/image_raw")
        self.get_logger().info("   - /gelsight_cam2/image_raw")
        
        # Start display timer
        self.display_timer = self.create_timer(0.033, self.update_display)  # ~30 FPS
        
        # Initialize OpenCV windows
        cv2.namedWindow('GelSight Camera 1', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('GelSight Camera 2', cv2.WINDOW_AUTOSIZE)
        
        # Position windows side by side (adjusted for smaller 240x320 windows)
        cv2.moveWindow('GelSight Camera 1', 100, 100)
        cv2.moveWindow('GelSight Camera 2', 450, 100)  # Closer together due to smaller size
    
    def camera1_callback(self, msg):
        """Camera 1 callback"""
        try:
            with self.frame_lock:
                self.frame1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera 1: {e}")
    
    def camera2_callback(self, msg):
        """Camera 2 callback"""
        try:
            with self.frame_lock:
                self.frame2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera 2: {e}")
    
    def update_display(self):
        """Update OpenCV display windows"""
        # Fixed display size: 240x320 (height x width)
        display_height = 240
        display_width = 320
        
        with self.frame_lock:
            # Display camera 1
            if self.frame1 is not None:
                # Resize frame to fixed size
                display_frame1 = cv2.resize(self.frame1, (display_width, display_height))
                
                # Add timestamp and info with smaller font for smaller window
                cv2.putText(display_frame1, "GelSight Camera 1", (5, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(display_frame1, f"Size: {display_frame1.shape[:2]}", (5, 35), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                cv2.imshow('GelSight Camera 1', display_frame1)
            else:
                # Show waiting message
                placeholder = np.zeros((display_height, display_width, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for", (80, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.putText(placeholder, "Camera 1...", (85, 130), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.imshow('GelSight Camera 1', placeholder)
            
            # Display camera 2
            if self.frame2 is not None:
                # Resize frame to fixed size
                display_frame2 = cv2.resize(self.frame2, (display_width, display_height))
                
                # Add timestamp and info with smaller font for smaller window
                cv2.putText(display_frame2, "GelSight Camera 2", (5, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.putText(display_frame2, f"Size: {display_frame2.shape[:2]}", (5, 35), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                cv2.imshow('GelSight Camera 2', display_frame2)
            else:
                # Show waiting message
                placeholder = np.zeros((display_height, display_width, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for", (80, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.putText(placeholder, "Camera 2...", (85, 130), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.imshow('GelSight Camera 2', placeholder)
        
        # Check for ESC key to exit
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC key
            self.get_logger().info("ESC pressed - shutting down...")
            rclpy.shutdown()
    
    def destroy_node(self):
        """Clean up OpenCV windows when shutting down"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    """Main function"""
    rclpy.init()
    
    try:
        viewer_node = CameraViewerNode()
        
        # Show instructions
        print("\n" + "="*60)
        print("🎥 GelSight Camera Viewer")
        print("="*60)
        print("📺 Two OpenCV windows will open showing camera feeds")
        print("🔄 Waiting for camera topics:")
        print("   • /gelsight_cam1/image_raw")
        print("   • /gelsight_cam2/image_raw")
        print("\n⌨️  Controls:")
        print("   • ESC key: Exit")
        print("   • Ctrl+C: Force exit")
        print("="*60)
        
        rclpy.spin(viewer_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        if 'viewer_node' in locals():
            viewer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
