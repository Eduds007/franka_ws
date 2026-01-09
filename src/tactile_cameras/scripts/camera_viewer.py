#!/usr/bin/env python3
"""
GelSight Camera Viewer Node
Simple viewer for GelSight camera feeds
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import numpy as np


class CameraViewerNode(Node):
    """GelSight camera viewer node"""
    
    def __init__(self):
        super().__init__('gelsight_camera_viewer')
        
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
        
        self.get_logger().info("🎥 GelSight Camera Viewer Node initialized")
        
        # Timer for updating display
        self.display_timer = self.create_timer(0.033, self.update_display)  # ~30 FPS
        
        # Create windows
        cv2.namedWindow("GelSight Camera 1", cv2.WINDOW_NORMAL)
        cv2.namedWindow("GelSight Camera 2", cv2.WINDOW_NORMAL)
    
    def camera1_callback(self, msg):
        """Callback for camera 1"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.frame_lock:
                self.frame1 = frame
        except Exception as e:
            self.get_logger().error(f"Error converting camera 1 image: {e}")
    
    def camera2_callback(self, msg):
        """Callback for camera 2"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.frame_lock:
                self.frame2 = frame
        except Exception as e:
            self.get_logger().error(f"Error converting camera 2 image: {e}")
    
    def update_display(self):
        """Update OpenCV display windows"""
        with self.frame_lock:
            if self.frame1 is not None:
                cv2.imshow("GelSight Camera 1", self.frame1)
            
            if self.frame2 is not None:
                cv2.imshow("GelSight Camera 2", self.frame2)
        
        # Wait for key press (needed for OpenCV window updates)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # 'q' or ESC
            self.get_logger().info("Quit requested")
            rclpy.shutdown()
    
    def destroy_node(self):
        """Clean up when shutting down"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    """Main function"""
    rclpy.init()
    
    try:
        viewer_node = CameraViewerNode()
        
        print("\n" + "="*60)
        print("🎥 GelSight Camera Viewer")
        print("="*60)
        print("📡 Subscribing to:")
        print("   • /gelsight_cam1/image_raw")
        print("   • /gelsight_cam2/image_raw")
        print("\n⌨️  Press 'q' or ESC to quit")
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
