#!/usr/bin/env python3
"""
Simple Camera Publisher Node
Publishes camera feeds from USB cameras or webcams to ROS2 topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraPublisherNode(Node):
    """Simple camera publisher node"""
    
    def __init__(self):
        super().__init__('camera_publisher')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Camera parameters
        self.camera1_id = self.declare_parameter('camera1_id', 1).value
        self.camera2_id = self.declare_parameter('camera2_id', 3).value
        self.fps = self.declare_parameter('fps', 30.0).value
        self.width = self.declare_parameter('width', 640).value
        self.height = self.declare_parameter('height', 480).value
        
        # Publishers
        self.pub1 = self.create_publisher(Image, '/gelsight_cam1/image_raw', 10)
        self.pub2 = self.create_publisher(Image, '/gelsight_cam2/image_raw', 10)
        
        # Try to open cameras
        self.cap1 = None
        self.cap2 = None
        self.setup_cameras()
        
        # Timer for publishing frames
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frames)
        
        self.get_logger().info("📸 Camera Publisher Node initialized")
    
    def setup_cameras(self):
        """Setup cameras"""
        self.get_logger().info("🔧 Setting up cameras...")
        
        # Try to open camera 1
        try:
            self.cap1 = cv2.VideoCapture(self.camera1_id)
            if self.cap1.isOpened():
                self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap1.set(cv2.CAP_PROP_FPS, self.fps)
                self.get_logger().info(f"✅ Camera 1 opened (ID: {self.camera1_id})")
            else:
                self.get_logger().warn(f"⚠️ Cannot open camera 1 (ID: {self.camera1_id})")
                self.cap1 = None
        except Exception as e:
            self.get_logger().error(f"❌ Error opening camera 1: {e}")
            self.cap1 = None
        
        # Try to open camera 2
        try:
            self.cap2 = cv2.VideoCapture(self.camera2_id)
            if self.cap2.isOpened():
                self.cap2.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap2.set(cv2.CAP_PROP_FPS, self.fps)
                self.get_logger().info(f"✅ Camera 2 opened (ID: {self.camera2_id})")
            else:
                self.get_logger().warn(f"⚠️ Cannot open camera 2 (ID: {self.camera2_id})")
                self.cap2 = None
        except Exception as e:
            self.get_logger().error(f"❌ Error opening camera 2: {e}")
            self.cap2 = None
        
        # Check if at least one camera is available
        if self.cap1 is None and self.cap2 is None:
            self.get_logger().warn("⚠️ No cameras available - will publish dummy frames")
            self.setup_dummy_mode()
    
    def setup_dummy_mode(self):
        """Setup dummy camera mode"""
        self.get_logger().info("🎮 Setting up dummy camera mode")
        self.dummy_mode = True
        
        # Create dummy frames
        self.dummy_frame1 = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.dummy_frame2 = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Add some patterns
        cv2.putText(self.dummy_frame1, "DUMMY CAMERA 1", (50, self.height//2), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.circle(self.dummy_frame1, (self.width//2, self.height//2), 100, (100, 100, 100), -1)
        
        cv2.putText(self.dummy_frame2, "DUMMY CAMERA 2", (50, self.height//2), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.rectangle(self.dummy_frame2, (self.width//4, self.height//4), 
                     (3*self.width//4, 3*self.height//4), (100, 100, 100), -1)
    
    def publish_frames(self):
        """Publish camera frames"""
        current_time = self.get_clock().now()
        
        # Get frame from camera 1
        if self.cap1 is not None and self.cap1.isOpened():
            ret1, frame1 = self.cap1.read()
            if ret1:
                # Add timestamp
                timestamp_str = f"Cam1: {current_time.nanoseconds // 1000000}"
                cv2.putText(frame1, timestamp_str, (10, self.height - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Convert and publish
                try:
                    img_msg1 = self.bridge.cv2_to_imgmsg(frame1, "bgr8")
                    img_msg1.header.stamp = current_time.to_msg()
                    img_msg1.header.frame_id = "gelsight_cam1"
                    self.pub1.publish(img_msg1)
                except Exception as e:
                    self.get_logger().error(f"Error publishing camera 1: {e}")
        else:
            # Publish dummy frame for camera 1
            if hasattr(self, 'dummy_frame1'):
                # Add some animation
                frame1 = self.dummy_frame1.copy()
                t = current_time.nanoseconds // 1000000 % 1000
                cv2.circle(frame1, (int(self.width//2 + 50*np.sin(t/100)), self.height//2), 
                          10, (255, 255, 255), -1)
                
                try:
                    img_msg1 = self.bridge.cv2_to_imgmsg(frame1, "bgr8")
                    img_msg1.header.stamp = current_time.to_msg()
                    img_msg1.header.frame_id = "gelsight_cam1"
                    self.pub1.publish(img_msg1)
                except Exception as e:
                    self.get_logger().error(f"Error publishing dummy camera 1: {e}")
        
        # Get frame from camera 2
        if self.cap2 is not None and self.cap2.isOpened():
            ret2, frame2 = self.cap2.read()
            if ret2:
                # Add timestamp
                timestamp_str = f"Cam2: {current_time.nanoseconds // 1000000}"
                cv2.putText(frame2, timestamp_str, (10, self.height - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Convert and publish
                try:
                    img_msg2 = self.bridge.cv2_to_imgmsg(frame2, "bgr8")
                    img_msg2.header.stamp = current_time.to_msg()
                    img_msg2.header.frame_id = "gelsight_cam2"
                    self.pub2.publish(img_msg2)
                except Exception as e:
                    self.get_logger().error(f"Error publishing camera 2: {e}")
        else:
            # Publish dummy frame for camera 2
            if hasattr(self, 'dummy_frame2'):
                # Add some animation
                frame2 = self.dummy_frame2.copy()
                t = current_time.nanoseconds // 1000000 % 1000
                size = int(50 + 20*np.cos(t/100))
                cv2.rectangle(frame2, (self.width//2 - size, self.height//2 - size),
                             (self.width//2 + size, self.height//2 + size), (255, 255, 255), 2)
                
                try:
                    img_msg2 = self.bridge.cv2_to_imgmsg(frame2, "bgr8")
                    img_msg2.header.stamp = current_time.to_msg()
                    img_msg2.header.frame_id = "gelsight_cam2"
                    self.pub2.publish(img_msg2)
                except Exception as e:
                    self.get_logger().error(f"Error publishing dummy camera 2: {e}")
    
    def destroy_node(self):
        """Clean up cameras when shutting down"""
        if self.cap1 is not None:
            self.cap1.release()
        if self.cap2 is not None:
            self.cap2.release()
        super().destroy_node()


def main():
    """Main function"""
    rclpy.init()
    
    try:
        publisher_node = CameraPublisherNode()
        
        print("\n" + "="*60)
        print("📸 Camera Publisher Node")
        print("="*60)
        print("📡 Publishing to topics:")
        print("   • /gelsight_cam1/image_raw")
        print("   • /gelsight_cam2/image_raw")
        print("\n💡 If no USB cameras found, dummy frames will be published")
        print("⌨️  Press Ctrl+C to stop")
        print("="*60)
        
        rclpy.spin(publisher_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        if 'publisher_node' in locals():
            publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
