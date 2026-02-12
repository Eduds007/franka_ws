#!/usr/bin/env python3
"""
Tactile Contact Analyzer Node
Analyzes tactile camera images to detect contact and compute displacement.

This node:
1. Subscribes to tactile camera image topics
2. Compares current image with reference (no-contact) image
3. Detects contact regions using difference thresholding
4. Computes contact center, orientation, and displacement from image center
5. Publishes contact information and displacement values
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from std_msgs.msg import Bool, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, Tuple, Dict
import time


class TactileContactAnalyzer(Node):
    """
    ROS2 Node for tactile contact analysis and displacement computation.
    Based on image difference analysis from notebook demo.
    """
    
    def __init__(self):
        super().__init__('tactile_contact_analyzer')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/gelsight_cam1/image_raw')
        self.declare_parameter('border_size', 120)  # Border to ignore in pixels
        self.declare_parameter('diff_threshold', 10)  # Threshold for difference detection
        self.declare_parameter('min_contour_area', 500)  # Minimum contour area
        self.declare_parameter('num_top_regions', 5)  # Number of top regions to select
        self.declare_parameter('morph_kernel_size', 15)  # Morphological operation kernel
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('debug_visualization', True)
        self.declare_parameter('auto_reference_capture', True)  # Auto-capture reference on start
        self.declare_parameter('pixel_to_mm', 0.0634)  # Resolution: mm per pixel
        
        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        self.border_size = self.get_parameter('border_size').value
        self.diff_threshold = self.get_parameter('diff_threshold').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.num_top_regions = self.get_parameter('num_top_regions').value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value
        self.debug_viz = self.get_parameter('debug_visualization').value
        self.auto_reference = self.get_parameter('auto_reference_capture').value
        self.pixel_to_mm = self.get_parameter('pixel_to_mm').value
        
        # Initialize components
        self.bridge = CvBridge()
        
        # State variables
        self.reference_image = None
        self.latest_image = None
        self.image_center = None  # Will be set on first image
        self.contact_detected = False
        self.latest_analysis = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.contact_detected_pub = self.create_publisher(Bool, '~/contact_detected', 10)
        self.displacement_pub = self.create_publisher(Vector3, '~/displacement', 10)
        self.contact_pose_pub = self.create_publisher(PoseStamped, '~/contact_pose', 10)
        self.contact_info_pub = self.create_publisher(Float32MultiArray, '~/contact_info', 10)
        
        if self.debug_viz:
            self.viz_pub = self.create_publisher(Image, '~/visualization', 10)
        
        self.get_logger().info(f'Tactile Contact Analyzer initialized')
        self.get_logger().info(f'Subscribing to: {camera_topic}')
        self.get_logger().info(f'Border size: {self.border_size} pixels')
        
        if not self.auto_reference:
            self.get_logger().info('⚠️  Press Enter to capture reference image (no-contact state)')
        
        # Timer for processing
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.process_callback)
        
    def image_callback(self, msg: Image):
        """Callback for incoming images."""
        try:
            # Convert ROS Image to OpenCV format (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
            
            # Set image center on first image
            if self.image_center is None:
                h, w = cv_image.shape[:2]
                self.image_center = (w // 2, h // 2)
                self.get_logger().info(f'Image size: {w}x{h}, Center: {self.image_center}')
                
                # Auto-capture reference if enabled
                if self.auto_reference and self.reference_image is None:
                    time.sleep(0.5)  # Wait a bit for camera to stabilize
                    self.capture_reference()
                
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
    
    def capture_reference(self) -> bool:
        """Capture reference image (no-contact state)."""
        if self.latest_image is not None:
            self.reference_image = self.latest_image.copy()
            self.get_logger().info('✓ Reference image captured')
            return True
        else:
            self.get_logger().warn('No image available for reference capture')
            return False
    
    def analyze_contact(self, current_image: np.ndarray, 
                       reference_image: np.ndarray) -> Optional[Dict]:
        """
        Analyze contact from image difference (based on notebook approach).
        
        Args:
            current_image: Current BGR image
            reference_image: Reference BGR image (no contact)
            
        Returns:
            Dictionary with contact information or None if no contact detected
        """
        # Convert to grayscale
        gray_current = cv2.cvtColor(current_image, cv2.COLOR_BGR2GRAY)
        gray_reference = cv2.cvtColor(reference_image, cv2.COLOR_BGR2GRAY)
        
        # Calculate absolute difference
        diff = cv2.absdiff(gray_current, gray_reference)
        
        # Apply threshold
        _, thresh = cv2.threshold(diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
        
        # Create border mask to ignore edges
        h, w = thresh.shape
        border_mask = np.zeros((h, w), dtype=np.uint8)
        bs = self.border_size
        border_mask[bs:h-bs, bs:w-bs] = 255
        
        # Apply border mask
        thresh = cv2.bitwise_and(thresh, border_mask)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_contour_area]
        
        if len(valid_contours) == 0:
            return None
        
        # Select top N largest regions
        valid_contours_sorted = sorted(valid_contours, key=cv2.contourArea, reverse=True)
        top_contours = valid_contours_sorted[:self.num_top_regions]
        
        # Create unified mask for top regions
        unified_mask = np.zeros_like(thresh)
        for contour in top_contours:
            cv2.drawContours(unified_mask, [contour], -1, 255, -1)
        
        # Apply morphological closing to unify nearby regions
        kernel_large = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        unified_mask = cv2.morphologyEx(unified_mask, cv2.MORPH_CLOSE, kernel_large)
        
        # Find contour of unified region
        unified_contours, _ = cv2.findContours(unified_mask, cv2.RETR_EXTERNAL, 
                                               cv2.CHAIN_APPROX_SIMPLE)
        
        if len(unified_contours) == 0:
            return None
        
        # Calculate total area
        contact_area = sum([cv2.contourArea(c) for c in top_contours])
        unified_area = sum([cv2.contourArea(c) for c in unified_contours])
        
        # Concatenate all points from unified contours
        all_points = np.vstack(unified_contours)
        
        # Calculate minimum oriented bounding rectangle
        rect = cv2.minAreaRect(all_points)
        box = cv2.boxPoints(rect)
        box = box.astype(int)
        
        # Extract rectangle information
        center, (width, height), angle = rect
        center_x, center_y = center
        
        # Calculate displacement from image center
        img_center_x, img_center_y = self.image_center
        offset_x = center_x - img_center_x
        offset_y = center_y - img_center_y
        distance_from_center = np.sqrt(offset_x**2 + offset_y**2)
        
        # Calculate angle in radians for consistency
        angle_rad = np.radians(angle)
        
        return {
            'detected': True,
            'center': (center_x, center_y),
            'offset': (offset_x, offset_y),
            'distance': distance_from_center,
            'angle_deg': angle,
            'angle_rad': angle_rad,
            'width': width,
            'height': height,
            'contact_area': contact_area,
            'unified_area': unified_area,
            'num_regions': len(top_contours),
            'top_contours': top_contours,
            'unified_contours': unified_contours,
            'bounding_box': box,
            'diff_image': diff,
            'thresh_image': thresh,
            'unified_mask': unified_mask
        }
    
    def process_callback(self):
        """Main processing loop."""
        if self.latest_image is None or self.reference_image is None:
            return
        
        try:
            # Analyze contact
            analysis = self.analyze_contact(self.latest_image, self.reference_image)
            self.latest_analysis = analysis
            
            # Update contact detected flag
            self.contact_detected = analysis is not None
            
            # Publish results
            self.publish_results(analysis)
            
        except Exception as e:
            self.get_logger().error(f'Error in processing: {str(e)}')
            import traceback
            traceback.print_exc()
    
    def publish_results(self, analysis: Optional[Dict]):
        """Publish analysis results."""
        stamp = self.get_clock().now().to_msg()
        
        # Publish contact detected flag
        contact_msg = Bool()
        contact_msg.data = analysis is not None
        self.contact_detected_pub.publish(contact_msg)
        
        if analysis is None:
            return
        
        # Convert pixels to millimeters
        offset_x_mm = analysis['offset'][0] * self.pixel_to_mm
        offset_y_mm = analysis['offset'][1] * self.pixel_to_mm
        distance_mm = analysis['distance'] * self.pixel_to_mm
        width_mm = analysis['width'] * self.pixel_to_mm
        height_mm = analysis['height'] * self.pixel_to_mm
        contact_area_mm2 = analysis['contact_area'] * (self.pixel_to_mm ** 2)
        unified_area_mm2 = analysis['unified_area'] * (self.pixel_to_mm ** 2)
        
        # Publish displacement (offset from center) in millimeters
        displacement_msg = Vector3()
        displacement_msg.x = float(offset_x_mm)
        displacement_msg.y = float(offset_y_mm)
        displacement_msg.z = float(distance_mm)
        self.displacement_pub.publish(displacement_msg)
        
        # Publish contact pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'tactile_camera'
        pose_msg.pose.position.x = float(analysis['center'][0])
        pose_msg.pose.position.y = float(analysis['center'][1])
        pose_msg.pose.position.z = 0.0
        
        # Orientation as quaternion (rotation around z-axis)
        angle = analysis['angle_rad']
        pose_msg.pose.orientation.w = np.cos(angle / 2)
        pose_msg.pose.orientation.z = np.sin(angle / 2)
        
        self.contact_pose_pub.publish(pose_msg)
        
        # Publish detailed contact info as array
        # [center_x_px, center_y_px, offset_x_mm, offset_y_mm, distance_mm, angle_deg, 
        #  width_mm, height_mm, contact_area_mm2, unified_area_mm2, num_regions]
        info_msg = Float32MultiArray()
        info_msg.data = [
            float(analysis['center'][0]),  # pixels
            float(analysis['center'][1]),  # pixels
            float(offset_x_mm),  # mm
            float(offset_y_mm),  # mm
            float(distance_mm),  # mm
            float(analysis['angle_deg']),  # degrees
            float(width_mm),  # mm
            float(height_mm),  # mm
            float(contact_area_mm2),  # mm²
            float(unified_area_mm2),  # mm²
            float(analysis['num_regions'])  # count
        ]
        self.contact_info_pub.publish(info_msg)
        
        # Publish debug visualization
        if self.debug_viz:
            self.publish_visualization(analysis)
    
    def publish_visualization(self, analysis: Dict):
        """Create and publish debug visualization (similar to notebook)."""
        # Create result image
        result = self.latest_image.copy()
        h, w = result.shape[:2]
        
        # Draw border area in blue
        bs = self.border_size
        cv2.rectangle(result, (bs, bs), (w-bs, h-bs), (255, 0, 0), 2)
        
        # Draw top regions in green
        for contour in analysis['top_contours']:
            cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
        
        # Draw unified contour in red
        cv2.drawContours(result, analysis['unified_contours'], -1, (0, 0, 255), 3)
        
        # Draw bounding box in cyan
        cv2.drawContours(result, [analysis['bounding_box']], 0, (255, 255, 0), 3)
        
        # Draw center point in magenta
        center = tuple(map(int, analysis['center']))
        cv2.circle(result, center, 8, (255, 0, 255), -1)
        
        # Draw orientation line
        angle_rad = analysis['angle_rad']
        line_length = max(analysis['width'], analysis['height']) / 2
        end_x = int(center[0] + line_length * np.cos(angle_rad))
        end_y = int(center[1] + line_length * np.sin(angle_rad))
        cv2.line(result, center, (end_x, end_y), (255, 0, 255), 3)
        
        # Draw image center in white
        img_center = tuple(map(int, self.image_center))
        cv2.circle(result, img_center, 5, (255, 255, 255), -1)
        
        # Draw line from image center to contact center
        cv2.line(result, img_center, center, (255, 255, 255), 2)
        
        # Add text annotations
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        color = (255, 255, 255)
        
        y_offset = 30
        cv2.putText(result, f"Offset: ({analysis['offset'][0]:.1f}, {analysis['offset'][1]:.1f}) px", 
                   (10, y_offset), font, font_scale, color, thickness)
        y_offset += 30
        cv2.putText(result, f"Distance: {analysis['distance']:.1f} px", 
                   (10, y_offset), font, font_scale, color, thickness)
        y_offset += 30
        cv2.putText(result, f"Angle: {analysis['angle_deg']:.1f} deg", 
                   (10, y_offset), font, font_scale, color, thickness)
        y_offset += 30
        cv2.putText(result, f"Size: {analysis['width']:.1f} x {analysis['height']:.1f} px", 
                   (10, y_offset), font, font_scale, color, thickness)
        y_offset += 30
        cv2.putText(result, f"Area: {analysis['unified_area']:.0f} px^2", 
                   (10, y_offset), font, font_scale, color, thickness)
        
        # Create multi-panel visualization
        # Each panel: 320x240 pixels (width x height) - same as camera visualizer
        # Grid: 2 rows x 3 columns = 960x480 total
        panel_width = 320
        panel_height = 240
        
        # Calculate individual panel size
        new_w = panel_width  # 320 pixels per panel width
        new_h = panel_height  # 240 pixels per panel height
        
        # Total window size
        target_width = panel_width * 3  # 960 pixels total width
        target_height = panel_height * 2  # 480 pixels total height
        
        # Original image
        orig_resized = cv2.resize(self.latest_image, (new_w, new_h))
        
        # Reference image
        ref_resized = cv2.resize(self.reference_image, (new_w, new_h))
        
        # Difference image (colormap)
        diff_colored = cv2.applyColorMap(analysis['diff_image'], cv2.COLORMAP_HOT)
        diff_resized = cv2.resize(diff_colored, (new_w, new_h))
        
        # Threshold image
        thresh_colored = cv2.cvtColor(analysis['thresh_image'], cv2.COLOR_GRAY2BGR)
        thresh_resized = cv2.resize(thresh_colored, (new_w, new_h))
        
        # Unified mask
        mask_colored = cv2.cvtColor(analysis['unified_mask'], cv2.COLOR_GRAY2BGR)
        mask_resized = cv2.resize(mask_colored, (new_w, new_h))
        
        # Result image
        result_resized = cv2.resize(result, (new_w, new_h))
        
        # Create 2x3 grid
        top_row = np.hstack([orig_resized, ref_resized, diff_resized])
        bottom_row = np.hstack([thresh_resized, mask_resized, result_resized])
        combined = np.vstack([top_row, bottom_row])
        
        # Force exact size (in case of rounding errors)
        if combined.shape[0] != target_height or combined.shape[1] != target_width:
            combined = cv2.resize(combined, (target_width, target_height))
        
        # Add panel labels (adjusted font for 320x240 panels)
        label_font_scale = 0.5
        label_thickness = 1
        cv2.putText(combined, "Original", (5, 20), font, label_font_scale, (255, 255, 255), label_thickness)
        cv2.putText(combined, "Reference", (new_w + 5, 20), font, label_font_scale, (255, 255, 255), label_thickness)
        cv2.putText(combined, "Difference", (2*new_w + 5, 20), font, label_font_scale, (255, 255, 255), label_thickness)
        cv2.putText(combined, "Threshold", (5, new_h + 20), font, label_font_scale, (255, 255, 255), label_thickness)
        cv2.putText(combined, "Unified Mask", (new_w + 5, new_h + 20), font, label_font_scale, (255, 255, 255), label_thickness)
        cv2.putText(combined, "Result", (2*new_w + 5, new_h + 20), font, label_font_scale, (255, 255, 255), label_thickness)
        
        # Publish
        viz_msg = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        self.viz_pub.publish(viz_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = TactileContactAnalyzer()
    
    # Wait a bit for first image
    time.sleep(1.0)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down tactile contact analyzer...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
