#!/usr/bin/env python3
"""
Cable Pose Estimator Node - ROS 2
Estimates cable position and orientation from tactile camera images
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np


class CablePoseEstimator(Node):
    """
    ROS 2 node for cable pose estimation from tactile camera images
    """
    
    def __init__(self):
        super().__init__('cable_pose_estimator')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/gelsight_cam1/image_raw')
        self.declare_parameter('border_size', 120)
        self.declare_parameter('diff_threshold', 10)
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('num_top_regions', 5)
        self.declare_parameter('morph_kernel_size', 15)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('debug_visualization', False)
        self.declare_parameter('pixel_to_mm', 0.0634)
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.border_size = self.get_parameter('border_size').value
        self.diff_threshold = self.get_parameter('diff_threshold').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.num_top_regions = self.get_parameter('num_top_regions').value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug_visualization = self.get_parameter('debug_visualization').value
        self.pixel_to_mm = self.get_parameter('pixel_to_mm').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # State variables
        self.reference_image = None
        self.current_image = None
        self.image_received = False
        
        # Publishers
        self.pose_pub = self.create_publisher(Pose2D, '~/cable_pose', 10)
        self.contact_pub = self.create_publisher(Bool, '~/contact_detected', 10)
        self.metrics_pub = self.create_publisher(Float32MultiArray, '~/cable_metrics', 10)
        
        if self.debug_visualization:
            self.debug_image_pub = self.create_publisher(Image, '~/debug_image', 10)
            self.diff_image_pub = self.create_publisher(Image, '~/diff_image', 10)
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # Service
        self.capture_ref_srv = self.create_service(
            Trigger,
            '~/capture_reference',
            self.capture_reference_callback
        )
        
        # Timer for processing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.process_and_publish)
        
        self.get_logger().info('🚀 Cable Pose Estimator Node started')
        self.get_logger().info(f'📷 Subscribing to: {self.camera_topic}')
        self.get_logger().info(f'🔧 Border size: {self.border_size}px')
        self.get_logger().info(f'📊 Debug visualization: {self.debug_visualization}')
        self.get_logger().info('⚠️  Call ~/capture_reference service to capture reference image')
    
    def image_callback(self, msg: Image):
        """Callback for incoming camera images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def capture_reference_callback(self, request, response):
        """Service callback to capture reference image"""
        if not self.image_received:
            response.success = False
            response.message = 'No camera image received yet. Check camera connection.'
            self.get_logger().warn(response.message)
            return response
        
        if self.current_image is None:
            response.success = False
            response.message = 'Current image is None'
            return response
        
        self.reference_image = self.current_image.copy()
        response.success = True
        response.message = f'Reference image captured! Shape: {self.reference_image.shape}'
        self.get_logger().info(f'✅ {response.message}')
        return response
    
    def process_and_publish(self):
        """Main processing loop - analyze image and publish results"""
        # Check if we have both reference and current images
        if self.reference_image is None:
            # No reference image yet
            return
        
        if self.current_image is None:
            return
        
        try:
            # Process the images
            results = self.process_images(self.reference_image, self.current_image)
            
            # Publish contact detection
            contact_msg = Bool()
            contact_msg.data = results['contact_detected']
            self.contact_pub.publish(contact_msg)
            
            # Publish pose and metrics if contact detected
            if results['contact_detected']:
                # Publish pose
                pose_msg = Pose2D()
                pose_msg.x = results['pose_mm']['x']
                pose_msg.y = results['pose_mm']['y']
                pose_msg.theta = results['pose_mm']['theta']
                self.pose_pub.publish(pose_msg)
                
                # Publish metrics
                metrics_msg = Float32MultiArray()
                metrics_msg.data = [
                    float(results['center_x']),
                    float(results['center_y']),
                    float(results['width']),
                    float(results['height']),
                    float(results['angle']),
                    float(results['offset_x']),
                    float(results['offset_y']),
                    float(results['distance_from_center']),
                    float(results['contact_area']),
                    float(results['unified_area'])
                ]
                self.metrics_pub.publish(metrics_msg)
            
            # Publish debug images if enabled
            if self.debug_visualization:
                if results['debug_image'] is not None:
                    debug_msg = self.bridge.cv2_to_imgmsg(results['debug_image'], encoding='bgr8')
                    self.debug_image_pub.publish(debug_msg)
                
                if results['diff_image'] is not None:
                    diff_msg = self.bridge.cv2_to_imgmsg(results['diff_image'], encoding='mono8')
                    self.diff_image_pub.publish(diff_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error processing images: {e}', throttle_duration_sec=5.0)
    
    def process_images(self, ref_image, curr_image):
        """
        Process reference and current images to detect cable pose
        
        Returns:
            dict: Processing results including pose, metrics, and debug images
        """
        # Convert to grayscale
        gray_ref = cv2.cvtColor(ref_image, cv2.COLOR_BGR2GRAY)
        gray_curr = cv2.cvtColor(curr_image, cv2.COLOR_BGR2GRAY)
        
        # Calculate difference
        diff = cv2.absdiff(gray_ref, gray_curr)
        
        # Apply threshold
        _, thresh = cv2.threshold(diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
        
        # Create border mask
        h, w = thresh.shape
        border_mask = np.zeros((h, w), dtype=np.uint8)
        border_mask[self.border_size:h-self.border_size, 
                   self.border_size:w-self.border_size] = 255
        
        # Apply border mask
        thresh = cv2.bitwise_and(thresh, border_mask)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter by minimum area
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_contour_area]
        
        # Initialize results
        results = {
            'contact_detected': False,
            'center_x': 0.0,
            'center_y': 0.0,
            'width': 0.0,
            'height': 0.0,
            'angle': 0.0,
            'offset_x': 0.0,
            'offset_y': 0.0,
            'distance_from_center': 0.0,
            'contact_area': 0.0,
            'unified_area': 0.0,
            'pose_mm': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'debug_image': None,
            'diff_image': None
        }
        
        if len(valid_contours) == 0:
            # No contact detected
            if self.debug_visualization:
                debug_img = curr_image.copy()
                cv2.rectangle(debug_img, 
                            (self.border_size, self.border_size),
                            (w-self.border_size, h-self.border_size),
                            (255, 0, 0), 2)
                cv2.putText(debug_img, "NO CONTACT", (50, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                results['debug_image'] = debug_img
                results['diff_image'] = thresh
            return results
        
        # Select top N regions
        valid_contours_sorted = sorted(valid_contours, key=cv2.contourArea, reverse=True)
        top_contours = valid_contours_sorted[:self.num_top_regions]
        
        # Create unified mask
        unified_mask = np.zeros_like(thresh)
        contact_area = 0
        for contour in top_contours:
            cv2.drawContours(unified_mask, [contour], -1, 255, -1)
            contact_area += cv2.contourArea(contour)
        
        # Morphological closing to unify regions
        kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        unified_mask = cv2.morphologyEx(unified_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find unified contours
        unified_contours, _ = cv2.findContours(unified_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(unified_contours) == 0:
            return results
        
        # Contact detected!
        results['contact_detected'] = True
        results['contact_area'] = contact_area
        
        # Calculate unified area
        unified_area = sum([cv2.contourArea(c) for c in unified_contours])
        results['unified_area'] = unified_area
        
        # Concatenate all points
        all_points = np.vstack(unified_contours)
        
        # Calculate minimum area rectangle
        rect = cv2.minAreaRect(all_points)
        box = cv2.boxPoints(rect)
        box = box.astype(int)
        
        # Extract rectangle information
        center, (width, height), angle = rect
        center_x, center_y = center
        
        # Calculate perpendicular angle (add 90 degrees to the rectangle angle)
        # This gives us the main axis perpendicular to the rectangle orientation
        cable_angle = angle + 90
        
        # Normalize angle to [-180, 180] range
        while cable_angle > 180:
            cable_angle -= 360
        while cable_angle < -180:
            cable_angle += 360
        
        # Calculate offset from image center
        img_center_x, img_center_y = w // 2, h // 2
        offset_x = center_x - img_center_x
        offset_y = center_y - img_center_y
        distance_from_center = np.sqrt(offset_x**2 + offset_y**2)
        
        # Store results
        results['center_x'] = center_x
        results['center_y'] = center_y
        results['width'] = width
        results['height'] = height
        results['angle'] = cable_angle  # Use the corrected cable angle
        results['offset_x'] = offset_x
        results['offset_y'] = offset_y
        results['distance_from_center'] = distance_from_center
        
        # Convert to mm and radians
        results['pose_mm'] = {
            'x': offset_x * self.pixel_to_mm,
            'y': offset_y * self.pixel_to_mm,
            'theta': np.radians(cable_angle)  # Use the corrected cable angle
        }
        
        # Create debug visualization
        if self.debug_visualization:
            debug_img = curr_image.copy()
            
            # Draw border
            cv2.rectangle(debug_img,
                         (self.border_size, self.border_size),
                         (w-self.border_size, h-self.border_size),
                         (255, 0, 0), 2)
            
            # Draw top contours in green
            for contour in top_contours:
                cv2.drawContours(debug_img, [contour], -1, (0, 255, 0), 2)
            
            # Draw unified contour in red
            cv2.drawContours(debug_img, unified_contours, -1, (0, 0, 255), 3)
            
            # Draw minimum area rectangle in cyan
            cv2.drawContours(debug_img, [box], 0, (255, 255, 0), 3)
            
            # Draw center point in magenta
            cv2.circle(debug_img, (int(center_x), int(center_y)), 8, (255, 0, 255), -1)
            
            # Draw orientation line using the perpendicular angle (cable_angle)
            cable_angle_rad = np.radians(cable_angle)
            line_length = max(width, height) / 2
            end_x = int(center_x + line_length * np.cos(cable_angle_rad))
            end_y = int(center_y + line_length * np.sin(cable_angle_rad))
            cv2.line(debug_img, (int(center_x), int(center_y)), (end_x, end_y), 
                    (255, 0, 255), 3)
            
            # Add text info
            info_text = [
                f"Contact Area: {contact_area:.0f} px",
                f"Center: ({center_x:.1f}, {center_y:.1f})",
                f"Offset: ({offset_x:.1f}, {offset_y:.1f}) px",
                f"Cable Angle: {cable_angle:.1f} deg",
                f"Size: {width:.1f}x{height:.1f} px"
            ]
            
            y_pos = 30
            for text in info_text:
                cv2.putText(debug_img, text, (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(debug_img, text, (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
                y_pos += 25
            
            results['debug_image'] = debug_img
            results['diff_image'] = thresh
        
        return results


def main(args=None):
    rclpy.init(args=args)
    node = CablePoseEstimator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
