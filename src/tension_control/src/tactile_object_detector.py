#!/usr/bin/env python3
"""
Tactile Object Detector Node for Tension Control
Integrates dual camera object detection with gripper control for adaptive tension control
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
from datetime import datetime

# For UI
os.environ["KIVY_NO_ARGS"] = "1"
from kivy.app import App
from kivy.clock import Clock
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image as KivyImage
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.slider import Slider
from kivy.uix.switch import Switch
from kivy.graphics.texture import Texture
from kivy.metrics import dp


class TactileObjectDetectorNode(Node):
    """ROS2 Node for tactile-based object detection and gripper control"""
    
    def __init__(self):
        super().__init__('tactile_object_detector')
        
        # ROS2 Setup
        self.bridge = CvBridge()
        
        # Camera state tracking
        self.cameras_available = False
        self.camera1_connected = False
        self.camera2_connected = False
        self.last_cam1_time = time.time()
        self.last_cam2_time = time.time()
        self.camera_timeout = 3.0  # seconds
        
        # Try to open cameras first
        self.setup_cameras()
        
        # Image subscribers
        self.cam1_sub = self.create_subscription(
            Image, '/gelsight_cam1/image_raw', self.camera1_callback, 10)
        self.cam2_sub = self.create_subscription(
            Image, '/gelsight_cam2/image_raw', self.camera2_callback, 10)
        
        # Publishers
        self.object_detected_pub = self.create_publisher(Bool, '/object_detected', 10)
        self.object_confidence_pub = self.create_publisher(Float32, '/object_confidence', 10)
        self.grip_force_pub = self.create_publisher(Float32, '/adaptive_grip_force', 10)
        self.status_pub = self.create_publisher(String, '/detector_status', 10)
        
        # Gripper action client
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_gripper/gripper_action')
        
        # Detection parameters
        self.detection_threshold = 0.6
        self.background_frames = []
        self.background_learned = False
        self.learning_frames = 30
        self.adaptive_grip_enabled = True
        self.min_grip_force = 5.0
        self.max_grip_force = 50.0
        self.simulation_mode = False  # Usar câmeras simuladas se não houver câmeras reais
        
        # Current frames
        self.current_frame1 = None
        self.current_frame2 = None
        self.detection_results = {'cam1': False, 'cam2': False, 'confidence1': 0.0, 'confidence2': 0.0}
        
        # Timer for detection processing
        self.detection_timer = self.create_timer(0.1, self.process_detection)
        
        # Timer for camera status check
        self.camera_check_timer = self.create_timer(2.0, self.check_camera_status)
        
        # Auto-background learning timer (only after cameras are ready)
        self.background_learn_timer = self.create_timer(5.0, self.try_auto_learn_background)
        
        self.get_logger().info("🔍 Tactile Object Detector Node initialized")
    
    def camera1_callback(self, msg):
        """Process camera 1 frames"""
        try:
            self.current_frame1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera 1: {e}")
    
    def camera2_callback(self, msg):
        """Process camera 2 frames"""
        try:
            self.current_frame2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing camera 2: {e}")
    
    def learn_background(self):
        """Learn background for both cameras"""
        self.get_logger().info("📚 Learning background...")
        self.background_frames = []
        self.background_learned = False
        
        # Check if cameras are available
        if not self.cameras_available:
            self.get_logger().warn("⚠️ Cannot learn background - cameras not available")
            return
        
        # Collect background frames
        frame_count = 0
        timeout_count = 0
        max_timeout = 100  # 10 seconds at 0.1s intervals
        
        while frame_count < self.learning_frames and rclpy.ok() and timeout_count < max_timeout:
            if self.current_frame1 is not None and self.current_frame2 is not None:
                self.background_frames.append({
                    'cam1': self.current_frame1.copy(),
                    'cam2': self.current_frame2.copy()
                })
                frame_count += 1
                
                # Log progress every 10 frames
                if frame_count % 10 == 0:
                    self.get_logger().info(f"📸 Background learning progress: {frame_count}/{self.learning_frames}")
            else:
                timeout_count += 1
                if timeout_count % 50 == 0:  # Log every 5 seconds
                    self.get_logger().warn(f"⏳ Waiting for camera frames... ({timeout_count/10:.1f}s)")
            
            time.sleep(0.1)
        
        if frame_count >= self.learning_frames:
            self.background_learned = True
            self.get_logger().info(f"✅ Background learned successfully with {len(self.background_frames)} frames")
        else:
            self.get_logger().error(f"❌ Background learning failed - only collected {frame_count}/{self.learning_frames} frames")
            self.background_learned = False
    
    def detect_object_change(self, current_frame, background_frames, camera_name):
        """Detect object presence based on background subtraction"""
        if not self.background_learned or len(background_frames) == 0:
            return False, 0.0
        
        # Calculate average background
        bg_sum = np.zeros_like(background_frames[0][camera_name], dtype=np.float32)
        for frame_data in background_frames:
            bg_sum += frame_data[camera_name].astype(np.float32)
        background = (bg_sum / len(background_frames)).astype(np.uint8)
        
        # Background subtraction
        diff = cv2.absdiff(current_frame, background)
        gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        
        # Apply threshold
        _, thresh = cv2.threshold(gray_diff, 30, 255, cv2.THRESH_BINARY)
        
        # Morphological operations to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        
        # Calculate detection confidence
        changed_pixels = cv2.countNonZero(thresh)
        total_pixels = thresh.shape[0] * thresh.shape[1]
        confidence = changed_pixels / total_pixels
        
        # Detection based on threshold
        detected = confidence > self.detection_threshold
        
        return detected, confidence
    
    def process_detection(self):
        """Main detection processing loop"""
        if self.current_frame1 is None or self.current_frame2 is None:
            return
        
        if not self.background_learned:
            return
        
        # Detect objects in both cameras
        detected1, conf1 = self.detect_object_change(
            self.current_frame1, self.background_frames, 'cam1')
        detected2, conf2 = self.detect_object_change(
            self.current_frame2, self.background_frames, 'cam2')
        
        # Update detection results
        self.detection_results = {
            'cam1': detected1,
            'cam2': detected2,
            'confidence1': conf1,
            'confidence2': conf2
        }
        
        # Overall detection (both cameras or any camera)
        object_detected = detected1 or detected2
        max_confidence = max(conf1, conf2)
        
        # Publish results
        self.object_detected_pub.publish(Bool(data=object_detected))
        self.object_confidence_pub.publish(Float32(data=max_confidence))
        
        # Adaptive grip force based on confidence
        if self.adaptive_grip_enabled and object_detected:
            grip_force = self.calculate_adaptive_grip_force(max_confidence)
            self.grip_force_pub.publish(Float32(data=grip_force))
        
        # Status message
        status = f"Cam1: {'✅' if detected1 else '❌'} ({conf1:.3f}) | " \
                f"Cam2: {'✅' if detected2 else '❌'} ({conf2:.3f}) | " \
                f"Overall: {'DETECTED' if object_detected else 'CLEAR'}"
        self.status_pub.publish(String(data=status))
    
    def calculate_adaptive_grip_force(self, confidence):
        """Calculate adaptive grip force based on detection confidence"""
        # Map confidence to grip force (higher confidence = gentler grip)
        normalized_conf = min(max(confidence, 0.0), 1.0)
        # Inverse relationship: higher confidence -> lower force (more gentle)
        force_ratio = 1.0 - normalized_conf
        grip_force = self.min_grip_force + (self.max_grip_force - self.min_grip_force) * force_ratio
        return grip_force
    
    def send_gripper_command(self, position, effort):
        """Send command to gripper"""
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn("Gripper action server not available")
            return False
        
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort
        
        future = self.gripper_client.send_goal_async(goal)
        self.get_logger().info(f"🤏 Gripper command: pos={position:.3f}, effort={effort:.1f}")
        return True
    
    def get_detection_frame1_with_overlay(self):
        """Get camera 1 frame with detection overlay"""
        if self.current_frame1 is None:
            return None
        
        frame = self.current_frame1.copy()
        
        # Add detection overlay
        color = (0, 255, 0) if self.detection_results['cam1'] else (0, 0, 255)
        status_text = f"CAM1: {'OBJECT' if self.detection_results['cam1'] else 'CLEAR'} " \
                     f"({self.detection_results['confidence1']:.3f})"
        
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return frame
    
    def get_detection_frame2_with_overlay(self):
        """Get camera 2 frame with detection overlay"""
        if self.current_frame2 is None:
            return None
        
        frame = self.current_frame2.copy()
        
        # Add detection overlay
        color = (0, 255, 0) if self.detection_results['cam2'] else (0, 0, 255)
        status_text = f"CAM2: {'OBJECT' if self.detection_results['cam2'] else 'CLEAR'} " \
                     f"({self.detection_results['confidence2']:.3f})"
        
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return frame

    def setup_cameras(self):
        """Setup cameras - try real cameras first, fallback to simulation"""
        self.get_logger().info("🎥 Setting up cameras...")
        
        # Try to open real cameras
        try:
            # Check if camera devices exist
            import os
            if os.path.exists('/dev/video0') or os.path.exists('/dev/video1'):
                self.get_logger().info("📹 Real camera devices found")
                self.simulation_mode = False
            else:
                self.get_logger().warn("⚠️ No real camera devices found, using simulation mode")
                self.simulation_mode = True
                
        except Exception as e:
            self.get_logger().warn(f"⚠️ Camera setup error: {e}, using simulation mode")
            self.simulation_mode = True
        
        if self.simulation_mode:
            self.setup_simulated_cameras()
    
    def setup_simulated_cameras(self):
        """Setup simulated cameras with dummy frames"""
        self.get_logger().info("🎮 Setting up simulated cameras...")
        
        # Create dummy frames (640x480 BGR)
        self.dummy_frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
        self.dummy_frame2 = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add some pattern to make it interesting
        cv2.putText(self.dummy_frame1, "SIMULATED CAM1", (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(self.dummy_frame2, "SIMULATED CAM2", (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Add some texture
        cv2.circle(self.dummy_frame1, (320, 240), 100, (100, 100, 100), -1)
        cv2.circle(self.dummy_frame2, (320, 240), 100, (100, 100, 100), -1)
        
        self.current_frame1 = self.dummy_frame1.copy()
        self.current_frame2 = self.dummy_frame2.copy()
        self.cameras_available = True
        
        # Start timer to simulate camera updates
        self.sim_camera_timer = self.create_timer(0.033, self.update_simulated_cameras)  # ~30 FPS
        
        self.get_logger().info("✅ Simulated cameras ready")
    
    def update_simulated_cameras(self):
        """Update simulated camera frames with some variation"""
        if self.simulation_mode:
            # Add some noise to simulate real camera variation
            noise1 = np.random.randint(-5, 5, self.dummy_frame1.shape, dtype=np.int16)
            noise2 = np.random.randint(-5, 5, self.dummy_frame2.shape, dtype=np.int16)
            
            self.current_frame1 = np.clip(self.dummy_frame1.astype(np.int16) + noise1, 0, 255).astype(np.uint8)
            self.current_frame2 = np.clip(self.dummy_frame2.astype(np.int16) + noise2, 0, 255).astype(np.uint8)
            
            # Simulate object detection randomly (very low probability)
            if np.random.random() < 0.01:  # 1% chance per frame
                # Add a temporary "object" (white circle)
                cv2.circle(self.current_frame1, (np.random.randint(100, 540), np.random.randint(100, 380)), 
                          30, (255, 255, 255), -1)
    
    def check_camera_status(self):
        """Check camera connection status"""
        current_time = time.time()
        
        if not self.simulation_mode:
            # Check if we're receiving frames from real cameras
            if self.current_frame1 is not None:
                self.camera1_connected = True
                self.last_cam1_time = current_time
            elif current_time - self.last_cam1_time > self.camera_timeout:
                self.camera1_connected = False
            
            if self.current_frame2 is not None:
                self.camera2_connected = True
                self.last_cam2_time = current_time
            elif current_time - self.last_cam2_time > self.camera_timeout:
                self.camera2_connected = False
            
            self.cameras_available = self.camera1_connected and self.camera2_connected
            
            if not self.cameras_available:
                self.get_logger().warn(f"⚠️ Cameras status - Cam1: {self.camera1_connected}, Cam2: {self.camera2_connected}")
        
        # Log status periodically
        if hasattr(self, '_last_status_log'):
            if current_time - self._last_status_log > 10.0:  # Every 10 seconds
                mode = "Simulation" if self.simulation_mode else "Real"
                self.get_logger().info(f"📊 Camera status - Mode: {mode}, Available: {self.cameras_available}")
                self._last_status_log = current_time
        else:
            self._last_status_log = current_time
    
    def try_auto_learn_background(self):
        """Try to automatically learn background when cameras are ready"""
        if self.cameras_available and not self.background_learned:
            if self.current_frame1 is not None and self.current_frame2 is not None:
                self.get_logger().info("🎯 Auto-learning background now that cameras are ready...")
                # Run in separate thread to avoid blocking
                def learn_thread():
                    time.sleep(1.0)  # Give cameras a moment to stabilize
                    self.learn_background()
                
                threading.Thread(target=learn_thread, daemon=True).start()
                # Cancel this timer since we only want to run once
                self.background_learn_timer.cancel()


class TactileDetectorApp(App):
    """Kivy app for visual interface"""
    
    def __init__(self, detector_node, **kwargs):
        super().__init__(**kwargs)
        self.detector_node = detector_node
        self.title = "Tactile Object Detector - Tension Control"
    
    def build(self):
        root = BoxLayout(orientation='vertical', spacing=dp(10), padding=dp(10))
        
        # Status bar
        status_layout = BoxLayout(orientation='horizontal', size_hint_y=None, height=dp(50))
        self.status_label = Label(
            text="🔍 Tactile Object Detector Ready",
            size_hint_y=None, height=dp(50)
        )
        status_layout.add_widget(self.status_label)
        root.add_widget(status_layout)
        
        # Camera views
        cameras_layout = BoxLayout(orientation='horizontal', size_hint_y=0.6)
        
        # Camera 1
        cam1_layout = BoxLayout(orientation='vertical')
        cam1_layout.add_widget(Label(text="Camera 1 (Tactile)", size_hint_y=None, height=dp(30)))
        self.camera1_widget = KivyImage()
        cam1_layout.add_widget(self.camera1_widget)
        cameras_layout.add_widget(cam1_layout)
        
        # Camera 2
        cam2_layout = BoxLayout(orientation='vertical')
        cam2_layout.add_widget(Label(text="Camera 2 (Tactile)", size_hint_y=None, height=dp(30)))
        self.camera2_widget = KivyImage()
        cam2_layout.add_widget(self.camera2_widget)
        cameras_layout.add_widget(cam2_layout)
        
        root.add_widget(cameras_layout)
        
        # Controls
        controls_layout = BoxLayout(orientation='vertical', size_hint_y=0.4, spacing=dp(10))
        
        # Background learning
        bg_layout = BoxLayout(orientation='horizontal', size_hint_y=None, height=dp(40))
        self.learn_bg_btn = Button(text="Learn Background", size_hint=(0.3, 1))
        self.learn_bg_btn.bind(on_press=self.learn_background)
        bg_layout.add_widget(self.learn_bg_btn)
        
        self.bg_status_label = Label(text="Background: Not learned", size_hint=(0.7, 1))
        bg_layout.add_widget(self.bg_status_label)
        controls_layout.add_widget(bg_layout)
        
        # Detection threshold
        threshold_layout = BoxLayout(orientation='horizontal', size_hint_y=None, height=dp(40))
        threshold_layout.add_widget(Label(text="Threshold:", size_hint=(0.2, 1)))
        self.threshold_slider = Slider(min=0.1, max=1.0, value=0.6, size_hint=(0.6, 1))
        self.threshold_slider.bind(value=self.on_threshold_change)
        threshold_layout.add_widget(self.threshold_slider)
        self.threshold_label = Label(text="0.6", size_hint=(0.2, 1))
        threshold_layout.add_widget(self.threshold_label)
        controls_layout.add_widget(threshold_layout)
        
        # Adaptive grip
        grip_layout = BoxLayout(orientation='horizontal', size_hint_y=None, height=dp(40))
        grip_layout.add_widget(Label(text="Adaptive Grip:", size_hint=(0.3, 1)))
        self.adaptive_switch = Switch(active=True, size_hint=(0.2, 1))
        self.adaptive_switch.bind(active=self.on_adaptive_change)
        grip_layout.add_widget(self.adaptive_switch)
        self.grip_force_label = Label(text="Force: 0.0N", size_hint=(0.5, 1))
        grip_layout.add_widget(self.grip_force_label)
        controls_layout.add_widget(grip_layout)
        
        # Gripper controls
        gripper_layout = BoxLayout(orientation='horizontal', size_hint_y=None, height=dp(40))
        
        open_btn = Button(text="Open Gripper", size_hint=(0.25, 1))
        open_btn.bind(on_press=lambda x: self.gripper_command(0.08, 20.0))
        gripper_layout.add_widget(open_btn)
        
        close_btn = Button(text="Close Gripper", size_hint=(0.25, 1))
        close_btn.bind(on_press=lambda x: self.gripper_command(0.0, 20.0))
        gripper_layout.add_widget(close_btn)
        
        gentle_btn = Button(text="Gentle Grip", size_hint=(0.25, 1))
        gentle_btn.bind(on_press=lambda x: self.gripper_command(0.01, 10.0))
        gripper_layout.add_widget(gentle_btn)
        
        firm_btn = Button(text="Firm Grip", size_hint=(0.25, 1))
        firm_btn.bind(on_press=lambda x: self.gripper_command(0.0, 30.0))
        gripper_layout.add_widget(firm_btn)
        
        controls_layout.add_widget(gripper_layout)
        
        root.add_widget(controls_layout)
        
        # Start update timer
        Clock.schedule_interval(self.update_display, 1/30.0)
        
        return root
    
    def learn_background(self, instance):
        """Learn background in separate thread"""
        def learn_thread():
            self.detector_node.learn_background()
            Clock.schedule_once(lambda dt: setattr(self.bg_status_label, 'text', 
                                                 "Background: ✅ Learned"), 0)
        
        threading.Thread(target=learn_thread, daemon=True).start()
        self.bg_status_label.text = "Background: 📚 Learning..."
    
    def on_threshold_change(self, instance, value):
        """Update detection threshold"""
        self.detector_node.detection_threshold = value
        self.threshold_label.text = f"{value:.2f}"
    
    def on_adaptive_change(self, instance, active):
        """Toggle adaptive grip"""
        self.detector_node.adaptive_grip_enabled = active
    
    def gripper_command(self, position, effort):
        """Send gripper command"""
        self.detector_node.send_gripper_command(position, effort)
    
    def update_display(self, dt):
        """Update camera displays"""
        # Update camera 1
        frame1 = self.detector_node.get_detection_frame1_with_overlay()
        if frame1 is not None:
            self.update_camera_widget(self.camera1_widget, frame1)
        
        # Update camera 2
        frame2 = self.detector_node.get_detection_frame2_with_overlay()
        if frame2 is not None:
            self.update_camera_widget(self.camera2_widget, frame2)
        
        # Update status
        results = self.detector_node.detection_results
        overall_detected = results['cam1'] or results['cam2']
        max_conf = max(results['confidence1'], results['confidence2'])
        
        status = f"{'🟢 OBJECT DETECTED' if overall_detected else '🔴 NO OBJECT'} " \
                f"(Conf: {max_conf:.3f})"
        self.status_label.text = status
        
        # Update grip force display
        if overall_detected and self.detector_node.adaptive_grip_enabled:
            force = self.detector_node.calculate_adaptive_grip_force(max_conf)
            self.grip_force_label.text = f"Force: {force:.1f}N"
        else:
            self.grip_force_label.text = "Force: --"
    
    def update_camera_widget(self, widget, frame):
        """Update Kivy image widget with OpenCV frame"""
        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Create texture
        texture = Texture.create(size=(frame_rgb.shape[1], frame_rgb.shape[0]), colorfmt='rgb')
        texture.blit_buffer(frame_rgb.tobytes(), colorfmt='rgb', bufferfmt='ubyte')
        texture.flip_vertical()
        
        widget.texture = texture


def main():
    """Main function"""
    rclpy.init()
    
    try:
        # Create detector node
        detector_node = TactileObjectDetectorNode()
        
        # Run ROS2 node in separate thread
        def ros_thread():
            rclpy.spin(detector_node)
        
        ros_executor = threading.Thread(target=ros_thread, daemon=True)
        ros_executor.start()
        
        # Run Kivy app in main thread
        app = TactileDetectorApp(detector_node)
        app.run()
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'detector_node' in locals():
            detector_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
