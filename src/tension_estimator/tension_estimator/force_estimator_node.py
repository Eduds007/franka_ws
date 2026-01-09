#!/usr/bin/env python3
"""
FEATS Force Estimator ROS2 Node
Subscribes to camera images and publishes force estimates
"""

import sys
import os

# Add FEATS to path
feats_path = "/home/nuc_6g_life_3/franka_ws/src/feats/src/feats/src"
if feats_path not in sys.path:
    sys.path.insert(0, feats_path)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import yaml

from data.dataloader import normalize, unnormalize
from models.unet import UNet


class ForceEstimatorNode(Node):
    """
    ROS2 node that estimates forces from tactile camera images using FEATS.
    """
    
    def __init__(self):
        super().__init__('feats_force_estimator')
        
        # Declare parameters
        self.declare_parameter('config_file', 
            '/home/nuc_6g_life_3/franka_ws/src/feats/src/feats/src/predict/predict_config.yaml')
        self.declare_parameter('camera1_topic', '/gelsight_cam1/image_raw')
        self.declare_parameter('camera2_topic', '/gelsight_cam2/image_raw')
        
        # Get parameters
        config_file = self.get_parameter('config_file').value
        topic1 = self.get_parameter('camera1_topic').value
        topic2 = self.get_parameter('camera2_topic').value
        
        # Load configuration
        self.get_logger().info(f"Loading config from: {config_file}")
        self.config = yaml.load(open(config_file, 'r'), Loader=yaml.FullLoader)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Device setup
        if torch.backends.mps.is_available():
            self.device = torch.device("mps")
        else:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.get_logger().info(f"🖥️  Using device: {self.device}")
        
        # Load model
        self.model = UNet(
            enc_chs=self.config["enc_chs"], 
            dec_chs=self.config["dec_chs"], 
            out_sz=self.config["output_size"]
        )
        
        model_path = self.config["model"]
        # Make path absolute if relative
        if not os.path.isabs(model_path):
            model_path = os.path.join(os.path.dirname(config_file), model_path)
        
        self.get_logger().info(f"Loading model from: {model_path}")
        self.model.load_state_dict(
            torch.load(model_path, map_location=torch.device("cpu"), weights_only=True)
        )
        self.model.eval().to(self.device)
        self.get_logger().info(f"✅ Model loaded successfully")
        
        # Subscribers
        self.sub1 = self.create_subscription(Image, topic1, self.camera1_callback, 10)
        self.sub2 = self.create_subscription(Image, topic2, self.camera2_callback, 10)
        
        # Publishers for force estimates
        self.pub_cam1_force = self.create_publisher(Float32MultiArray, '/feats/cam1/forces', 10)
        self.pub_cam2_force = self.create_publisher(Float32MultiArray, '/feats/cam2/forces', 10)
        self.pub_cam1_total = self.create_publisher(Float32MultiArray, '/feats/cam1/total_force', 10)
        self.pub_cam2_total = self.create_publisher(Float32MultiArray, '/feats/cam2/total_force', 10)
        
        # Statistics
        self.cam1_count = 0
        self.cam2_count = 0
        
        # Timer for status updates
        self.create_timer(5.0, self.print_status)
        
        self.get_logger().info("="*60)
        self.get_logger().info("📸 FEATS Force Estimator Node Started")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Subscribed to: {topic1}")
        self.get_logger().info(f"Subscribed to: {topic2}")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  • /feats/cam1/forces (force grids)")
        self.get_logger().info("  • /feats/cam2/forces (force grids)")
        self.get_logger().info("  • /feats/cam1/total_force (summed forces)")
        self.get_logger().info("  • /feats/cam2/total_force (summed forces)")
        self.get_logger().info("="*60)
    
    def process_image(self, img, imgw=320, imgh=240):
        """Process camera image"""
        if img is None:
            return None
        
        # Resize, crop and resize back
        img = cv2.resize(img, (895, 672))
        border_size_x = int(img.shape[0] * (1 / 7))
        border_size_y = int(np.floor(img.shape[1] * (1 / 7)))
        img = img[border_size_x:img.shape[0] - border_size_x, 
                  border_size_y:img.shape[1] - border_size_y]
        img = img[:, :-1]
        img = cv2.resize(img, (imgw, imgh))
        
        # Convert BGR to RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        return img
    
    def make_prediction(self, img):
        """Make force prediction"""
        # Store data in dictionary
        data = {}
        data["gs_img"] = img
        
        # Normalize data
        norm_file = self.config["norm_file"]
        if not os.path.isabs(norm_file):
            config_dir = os.path.dirname(self.get_parameter('config_file').value)
            norm_file = os.path.join(config_dir, norm_file)
        
        data = normalize(data, norm_file)
        
        # Convert to torch tensor
        gs_img = torch.from_numpy(data["gs_img"]).float()
        gs_img = gs_img.unsqueeze(0).permute(0, 3, 1, 2).to(self.device)
        
        # Load calibration if available
        if self.config.get("calibration_file") is not None:
            calibration = np.load(self.config["calibration_file"])
            rows, cols = 240, 320
            M = np.float32([[1, 0, calibration[0]], [0, 1, calibration[1]]])
            inputs_prewarp = data["gs_img"]
            inputs_warp = cv2.warpAffine(inputs_prewarp, M, (cols, rows), 
                                        borderMode=cv2.BORDER_REPLICATE)
            inputs = torch.from_numpy(inputs_warp).permute(2, 0, 1).unsqueeze(0).float().to(self.device)
        else:
            inputs = gs_img
        
        # Get model prediction
        with torch.no_grad():
            outputs = self.model(inputs)
        
        # Unnormalize outputs
        outputs_transf = outputs.squeeze(0).permute(1, 2, 0)
        pred_grid_x = unnormalize(outputs_transf[:, :, 0], "grid_x", norm_file)
        pred_grid_y = unnormalize(outputs_transf[:, :, 1], "grid_y", norm_file)
        pred_grid_z = unnormalize(outputs_transf[:, :, 2], "grid_z", norm_file)
        
        # Convert to numpy
        pred_grid_x = pred_grid_x.cpu().detach().numpy()
        pred_grid_y = pred_grid_y.cpu().detach().numpy()
        pred_grid_z = pred_grid_z.cpu().detach().numpy()
        
        return pred_grid_x, pred_grid_y, pred_grid_z
    
    def camera1_callback(self, msg):
        """Process camera 1 images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process image
            processed_img = self.process_image(cv_image)
            if processed_img is None:
                return
            
            # Make prediction
            pred_x, pred_y, pred_z = self.make_prediction(processed_img)
            
            # Calculate total forces
            total_x = np.sum(pred_x)
            total_y = np.sum(pred_y)
            total_z = np.sum(pred_z)
            
            # Publish force grids (flattened)
            force_msg = Float32MultiArray()
            force_msg.data = np.concatenate([
                pred_x.flatten(), 
                pred_y.flatten(), 
                pred_z.flatten()
            ]).tolist()
            self.pub_cam1_force.publish(force_msg)
            
            # Publish total forces
            total_msg = Float32MultiArray()
            total_msg.data = [float(total_x), float(total_y), float(total_z)]
            self.pub_cam1_total.publish(total_msg)
            
            self.cam1_count += 1
            
            # Log every 30 frames
            if self.cam1_count % 30 == 0:
                self.get_logger().info(
                    f"📊 Cam1 [{self.cam1_count}]: "
                    f"F_x={total_x:.3f}N, F_y={total_y:.3f}N, F_z={total_z:.3f}N"
                )
            
        except Exception as e:
            self.get_logger().error(f"Error processing camera 1: {e}")
    
    def camera2_callback(self, msg):
        """Process camera 2 images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process image
            processed_img = self.process_image(cv_image)
            if processed_img is None:
                return
            
            # Make prediction
            pred_x, pred_y, pred_z = self.make_prediction(processed_img)
            
            # Calculate total forces
            total_x = np.sum(pred_x)
            total_y = np.sum(pred_y)
            total_z = np.sum(pred_z)
            
            # Publish force grids (flattened)
            force_msg = Float32MultiArray()
            force_msg.data = np.concatenate([
                pred_x.flatten(), 
                pred_y.flatten(), 
                pred_z.flatten()
            ]).tolist()
            self.pub_cam2_force.publish(force_msg)
            
            # Publish total forces
            total_msg = Float32MultiArray()
            total_msg.data = [float(total_x), float(total_y), float(total_z)]
            self.pub_cam2_total.publish(total_msg)
            
            self.cam2_count += 1
            
            # Log every 30 frames
            if self.cam2_count % 30 == 0:
                self.get_logger().info(
                    f"📊 Cam2 [{self.cam2_count}]: "
                    f"F_x={total_x:.3f}N, F_y={total_y:.3f}N, F_z={total_z:.3f}N"
                )
            
        except Exception as e:
            self.get_logger().error(f"Error processing camera 2: {e}")
    
    def print_status(self):
        """Print periodic status"""
        self.get_logger().info(
            f"📈 Status: Cam1={self.cam1_count} frames, Cam2={self.cam2_count} frames processed"
        )


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = ForceEstimatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Force Estimator Node")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
