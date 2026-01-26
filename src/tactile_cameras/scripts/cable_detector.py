#!/usr/bin/env python3
"""
Cable Detector Node
Detects sudden changes in total force magnitude from GelSight cameras
to determine cable presence
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import math
from collections import deque


class CableDetectorNode(Node):
    """Node for detecting cable based on force magnitude changes"""
    
    def __init__(self):
        super().__init__('cable_detector')
        
        # Parameters
        self.declare_parameter('threshold', 2.0)  # Threshold for absolute force magnitude (N)
        self.declare_parameter('window_size', 10)  # Number of samples for moving average
        
        self.threshold = self.get_parameter('threshold').value
        self.window_size = self.get_parameter('window_size').value
        
        # Data buffers for moving average
        self.force_buffer = deque(maxlen=self.window_size)
        
        # Current force values
        self.force1_x = 0.0
        self.force1_y = 0.0
        self.force1_z = 0.0
        self.force2_x = 0.0
        self.force2_y = 0.0
        self.force2_z = 0.0
        
        # Detection state
        self.cable_detected = False
        
        # Subscribers
        self.force1_sub = self.create_subscription(
            Float32MultiArray,
            '/feats/cam1/total_force',
            self.force1_callback,
            10
        )
        
        self.force2_sub = self.create_subscription(
            Float32MultiArray,
            '/feats/cam2/total_force',
            self.force2_callback,
            10
        )
        
        # Publisher for cable status
        self.status_pub = self.create_publisher(Int32, '/cable_status', 10)
        
        # Timer for periodic checking
        self.timer = self.create_timer(0.01, self.check_cable_status)  # 100 Hz
        
        self.get_logger().info("🔌 Cable Detector Node initialized")
        self.get_logger().info(f"⚙️  Parameters:")
        self.get_logger().info(f"   • Threshold: {self.threshold} N")
        self.get_logger().info(f"   • Window size: {self.window_size}")
        self.get_logger().info("📡 Subscribing to:")
        self.get_logger().info("   • /feats/cam1/total_force")
        self.get_logger().info("   • /feats/cam2/total_force")
        self.get_logger().info("📤 Publishing to:")
        self.get_logger().info("   • /cable_status (Int32: 1=TEM CABO, 0=NÃO TEM CABO)")
    
    def force1_callback(self, msg):
        """Callback for camera 1 total force"""
        if len(msg.data) >= 3:
            self.force1_x = msg.data[0]
            self.force1_y = msg.data[1]
            self.force1_z = msg.data[2]
    
    def force2_callback(self, msg):
        """Callback for camera 2 total force"""
        if len(msg.data) >= 3:
            self.force2_x = msg.data[0]
            self.force2_y = msg.data[1]
            self.force2_z = msg.data[2]
    
    def calculate_total_magnitude(self):
        """Calculate total force magnitude from both cameras"""
        mag1 = math.sqrt(self.force1_x**2 + self.force1_y**2 + self.force1_z**2)
        mag2 = math.sqrt(self.force2_x**2 + self.force2_y**2 + self.force2_z**2)
        return mag1 + mag2
    
    def check_cable_status(self):
        """Check if cable is present based on absolute force magnitude"""
        # Calculate current total magnitude
        current_magnitude = self.calculate_total_magnitude()
        
        # Add to buffer
        self.force_buffer.append(current_magnitude)
        
        # Need at least window_size samples to start detection
        if len(self.force_buffer) < self.window_size:
            return
        
        # Calculate moving average
        current_avg = sum(self.force_buffer) / len(self.force_buffer)
        
        # Determine cable status based on threshold
        cable_present = current_avg > self.threshold
        
        # Publish status constantly (1 = TEM CABO, 0 = NÃO TEM CABO)
        status_msg = Int32()
        status_msg.data = 1 if cable_present else 0
        self.status_pub.publish(status_msg)
        
        # Log only when status changes
        if cable_present and not self.cable_detected:
            self.cable_detected = True
            self.get_logger().info(f"🔌 TEM CABO detectado! (Força = {current_avg:.3f} N) [Publicando: 1]")
        elif not cable_present and self.cable_detected:
            self.cable_detected = False
            self.get_logger().info(f"🔓 NÃO TEM CABO! (Força = {current_avg:.3f} N) [Publicando: 0]")


def main():
    """Main function"""
    rclpy.init()
    
    print("\n" + "="*60)
    print("🔌 Cable Detector - GelSight Force Monitoring")
    print("="*60)
    print("📡 Aguardando dados dos sensores...")
    print("📊 Detecta cabo por valor absoluto de força")
    print("\n💡 Regra de Detecção:")
    print(f"   🔌 Publica 1 = Força > 2.0 N (TEM CABO)")
    print(f"   🔓 Publica 0 = Força ≤ 2.0 N (NÃO TEM CABO)")
    print("\n📤 Tópico: /cable_status (Int32)")
    print("   Publicação contínua a 100 Hz")
    print("\n⌨️  Pressione Ctrl+C para sair")
    print("="*60 + "\n")
    
    node = CableDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Node shutdown complete")


if __name__ == '__main__':
    main()
