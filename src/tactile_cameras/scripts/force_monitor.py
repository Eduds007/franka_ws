#!/usr/bin/env python3
"""
Force Monitor Node
Simple subscriber that prints the sum of forces from both GelSight cameras
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ForceMonitorNode(Node):
    """Node for monitoring combined force from both GelSight cameras"""
    
    def __init__(self):
        super().__init__('force_monitor')
        
        # Current force values
        self.force1 = 0.0
        self.force2 = 0.0
        
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
        
        self.get_logger().info("📊 Force Monitor Node initialized")
        self.get_logger().info("📡 Listening to:")
        self.get_logger().info("   • /feats/cam1/total_force")
        self.get_logger().info("   • /feats/cam2/total_force")
        print("\n" + "="*60)
    
    def force1_callback(self, msg):
        """Callback for camera 1 total force"""
        # Float32MultiArray - pegar o primeiro valor ou a soma
        if len(msg.data) > 0:
            self.force1 = msg.data[0] if len(msg.data) == 1 else sum(msg.data)
        self.print_forces()
    
    def force2_callback(self, msg):
        """Callback for camera 2 total force"""
        # Float32MultiArray - pegar o primeiro valor ou a soma
        if len(msg.data) > 0:
            self.force2 = msg.data[0] if len(msg.data) == 1 else sum(msg.data)
        self.print_forces()
    
    def print_forces(self):
        """Print current forces and their sum"""
        total_force = self.force1 + self.force2
        print(f"Cam1: {self.force1:8.3f} N  |  Cam2: {self.force2:8.3f} N  |  Total: {total_force:8.3f} N")


def main():
    """Main function"""
    rclpy.init()
    
    try:
        monitor_node = ForceMonitorNode()
        
        print("\n" + "="*60)
        print("📊 GelSight Force Monitor")
        print("="*60)
        print("Monitorando forças dos sensores...")
        print("⌨️  Pressione Ctrl+C para sair")
        print("="*60 + "\n")
        
        rclpy.spin(monitor_node)
        
    except KeyboardInterrupt:
        print("\n\n🛑 Interrompido pelo usuário")
    finally:
        if 'monitor_node' in locals():
            monitor_node.destroy_node()
        rclpy.shutdown()
        print("✅ Node encerrado")


if __name__ == '__main__':
    main()
