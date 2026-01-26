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
        
        # Current force values (x, y, z components)
        self.force1_x = 0.0
        self.force1_y = 0.0
        self.force1_z = 0.0
        self.force2_x = 0.0
        self.force2_y = 0.0
        self.force2_z = 0.0
        
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
        if len(msg.data) >= 3:
            self.force1_x = msg.data[0]
            self.force1_y = msg.data[1]
            self.force1_z = msg.data[2]
        self.print_forces()
    
    def force2_callback(self, msg):
        """Callback for camera 2 total force"""
        if len(msg.data) >= 3:
            self.force2_x = msg.data[0]
            self.force2_y = msg.data[1]
            self.force2_z = msg.data[2]
        self.print_forces()
    
    def print_forces(self):
        """Print current forces and their sum"""
        import math
        
        # Calculate magnitudes
        mag1 = math.sqrt(self.force1_x**2 + self.force1_y**2 + self.force1_z**2)
        mag2 = math.sqrt(self.force2_x**2 + self.force2_y**2 + self.force2_z**2)
        total_force = mag1 + mag2
        
        print(f"Cam1: X={self.force1_x:7.3f} Y={self.force1_y:7.3f} Z={self.force1_z:7.3f} |Mag|={mag1:7.3f} N  |  "
              f"Cam2: X={self.force2_x:7.3f} Y={self.force2_y:7.3f} Z={self.force2_z:7.3f} |Mag|={mag2:7.3f} N  |  "
              f"Total: {total_force:7.3f} N")


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
