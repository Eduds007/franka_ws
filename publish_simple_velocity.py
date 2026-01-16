#!/usr/bin/env python3
"""
Simple script to publish constant velocity commands to the Cartesian velocity controller.
Useful for testing and manual control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys


class SimpleVelocityPublisher(Node):
    def __init__(self, v_x, v_y, v_z):
        super().__init__('simple_velocity_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(
            Twist, 
            '/cartesian_velocity_command', 
            10
        )
        
        # Timer for publishing at 100Hz
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # Velocity commands
        self.v_x = v_x
        self.v_y = v_y
        self.v_z = v_z
        
        self.get_logger().info('Simple velocity publisher started')
        self.get_logger().info(f'Publishing v_x={v_x}, v_y={v_y}, v_z={v_z}')
        
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.v_x
        msg.linear.y = self.v_y
        msg.linear.z = self.v_z
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        self.publisher.publish(msg)


def main(args=None):
    if len(sys.argv) < 4:
        print("Usage: ./publish_simple_velocity.py <v_x> <v_y> <v_z>")
        print("Example: ./publish_simple_velocity.py 0.0 0.01 0.01")
        sys.exit(1)
    
    v_x = float(sys.argv[1])
    v_y = float(sys.argv[2])
    v_z = float(sys.argv[3])
    
    rclpy.init(args=args)
    
    node = SimpleVelocityPublisher(v_x, v_y, v_z)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
