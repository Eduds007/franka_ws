#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaRobotState
from sensor_msgs.msg import JointState
import sys

class RobotCommunicationTester(Node):
    def __init__(self):
        super().__init__('robot_communication_tester')
        
        self.robot_state_received = False
        self.joint_state_received = False
        
        # Subscribe to robot state
        self.robot_state_sub = self.create_subscription(
            FrankaRobotState,
            '/franka_robot_state_broadcaster/robot_state',
            self.robot_state_callback,
            1
        )
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/franka/joint_states',
            self.joint_state_callback,
            1
        )
        
        # Timer to check status
        self.timer = self.create_timer(2.0, self.check_status)
        self.test_duration = 10.0  # Test for 10 seconds
        self.start_time = self.get_clock().now()
        
        self.get_logger().info("Robot Communication Tester Started")
        self.get_logger().info("Testing communication for 10 seconds...")

    def robot_state_callback(self, msg):
        if not self.robot_state_received:
            self.get_logger().info("✓ Robot state communication working!")
            self.get_logger().info(f"  - Robot mode: {msg.robot_mode}")
            self.get_logger().info(f"  - Safety state: {msg.safety_state}")
            self.robot_state_received = True

    def joint_state_callback(self, msg):
        if not self.joint_state_received:
            self.get_logger().info("✓ Joint state communication working!")
            self.get_logger().info(f"  - Number of joints: {len(msg.name)}")
            self.get_logger().info(f"  - Joint names: {msg.name}")
            self.joint_state_received = True

    def check_status(self):
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        if elapsed > self.test_duration:
            self.get_logger().info("\n=== Communication Test Results ===")
            if self.robot_state_received:
                self.get_logger().info("✓ Robot state: WORKING")
            else:
                self.get_logger().error("✗ Robot state: NOT WORKING")
                
            if self.joint_state_received:
                self.get_logger().info("✓ Joint states: WORKING")
            else:
                self.get_logger().error("✗ Joint states: NOT WORKING")
                
            if self.robot_state_received and self.joint_state_received:
                self.get_logger().info("🎉 Robot communication is FULLY WORKING!")
                sys.exit(0)
            else:
                self.get_logger().error("❌ Robot communication has ISSUES!")
                sys.exit(1)

def main():
    rclpy.init()
    
    try:
        tester = RobotCommunicationTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
