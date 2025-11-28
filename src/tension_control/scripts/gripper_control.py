#!/usr/bin/env python3
"""
Quick Gripper Control Utility
Usage examples:
  python3 gripper_control.py close
  python3 gripper_control.py open
  python3 gripper_control.py position 0.03
  python3 gripper_control.py grasp 15.0
"""

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from rclpy.node import Node
import sys
import argparse


class QuickGripperControl:
    def __init__(self):
        rclpy.init()
        self.node = Node('quick_gripper_control')
        self.action_client = ActionClient(
            self.node,
            GripperCommand,
            '/panda_gripper/gripper_action'
        )
        
    def connect(self):
        """Connect to gripper action server"""
        print("🔌 Connecting to gripper...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            print("❌ Error: Gripper not available!")
            return False
        print("✅ Connected!")
        return True
        
    def send_command(self, position, effort=20.0):
        """Send gripper command"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = effort
        
        print(f"🎯 Command: position={position}m, effort={effort}N")
        
        # Send goal
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ Command rejected!")
            return False
            
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        
        result = result_future.result().result
        if result:
            print(f"✅ Success! Position: {result.position:.4f}m, Effort: {result.effort:.2f}N")
            return True
        else:
            print("❌ Command failed!")
            return False
            
    def close(self):
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Quick Gripper Control')
    parser.add_argument('command', choices=['close', 'open', 'position', 'grasp'],
                       help='Gripper command')
    parser.add_argument('value', type=float, nargs='?', default=None,
                       help='Position value (for position command) or effort (for grasp)')
    
    args = parser.parse_args()
    
    gripper = QuickGripperControl()
    
    if not gripper.connect():
        sys.exit(1)
        
    try:
        if args.command == 'close':
            print("🔒 Closing gripper...")
            success = gripper.send_command(0.0, 20.0)
            
        elif args.command == 'open':
            print("🔓 Opening gripper...")
            success = gripper.send_command(0.9, 50.0)
            
        elif args.command == 'position':
            if args.value is None:
                print("❌ Error: position command requires a value (0.0-0.08)")
                sys.exit(1)
            if not (0.0 <= args.value <= 0.08):
                print("❌ Error: position must be between 0.0 and 0.08 meters")
                sys.exit(1)
            print(f"📍 Moving to position {args.value}m...")
            success = gripper.send_command(args.value, 20.0)
            
        elif args.command == 'grasp':
            effort = args.value if args.value is not None else 15.0
            if effort > 70.0:
                print("❌ Error: effort too high! Maximum is 70N")
                sys.exit(1)
            print(f"🤏 Grasping with {effort}N force...")
            success = gripper.send_command(0.0, effort)
            
        if success:
            print("🎉 Command completed successfully!")
        else:
            print("💥 Command failed!")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n⏹️ Interrupted by user")
    except Exception as e:
        print(f"💥 Error: {e}")
        sys.exit(1)
    finally:
        gripper.close()


if __name__ == '__main__':
    main()
