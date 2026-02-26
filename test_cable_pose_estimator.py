#!/usr/bin/env python3
"""
Example script to test Cable Pose Estimator Node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger


class CablePoseMonitor(Node):
    """Simple monitor node to display cable pose data"""
    
    def __init__(self):
        super().__init__('cable_pose_monitor')
        
        # Subscribe to pose
        self.pose_sub = self.create_subscription(
            Pose2D,
            '/cable_pose_estimator/cable_pose',
            self.pose_callback,
            10
        )
        
        # Subscribe to contact detection
        self.contact_sub = self.create_subscription(
            Bool,
            '/cable_pose_estimator/contact_detected',
            self.contact_callback,
            10
        )
        
        # Subscribe to metrics
        self.metrics_sub = self.create_subscription(
            Float32MultiArray,
            '/cable_pose_estimator/cable_metrics',
            self.metrics_callback,
            10
        )
        
        self.contact_detected = False
        self.get_logger().info('Cable Pose Monitor started')
        self.get_logger().info('Waiting for data...')
    
    def contact_callback(self, msg: Bool):
        """Handle contact detection updates"""
        if msg.data != self.contact_detected:
            self.contact_detected = msg.data
            if self.contact_detected:
                self.get_logger().info('✓ Cable contact DETECTED')
            else:
                self.get_logger().info('✗ No cable contact')
    
    def pose_callback(self, msg: Pose2D):
        """Handle pose updates"""
        if self.contact_detected:
            self.get_logger().info(
                f'Pose: x={msg.x:.2f}mm, y={msg.y:.2f}mm, θ={msg.theta:.3f}rad ({msg.theta*57.3:.1f}°)'
            )
    
    def metrics_callback(self, msg: Float32MultiArray):
        """Handle detailed metrics updates"""
        if self.contact_detected and len(msg.data) >= 10:
            data = msg.data
            self.get_logger().info(
                f'Metrics: Center=({data[0]:.1f}, {data[1]:.1f})px, '
                f'Size={data[2]:.1f}×{data[3]:.1f}px, '
                f'Angle={data[4]:.2f}°, '
                f'Offset=({data[5]:.1f}, {data[6]:.1f})px, '
                f'Distance={data[7]:.1f}px, '
                f'Area={data[8]:.0f}px²'
            )


def capture_reference():
    """Helper function to capture reference image"""
    rclpy.init()
    node = rclpy.create_node('reference_capture_client')
    client = node.create_client(Trigger, '/cable_pose_estimator/capture_reference')
    
    node.get_logger().info('Waiting for capture_reference service...')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available!')
        node.destroy_node()
        rclpy.shutdown()
        return False
    
    request = Trigger.Request()
    node.get_logger().info('Capturing reference image...')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        response = future.result()
        if response.success:
            node.get_logger().info(f'✓ {response.message}')
            success = True
        else:
            node.get_logger().error(f'✗ {response.message}')
            success = False
    else:
        node.get_logger().error('Service call failed!')
        success = False
    
    node.destroy_node()
    rclpy.shutdown()
    return success


def main(args=None):
    """Main function"""
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'capture':
        # Capture reference mode
        
        capture_reference()
    else:
        # Monitor mode
        rclpy.init(args=args)
        node = CablePoseMonitor()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
