#!/usr/bin/env python3
"""
Script to publish circular motion velocity commands to the Cartesian velocity controller.
This generates circular motion in the YZ plane.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
import time


class CircularVelocityPublisher(Node):
    def __init__(self):
        super().__init__('circular_velocity_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(
            Twist, 
            '/cartesian_velocity_command', 
            10
        )
        
        # Create subscriber for force feedback
        self.force_subscriber = self.create_subscription(
            Float32MultiArray,
            '/feats/cam1/total_force',
            self.force_callback,
            10
        )
        
        # Timer for publishing at 100Hz (matching typical control rate)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # Circular motion parameters
        self.omega = 2.0 * math.pi / 4.0  # Angular velocity (1 cycle in 4 seconds)
        self.radius = 0.1 / self.omega    # Radius to achieve max velocity of 0.05 m/s
        self.ramp_time = 2.0               # Smooth ramp-up time
        
        # Force-based stopping parameters
        self.force_threshold = -1.0        # Force threshold to trigger stopping
        self.force_stable_threshold = -0.5 # Force threshold to resume motion
        self.current_force = 0.0           # Current force value
        self.stopping = False              # Flag to indicate stopping mode
        self.resuming = False              # Flag to indicate resuming mode
        self.stop_start_time = None        # Time when stopping was triggered
        self.resume_start_time = None      # Time when resuming was triggered
        self.stop_ramp_time = 1.0          # Time to ramp down to zero
        self.resume_ramp_time = 1.0        # Time to ramp up from zero
        
        # Time tracking
        self.start_time = time.time()
        
        self.get_logger().info('Circular velocity publisher started')
        self.get_logger().info(f'Publishing to: /cartesian_velocity_command')
        self.get_logger().info(f'Listening to: /feats/cam1/total_force')
        self.get_logger().info(f'Force threshold: {self.force_threshold}')
        self.get_logger().info(f'Radius: {self.radius:.4f} m, Omega: {self.omega:.4f} rad/s')
        
    def force_callback(self, msg):
        """Callback for force feedback"""
        # Assuming the force is in the first element of the array
        if len(msg.data) > 0:
            self.current_force = msg.data[2]
            
            # Check if force threshold is exceeded and we're not already stopping
            if self.current_force < self.force_threshold and not self.stopping:
                self.stopping = True
                self.resuming = False
                self.stop_start_time = time.time()
                self.get_logger().warn(f'Force threshold exceeded! Force: {self.current_force:.3f} N - Starting ramp down')
            
            # Check if force is stable again and we're stopped
            elif self.current_force >= self.force_stable_threshold and self.stopping and not self.resuming:
                # Check if we have completely stopped first
                if self.stop_start_time is not None:
                    stop_elapsed = time.time() - self.stop_start_time
                    if stop_elapsed >= self.stop_ramp_time:
                        self.resuming = True
                        self.stopping = False
                        self.resume_start_time = time.time()
                        self.get_logger().info(f'Force stable again! Force: {self.current_force:.3f} N - Resuming motion')
    
    def timer_callback(self):
        # Calculate elapsed time
        t = time.time() - self.start_time
        
        # Smooth ramp-up factor using cosine function
        amplitude_factor = 1.0
        if t < self.ramp_time:
            amplitude_factor = 0.5 * (1.0 - math.cos(math.pi * t / self.ramp_time))
        
        # Apply stopping ramp if triggered
        if self.stopping:
            stop_elapsed = time.time() - self.stop_start_time
            if stop_elapsed < self.stop_ramp_time:
                # Cosine ramp down from 1 to 0
                stop_factor = 0.5 * (1.0 + math.cos(math.pi * stop_elapsed / self.stop_ramp_time))
                amplitude_factor *= stop_factor
            else:
                # Completely stopped
                amplitude_factor = 0.0
        
        # Apply resuming ramp if triggered
        elif self.resuming:
            resume_elapsed = time.time() - self.resume_start_time
            if resume_elapsed < self.resume_ramp_time:
                # Cosine ramp up from 0 to 1
                resume_factor = 0.5 * (1.0 - math.cos(math.pi * resume_elapsed / self.resume_ramp_time))
                amplitude_factor *= resume_factor
            else:
                # Fully resumed
                self.resuming = False
                self.get_logger().info('Motion fully resumed')
        
        # Circular motion equations in YZ plane
        # vy(t) = -R * omega * sin(omega * t)
        # vz(t) =  R * omega * cos(omega * t)
        v_y = -self.radius * self.omega * math.sin(self.omega * t) * amplitude_factor
        v_z = self.radius * self.omega * math.cos(self.omega * t) * amplitude_factor
        
        # Create and publish Twist message
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = v_y
        msg.linear.z = v_z
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        self.publisher.publish(msg)
        
        # Log every second
        if int(t * 10) % 10 == 0:
            if self.stopping:
                self.get_logger().info(f't={t:.2f}s: v_y={v_y:.4f}, v_z={v_z:.4f}, force={self.current_force:.3f} N [STOPPING]')
            else:
                self.get_logger().info(f't={t:.2f}s: v_y={v_y:.4f}, v_z={v_z:.4f}, force={self.current_force:.3f} N')


def main(args=None):
    rclpy.init(args=args)
    
    node = CircularVelocityPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
