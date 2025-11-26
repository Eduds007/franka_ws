#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float32, String
import math
import numpy as np

# In a real implementation, these would be imported from the action package
# For now, using the classes defined in the primitive file
from slide_along_cable_primitive import SlideAlongCableGoal, SlideAlongCableResult, SlideAlongCableFeedback

class SlideAlongCableClient(Node):
    def __init__(self):
        super().__init__('slide_along_cable_client')
        
        # Subscribers to monitor primitive output
        self.velocity_sub = self.create_subscription(
            String, '/slide_primitive/feedback', self.feedback_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/slide_primitive/status', self.status_callback, 10)
        
        # Service client to control primitive
        # In real implementation would use Action client
        
        self.get_logger().info("SlideAlongCable client initialized")
        
    def create_slide_goal(self, start_xyz, end_xyz, start_orientation=None, end_orientation=None):
        """Create a slide along cable goal with simplified parameters"""
        goal = SlideAlongCableGoal()
        
        # Start pose
        goal.start_pose.position.x = start_xyz[0]
        goal.start_pose.position.y = start_xyz[1] 
        goal.start_pose.position.z = start_xyz[2]
        
        if start_orientation is None:
            # Default orientation (pointing down for cable holding)
            goal.start_pose.orientation.x = 0.0
            goal.start_pose.orientation.y = 0.707  # 90 degrees around Y
            goal.start_pose.orientation.z = 0.0
            goal.start_pose.orientation.w = 0.707
        else:
            goal.start_pose.orientation.x = start_orientation[0]
            goal.start_pose.orientation.y = start_orientation[1]
            goal.start_pose.orientation.z = start_orientation[2]
            goal.start_pose.orientation.w = start_orientation[3]
        
        # End pose
        goal.end_pose.position.x = end_xyz[0]
        goal.end_pose.position.y = end_xyz[1]
        goal.end_pose.position.z = end_xyz[2]
        
        if end_orientation is None:
            # Same orientation as start
            goal.end_pose.orientation = goal.start_pose.orientation
        else:
            goal.end_pose.orientation.x = end_orientation[0]
            goal.end_pose.orientation.y = end_orientation[1]
            goal.end_pose.orientation.z = end_orientation[2]
            goal.end_pose.orientation.w = end_orientation[3]
        
        return goal
    
    def execute_slide_along_cable(self, start_xyz, end_xyz, tension=5.0, velocity=0.05):
        """Execute a slide along cable movement"""
        
        # Create goal
        goal = self.create_slide_goal(start_xyz, end_xyz)
        
        # Set movement parameters
        goal.desired_tension = tension      # 5 N desired tension
        goal.slide_velocity = velocity      # 5 cm/s sliding speed
        goal.max_tension_limit = tension * 1.5  # 150% safety limit
        goal.position_tolerance = 0.005     # 5 mm position accuracy
        goal.tension_tolerance = 0.5        # 0.5 N tension accuracy
        goal.stop_criterion = "both"        # Stop when both position and tension are achieved
        
        # Calculate distance and time
        distance = math.sqrt(sum((end_xyz[i] - start_xyz[i])**2 for i in range(3)))
        estimated_time = distance / velocity
        
        self.get_logger().info(f"Executing slide along cable:")
        self.get_logger().info(f"  Start: [{start_xyz[0]:.3f}, {start_xyz[1]:.3f}, {start_xyz[2]:.3f}]")
        self.get_logger().info(f"  End: [{end_xyz[0]:.3f}, {end_xyz[1]:.3f}, {end_xyz[2]:.3f}]")
        self.get_logger().info(f"  Distance: {distance:.3f} m")
        self.get_logger().info(f"  Tension: {tension:.1f} N")
        self.get_logger().info(f"  Velocity: {velocity:.3f} m/s")
        self.get_logger().info(f"  Estimated time: {estimated_time:.1f} s")
        
        # In real implementation, would send goal to action server
        # For now, just log the goal parameters
        return goal
    
    def slide_horizontal(self, start_x, start_y, start_z, distance, tension=5.0, velocity=0.05):
        """Slide horizontally along cable"""
        start_xyz = [start_x, start_y, start_z]
        end_xyz = [start_x + distance, start_y, start_z]
        return self.execute_slide_along_cable(start_xyz, end_xyz, tension, velocity)
    
    def slide_vertical(self, start_x, start_y, start_z, distance, tension=5.0, velocity=0.05):
        """Slide vertically along cable"""
        start_xyz = [start_x, start_y, start_z]
        end_xyz = [start_x, start_y, start_z + distance]
        return self.execute_slide_along_cable(start_xyz, end_xyz, tension, velocity)
    
    def slide_along_curve(self, waypoints, tension=5.0, velocity=0.05):
        """Slide along a curved path defined by waypoints"""
        self.get_logger().info(f"Planning curved slide with {len(waypoints)} waypoints")
        
        goals = []
        for i in range(len(waypoints) - 1):
            start_xyz = waypoints[i]
            end_xyz = waypoints[i + 1]
            goal = self.execute_slide_along_cable(start_xyz, end_xyz, tension, velocity)
            goals.append(goal)
        
        return goals
    
    def emergency_stop(self):
        """Send emergency stop command to primitive"""
        self.get_logger().warn("Sending emergency stop command")
        # In real implementation, would call stop service
        
    def feedback_callback(self, msg):
        """Handle primitive feedback"""
        self.get_logger().info(f"Primitive feedback: {msg.data}")
    
    def status_callback(self, msg):
        """Handle primitive status updates"""
        self.get_logger().info(f"Primitive status: {msg.data}")


def demo_slide_movements():
    """Demonstrate various slide movements"""
    rclpy.init()
    
    client = SlideAlongCableClient()
    
    try:
        # Demo 1: Simple horizontal slide
        print("\n=== Demo 1: Horizontal Slide ===")
        goal1 = client.slide_horizontal(
            start_x=0.5, start_y=0.0, start_z=0.3,
            distance=0.2,  # 20 cm slide
            tension=4.0,   # 4 N tension
            velocity=0.03  # 3 cm/s
        )
        
        # Demo 2: Vertical slide (cable hanging)
        print("\n=== Demo 2: Vertical Slide ===")
        goal2 = client.slide_vertical(
            start_x=0.5, start_y=0.0, start_z=0.5,
            distance=-0.15,  # 15 cm downward
            tension=6.0,     # 6 N tension
            velocity=0.02    # 2 cm/s
        )
        
        # Demo 3: Curved path slide
        print("\n=== Demo 3: Curved Path Slide ===")
        waypoints = [
            [0.4, 0.0, 0.4],   # Start
            [0.5, 0.1, 0.35],  # Curve right and down
            [0.6, 0.0, 0.3],   # Curve back and down
            [0.7, -0.1, 0.25], # Curve left and down
            [0.8, 0.0, 0.2]    # End straight
        ]
        goals3 = client.slide_along_curve(waypoints, tension=5.5, velocity=0.025)
        
        # Demo 4: Precision positioning
        print("\n=== Demo 4: Precision Positioning ===")
        goal4 = client.execute_slide_along_cable(
            start_xyz=[0.6, 0.0, 0.3],
            end_xyz=[0.65, 0.02, 0.295],  # Small precise movement
            tension=3.0,
            velocity=0.01  # Very slow for precision
        )
        
        print("\n=== All demos planned successfully ===")
        print("In real implementation, these would be sent to the action server")
        
        # Keep node alive to receive feedback
        rclpy.spin(client)
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
        client.emergency_stop()
    finally:
        client.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Run interactive slide primitive client"""
    rclpy.init(args=args)
    
    client = SlideAlongCableClient()
    
    print("\n=== Slide Along Cable Primitive Client ===")
    print("Available commands:")
    print("1. slide_horizontal(x, y, z, distance, tension, velocity)")
    print("2. slide_vertical(x, y, z, distance, tension, velocity)")
    print("3. execute_slide_along_cable(start_xyz, end_xyz, tension, velocity)")
    print("4. emergency_stop()")
    print("5. demo - Run demonstration movements")
    print("Type 'exit' to quit")
    
    try:
        while True:
            command = input("\nEnter command: ").strip()
            
            if command == 'exit':
                break
            elif command == 'demo':
                demo_slide_movements()
                break
            elif command.startswith('slide_horizontal'):
                # Parse command: slide_horizontal(0.5, 0.0, 0.3, 0.2, 4.0, 0.03)
                try:
                    # Simple parsing - in real app would use proper parser
                    params = command[command.find('(')+1:command.find(')')].split(',')
                    params = [float(p.strip()) for p in params]
                    
                    if len(params) >= 4:
                        tension = params[4] if len(params) > 4 else 5.0
                        velocity = params[5] if len(params) > 5 else 0.05
                        goal = client.slide_horizontal(params[0], params[1], params[2], params[3], tension, velocity)
                    else:
                        print("Usage: slide_horizontal(x, y, z, distance, [tension], [velocity])")
                except Exception as e:
                    print(f"Error parsing command: {e}")
            
            elif command == 'emergency_stop':
                client.emergency_stop()
            
            else:
                print("Unknown command. Type 'demo' for demonstration or 'exit' to quit.")
        
        rclpy.spin(client)
        
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Run demo by default
    demo_slide_movements()