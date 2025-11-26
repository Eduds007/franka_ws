#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point, Quaternion
from std_msgs.msg import Float32, String
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import math
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d

# Custom action message (would need to be defined in a separate package)
# For now, using basic types
class SlideAlongCableGoal:
    def __init__(self):
        self.start_pose = Pose()
        self.end_pose = Pose()
        self.desired_tension = 0.0
        self.slide_velocity = 0.0
        self.max_tension_limit = 0.0
        self.position_tolerance = 0.0
        self.tension_tolerance = 0.0
        self.stop_criterion = ""  # "position" or "tension" or "both"

class SlideAlongCableResult:
    def __init__(self):
        self.success = False
        self.final_pose = Pose()
        self.final_tension = 0.0
        self.execution_time = 0.0
        self.error_message = ""

class SlideAlongCableFeedback:
    def __init__(self):
        self.current_pose = Pose()
        self.current_tension = 0.0
        self.progress_percentage = 0.0
        self.distance_remaining = 0.0
        self.status_message = ""

class SlideAlongCablePrimitive(Node):
    def __init__(self):
        super().__init__('slide_along_cable_primitive')
        
        # Declare parameters
        self.declare_parameter('control_frequency', 125.0)
        self.declare_parameter('velocity_smoothing_factor', 0.9)
        self.declare_parameter('trajectory_interpolation_points', 100)
        self.declare_parameter('feedback_frequency', 10.0)
        self.declare_parameter('safety_tension_multiplier', 1.2)
        self.declare_parameter('emergency_stop_acceleration', 0.5)
        self.declare_parameter('max_execution_time', 60.0)  # seconds
        
        # Get parameters
        self.control_freq = self.get_parameter('control_frequency').as_double()
        self.velocity_smoothing = self.get_parameter('velocity_smoothing_factor').as_double()
        self.interp_points = self.get_parameter('trajectory_interpolation_points').as_int()
        self.feedback_freq = self.get_parameter('feedback_frequency').as_double()
        self.safety_multiplier = self.get_parameter('safety_tension_multiplier').as_double()
        self.emergency_decel = self.get_parameter('emergency_stop_acceleration').as_double()
        self.max_exec_time = self.get_parameter('max_execution_time').as_double()
        
        # State variables
        self.is_executing = False
        self.current_goal = None
        self.start_time = None
        self.trajectory_points = []
        self.trajectory_times = []
        self.current_trajectory_index = 0
        self.current_pose = Pose()
        self.current_tension = 0.0
        self.previous_velocity = np.zeros(6)  # [x, y, z, rx, ry, rz]
        self.emergency_stop = False
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/slide_primitive/desired_velocity', 1)
        self.tension_pub = self.create_publisher(
            Float32, '/slide_primitive/current_tension', 1)
        self.status_pub = self.create_publisher(
            String, '/slide_primitive/status', 1)
        self.feedback_pub = self.create_publisher(
            String, '/slide_primitive/feedback', 1)  # Would use custom message in real implementation
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot/current_pose', self.pose_callback, 1)
        self.tension_sub = self.create_subscription(
            Float32, '/gelsight/combined_tension', self.tension_callback, 1)
        
        # Services for primitive control
        self.execute_service = self.create_service(
            # Would use custom service type in real implementation
            # For now using basic callback structure
        )
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_freq, self.control_loop)
        
        # Feedback timer
        self.feedback_timer = self.create_timer(
            1.0 / self.feedback_freq, self.publish_feedback)
        
        self.get_logger().info("SlideAlongCable primitive initialized")
        self.get_logger().info(f"Control frequency: {self.control_freq} Hz")
        
    def start_slide_primitive(self, goal):
        """Start executing the slide along cable primitive"""
        if self.is_executing:
            return False, "Primitive already executing"
        
        # Validate goal
        validation_result = self._validate_goal(goal)
        if not validation_result[0]:
            return False, validation_result[1]
        
        # Store goal and initialize execution
        self.current_goal = goal
        self.is_executing = True
        self.start_time = self.get_clock().now()
        self.current_trajectory_index = 0
        self.emergency_stop = False
        
        # Generate trajectory
        self._generate_trajectory()
        
        self.get_logger().info("Started slide along cable primitive")
        self._publish_status("EXECUTING", "Sliding along cable")
        
        return True, "Primitive started successfully"
    
    def stop_slide_primitive(self, emergency=False):
        """Stop the primitive execution"""
        if not self.is_executing:
            return False, "No primitive executing"
        
        if emergency:
            self.emergency_stop = True
            self.get_logger().warn("Emergency stop activated")
            self._publish_status("EMERGENCY_STOP", "Emergency stop activated")
        else:
            self.is_executing = False
            self.get_logger().info("Primitive execution stopped")
            self._publish_status("STOPPED", "Execution stopped by user")
        
        # Publish zero velocity
        self._publish_zero_velocity()
        return True, "Primitive stopped"
    
    def _validate_goal(self, goal):
        """Validate the primitive goal parameters"""
        if goal.desired_tension <= 0:
            return False, "Desired tension must be positive"
        
        if goal.slide_velocity <= 0:
            return False, "Slide velocity must be positive"
        
        if goal.max_tension_limit <= goal.desired_tension:
            return False, "Max tension limit must be greater than desired tension"
        
        # Calculate trajectory distance
        start_pos = np.array([goal.start_pose.position.x, 
                            goal.start_pose.position.y, 
                            goal.start_pose.position.z])
        end_pos = np.array([goal.end_pose.position.x, 
                          goal.end_pose.position.y, 
                          goal.end_pose.position.z])
        
        distance = np.linalg.norm(end_pos - start_pos)
        if distance < 0.001:  # 1mm minimum
            return False, "Start and end positions are too close"
        
        # Estimate execution time
        estimated_time = distance / goal.slide_velocity
        if estimated_time > self.max_exec_time:
            return False, f"Estimated execution time ({estimated_time:.1f}s) exceeds maximum ({self.max_exec_time:.1f}s)"
        
        return True, "Goal validated successfully"
    
    def _generate_trajectory(self):
        """Generate smooth trajectory from start to end pose"""
        goal = self.current_goal
        
        # Extract positions
        start_pos = np.array([goal.start_pose.position.x, 
                            goal.start_pose.position.y, 
                            goal.start_pose.position.z])
        end_pos = np.array([goal.end_pose.position.x, 
                          goal.end_pose.position.y, 
                          goal.end_pose.position.z])
        
        # Extract orientations (quaternions)
        start_quat = [goal.start_pose.orientation.x, goal.start_pose.orientation.y,
                     goal.start_pose.orientation.z, goal.start_pose.orientation.w]
        end_quat = [goal.end_pose.orientation.x, goal.end_pose.orientation.y,
                   goal.end_pose.orientation.z, goal.end_pose.orientation.w]
        
        # Calculate total distance and time
        total_distance = np.linalg.norm(end_pos - start_pos)
        total_time = total_distance / goal.slide_velocity
        
        # Generate time points with smooth acceleration profile
        t = np.linspace(0, total_time, self.interp_points)
        
        # Smooth velocity profile (trapezoidal with rounded corners)
        s = self._generate_smooth_profile(t, total_time)
        
        # Interpolate positions
        pos_traj = []
        for i in range(self.interp_points):
            pos = start_pos + s[i] * (end_pos - start_pos)
            pos_traj.append(pos)
        
        # Interpolate orientations using SLERP
        start_rot = Rotation.from_quat(start_quat)
        end_rot = Rotation.from_quat(end_quat)
        
        quat_traj = []
        for i in range(self.interp_points):
            # Spherical linear interpolation
            interp_rot = start_rot.slerp(end_rot, s[i])
            quat_traj.append(interp_rot.as_quat())
        
        # Store trajectory
        self.trajectory_points = []
        self.trajectory_times = t
        
        for i in range(self.interp_points):
            pose = Pose()
            pose.position.x = pos_traj[i][0]
            pose.position.y = pos_traj[i][1] 
            pose.position.z = pos_traj[i][2]
            pose.orientation.x = quat_traj[i][0]
            pose.orientation.y = quat_traj[i][1]
            pose.orientation.z = quat_traj[i][2]
            pose.orientation.w = quat_traj[i][3]
            self.trajectory_points.append(pose)
        
        self.get_logger().info(f"Generated trajectory with {len(self.trajectory_points)} points")
        self.get_logger().info(f"Total distance: {total_distance:.3f} m, Time: {total_time:.1f} s")
    
    def _generate_smooth_profile(self, t, total_time):
        """Generate smooth S-curve velocity profile"""
        # Acceleration phase duration (20% of total time)
        accel_time = 0.2 * total_time
        decel_time = 0.2 * total_time
        const_time = total_time - accel_time - decel_time
        
        s = np.zeros_like(t)
        
        for i, time in enumerate(t):
            if time <= accel_time:
                # Smooth acceleration (S-curve)
                tau = time / accel_time
                s[i] = 0.5 * tau * tau * (3 - 2 * tau)
                s[i] *= 0.2  # Scale to reach 20% at end of accel phase
            elif time <= accel_time + const_time:
                # Constant velocity phase
                const_progress = (time - accel_time) / const_time
                s[i] = 0.2 + const_progress * 0.6  # From 20% to 80%
            else:
                # Smooth deceleration
                decel_progress = (time - accel_time - const_time) / decel_time
                tau = decel_progress
                decel_s = 0.5 * tau * tau * (3 - 2 * tau)
                s[i] = 0.8 + decel_s * 0.2  # From 80% to 100%
        
        return s
    
    def control_loop(self):
        """Main control loop executed at high frequency"""
        if not self.is_executing or self.current_goal is None:
            return
        
        # Check execution time limit
        current_time = self.get_clock().now()
        execution_time = (current_time - self.start_time).nanoseconds / 1e9
        
        if execution_time > self.max_exec_time:
            self._finish_execution(False, "Execution time limit exceeded")
            return
        
        # Check emergency stop
        if self.emergency_stop:
            self._handle_emergency_stop()
            return
        
        # Check tension safety
        if self.current_tension > self.current_goal.max_tension_limit * self.safety_multiplier:
            self.get_logger().warn(f"Tension safety limit exceeded: {self.current_tension}")
            self.emergency_stop = True
            return
        
        # Find current trajectory point
        target_pose = self._get_current_trajectory_target(execution_time)
        
        if target_pose is None:
            # Trajectory completed
            self._check_completion_criteria()
            return
        
        # Calculate desired velocity
        desired_velocity = self._calculate_desired_velocity(target_pose, execution_time)
        
        # Apply velocity smoothing
        smoothed_velocity = self._apply_velocity_smoothing(desired_velocity)
        
        # Publish velocity command
        self._publish_velocity_command(smoothed_velocity)
        
        # Update previous velocity for smoothing
        self.previous_velocity = smoothed_velocity
    
    def _get_current_trajectory_target(self, execution_time):
        """Get the target pose for current execution time"""
        if execution_time >= self.trajectory_times[-1]:
            return None  # Trajectory completed
        
        # Find the trajectory segment
        for i in range(len(self.trajectory_times) - 1):
            if execution_time <= self.trajectory_times[i + 1]:
                # Interpolate between points i and i+1
                t0, t1 = self.trajectory_times[i], self.trajectory_times[i + 1]
                alpha = (execution_time - t0) / (t1 - t0)
                
                pose0 = self.trajectory_points[i]
                pose1 = self.trajectory_points[i + 1]
                
                # Linear interpolation for position
                target_pose = Pose()
                target_pose.position.x = pose0.position.x + alpha * (pose1.position.x - pose0.position.x)
                target_pose.position.y = pose0.position.y + alpha * (pose1.position.y - pose0.position.y)
                target_pose.position.z = pose0.position.z + alpha * (pose1.position.z - pose0.position.z)
                
                # SLERP for orientation
                quat0 = [pose0.orientation.x, pose0.orientation.y, pose0.orientation.z, pose0.orientation.w]
                quat1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
                
                rot0 = Rotation.from_quat(quat0)
                rot1 = Rotation.from_quat(quat1)
                interp_rot = rot0.slerp(rot1, alpha)
                interp_quat = interp_rot.as_quat()
                
                target_pose.orientation.x = interp_quat[0]
                target_pose.orientation.y = interp_quat[1]
                target_pose.orientation.z = interp_quat[2]
                target_pose.orientation.w = interp_quat[3]
                
                return target_pose
        
        return self.trajectory_points[-1]  # Last point
    
    def _calculate_desired_velocity(self, target_pose, execution_time):
        """Calculate desired velocity towards target pose"""
        # Position error
        pos_error = np.array([
            target_pose.position.x - self.current_pose.position.x,
            target_pose.position.y - self.current_pose.position.y,
            target_pose.position.z - self.current_pose.position.z
        ])
        
        # Orientation error (simplified)
        current_quat = [self.current_pose.orientation.x, self.current_pose.orientation.y,
                       self.current_pose.orientation.z, self.current_pose.orientation.w]
        target_quat = [target_pose.orientation.x, target_pose.orientation.y,
                      target_pose.orientation.z, target_pose.orientation.w]
        
        current_rot = Rotation.from_quat(current_quat)
        target_rot = Rotation.from_quat(target_quat)
        
        # Rotation error as axis-angle
        error_rot = target_rot * current_rot.inv()
        rot_error = error_rot.as_rotvec()
        
        # Proportional controller gains
        kp_pos = 2.0  # Position gain
        kp_rot = 1.0  # Rotation gain
        
        # Calculate desired velocity
        desired_velocity = np.zeros(6)
        desired_velocity[0:3] = kp_pos * pos_error  # Linear velocity
        desired_velocity[3:6] = kp_rot * rot_error  # Angular velocity
        
        # Apply velocity limits based on trajectory
        max_linear = self.current_goal.slide_velocity * 1.2  # 20% margin
        max_angular = 0.5  # rad/s
        
        # Limit linear velocity
        linear_norm = np.linalg.norm(desired_velocity[0:3])
        if linear_norm > max_linear:
            desired_velocity[0:3] *= max_linear / linear_norm
        
        # Limit angular velocity
        angular_norm = np.linalg.norm(desired_velocity[3:6])
        if angular_norm > max_angular:
            desired_velocity[3:6] *= max_angular / angular_norm
        
        return desired_velocity
    
    def _apply_velocity_smoothing(self, desired_velocity):
        """Apply velocity smoothing to avoid jerky motion"""
        # Exponential smoothing
        smoothed = self.velocity_smoothing * self.previous_velocity + \
                  (1 - self.velocity_smoothing) * desired_velocity
        
        # Apply acceleration limits
        dt = 1.0 / self.control_freq
        max_accel = np.array([0.5, 0.5, 0.5, 1.0, 1.0, 1.0])  # [m/s², rad/s²]
        
        velocity_change = smoothed - self.previous_velocity
        acceleration = velocity_change / dt
        
        # Limit acceleration
        for i in range(6):
            if abs(acceleration[i]) > max_accel[i]:
                acceleration[i] = np.sign(acceleration[i]) * max_accel[i]
                smoothed[i] = self.previous_velocity[i] + acceleration[i] * dt
        
        return smoothed
    
    def _publish_velocity_command(self, velocity):
        """Publish velocity command"""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        
        twist_msg.twist.linear.x = velocity[0]
        twist_msg.twist.linear.y = velocity[1]
        twist_msg.twist.linear.z = velocity[2]
        twist_msg.twist.angular.x = velocity[3]
        twist_msg.twist.angular.y = velocity[4]
        twist_msg.twist.angular.z = velocity[5]
        
        self.velocity_pub.publish(twist_msg)
    
    def _publish_zero_velocity(self):
        """Publish zero velocity command"""
        self._publish_velocity_command(np.zeros(6))
    
    def _check_completion_criteria(self):
        """Check if primitive completion criteria are met"""
        goal = self.current_goal
        
        # Check position criteria
        current_pos = np.array([self.current_pose.position.x, 
                              self.current_pose.position.y, 
                              self.current_pose.position.z])
        target_pos = np.array([goal.end_pose.position.x, 
                             goal.end_pose.position.y, 
                             goal.end_pose.position.z])
        
        position_error = np.linalg.norm(current_pos - target_pos)
        position_reached = position_error <= goal.position_tolerance
        
        # Check tension criteria
        tension_error = abs(self.current_tension - goal.desired_tension)
        tension_reached = tension_error <= goal.tension_tolerance
        
        # Determine completion based on stop criterion
        if goal.stop_criterion == "position":
            completed = position_reached
        elif goal.stop_criterion == "tension":
            completed = tension_reached
        elif goal.stop_criterion == "both":
            completed = position_reached and tension_reached
        else:
            completed = position_reached  # Default
        
        if completed:
            self._finish_execution(True, "Primitive completed successfully")
        else:
            # Continue execution if trajectory not finished
            pass
    
    def _finish_execution(self, success, message):
        """Finish primitive execution"""
        self.is_executing = False
        self._publish_zero_velocity()
        
        # Calculate final results
        execution_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        status = "COMPLETED" if success else "FAILED"
        self._publish_status(status, message)
        
        self.get_logger().info(f"Primitive finished: {message}")
        self.get_logger().info(f"Execution time: {execution_time:.2f} s")
        self.get_logger().info(f"Final tension: {self.current_tension:.2f} N")
    
    def _handle_emergency_stop(self):
        """Handle emergency stop with controlled deceleration"""
        # Apply controlled deceleration
        decel_velocity = self.previous_velocity.copy()
        dt = 1.0 / self.control_freq
        
        # Reduce velocity gradually
        for i in range(6):
            if abs(decel_velocity[i]) > 0.001:
                decel_sign = -np.sign(decel_velocity[i])
                decel_amount = self.emergency_decel * dt
                
                if abs(decel_velocity[i]) > decel_amount:
                    decel_velocity[i] += decel_sign * decel_amount
                else:
                    decel_velocity[i] = 0.0
        
        self._publish_velocity_command(decel_velocity)
        self.previous_velocity = decel_velocity
        
        # Check if stopped
        if np.linalg.norm(decel_velocity) < 0.001:
            self._finish_execution(False, "Emergency stop completed")
    
    def publish_feedback(self):
        """Publish feedback at lower frequency"""
        if not self.is_executing:
            return
        
        # Calculate progress
        if len(self.trajectory_points) > 0:
            # Find closest trajectory point to current pose
            current_pos = np.array([self.current_pose.position.x, 
                                  self.current_pose.position.y, 
                                  self.current_pose.position.z])
            
            min_distance = float('inf')
            closest_index = 0
            
            for i, point in enumerate(self.trajectory_points):
                point_pos = np.array([point.position.x, point.position.y, point.position.z])
                distance = np.linalg.norm(current_pos - point_pos)
                if distance < min_distance:
                    min_distance = distance
                    closest_index = i
            
            progress = (closest_index / len(self.trajectory_points)) * 100.0
            
            # Calculate remaining distance
            end_pos = np.array([self.current_goal.end_pose.position.x,
                              self.current_goal.end_pose.position.y,
                              self.current_goal.end_pose.position.z])
            distance_remaining = np.linalg.norm(current_pos - end_pos)
            
            # Publish feedback (would use custom message in real implementation)
            feedback_str = f"Progress: {progress:.1f}%, Remaining: {distance_remaining:.3f}m, Tension: {self.current_tension:.2f}N"
            feedback_msg = String()
            feedback_msg.data = feedback_str
            self.feedback_pub.publish(feedback_msg)
        
        # Publish current tension
        tension_msg = Float32()
        tension_msg.data = self.current_tension
        self.tension_pub.publish(tension_msg)
    
    def _publish_status(self, status, message):
        """Publish primitive status"""
        status_msg = String()
        status_msg.data = f"{status}: {message}"
        self.status_pub.publish(status_msg)
    
    def pose_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg.pose
    
    def tension_callback(self, msg):
        """Update current cable tension"""
        self.current_tension = msg.data


def main(args=None):
    rclpy.init(args=args)
    
    primitive = SlideAlongCablePrimitive()
    
    executor = MultiThreadedExecutor()
    executor.add_node(primitive)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        primitive.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()