#!/usr/bin/env python3
"""
Primitive 1: Homing + Gripper Close + Cable Detection

This script performs the following sequence:
1. Move robot to homing position using joint trajectory controller
2. Close gripper using gripper action
3. Wait and detect if cable is present
4. Return detection result

Usage:
    ./primitive_1_detect_cable.py
    
Returns:
    Exit code 0: Cable detected
    Exit code 1: No cable detected
    Exit code 2: Error during execution
"""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# Homing position (default safe configuration for Panda)
# NOTE: Slow movement (10 seconds) to avoid communication_constraints_violation
HOMING_JOINT_POSITIONS = [
    0.0,      # panda_joint1
    -0.785,   # panda_joint2 (-45 degrees)
    0.0,      # panda_joint3
    -2.356,   # panda_joint4 (-135 degrees)
    0.0,      # panda_joint5
    1.571,    # panda_joint6 (90 degrees)
    0.785     # panda_joint7 (45 degrees)
]

# Joint names for Panda robot
JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7'
]


class Primitive1Node(Node):
    """Node for Primitive 1: Homing + Gripper Close + Cable Detection"""
    
    def __init__(self):
        super().__init__('primitive_1_detect_cable')
        
        # Cable detection state
        self.cable_status = None  # None = unknown, 0 = no cable, 1 = cable
        self.cable_status_received = False
        self.cable_status_timestamp = 0.0  # Track when last message was received
        
        # Action clients
        self.joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/panda_gripper/gripper_action'
        )
        
        # Subscriber for cable detection
        self.cable_sub = self.create_subscription(
            Int32,
            '/cable_status',
            self.cable_callback,
            10
        )
        
        self.get_logger().info("🔌 Primitive 1: Cable Detection initialized")
    
    def cable_callback(self, msg):
        """Callback for cable detection status"""
        self.cable_status = msg.data
        self.cable_status_timestamp = time.time()
        self.cable_status_received = True
        # Debug print removido para não poluir o log
    
    def wait_for_servers(self, timeout_sec=10.0):
        """Wait for action servers to be available"""
        self.get_logger().info("⏳ Waiting for action servers...")
        
        if not self.joint_trajectory_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("❌ Joint trajectory action server not available!")
            return False
        
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("❌ Gripper action server not available!")
            return False
        
        self.get_logger().info("✅ All action servers available")
        return True
    
    def move_to_joint_positions(self, joint_positions, duration_sec=10.0):
        """
        Move robot to specified joint positions
        
        Args:
            joint_positions: List of 7 joint positions in radians
            duration_sec: Time to reach the target position
            
        Returns:
            True if successful, False otherwise
        """
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES
        
        # Create trajectory point with velocities and accelerations
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * 7  # Stop at target
        point.accelerations = [0.0] * 7  # Zero acceleration at target
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory
        
        # Set goal tolerances for safety
        goal_msg.goal_time_tolerance = Duration(sec=1, nanosec=0)
        
        self.get_logger().info(f"📤 Sending joint trajectory goal...")
        
        # Send goal
        send_goal_future = self.joint_trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Joint trajectory goal rejected!")
            return False
        
        self.get_logger().info("✅ Goal accepted, waiting for completion...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("✅ Joint trajectory completed successfully")
            return True
        else:
            self.get_logger().error(f"❌ Joint trajectory failed with status: {result.status}")
            return False
    
    def close_gripper(self, force=20.0):
        """
        Close the gripper
        
        Args:
            force: Maximum effort in Newtons (default 20N)
            
        Returns:
            True if successful, False otherwise
        """
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.0  # Fully closed
        goal_msg.command.max_effort = force
        
        self.get_logger().info(f"📤 Sending gripper close command (force={force}N)...")
        
        # Send goal
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Gripper command rejected!")
            return False
        
        self.get_logger().info("✅ Gripper command accepted, waiting for completion...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"✅ Gripper closed (position={result.result.position:.4f}m, effort={result.result.effort:.2f}N)")
            return True
        else:
            self.get_logger().error(f"❌ Gripper command failed with status: {result.status}")
            return False
    
    def wait_for_cable_detection(self, timeout_sec=5.0, stable_time_sec=1.0):
        """
        Wait for stable cable detection result using consecutive-samples rule.
        Consider cable present if 5 consecutive readings == 1.
        
        Args:
            timeout_sec: Maximum time to wait for detection
            stable_time_sec: (ignored) kept for API compatibility
            
        Returns:
            True if cable detected, False if no cable, None if timeout
        """
        # IMPORTANTE: Limpar estado antigo e aguardar dados NOVOS
        self.get_logger().info("🔄 Limpando dados antigos do sensor...")
        self.cable_status_received = False
        self.cable_status = None
        
        # Aguardar primeira mensagem NOVA (dados frescos após gripper fechar)
        self.get_logger().info("⏳ Aguardando primeira leitura fresca do sensor...")
        start_wait = time.time()
        
        while not self.cable_status_received and (time.time() - start_wait) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)  # Processar callbacks
            time.sleep(0.01)  # Small sleep to avoid busy-wait
        
        if not self.cable_status_received:
            self.get_logger().error("❌ Nenhum dado recebido do sensor!")
            return None
        
        self.get_logger().info(f"✅ Primeira leitura recebida: {self.cable_status}")
        
        # Agora sim, coletar amostras e decidir
        required_consecutive = 5
        self.get_logger().info(f"📊 Coletando {required_consecutive} leituras consecutivas para decisão...")
        
        start_time = time.time()
        consec_ones = 0
        consec_zeros = 0
        
        while time.time() - start_time < timeout_sec:
            # Processar callbacks para atualizar self.cable_status
            rclpy.spin_once(self, timeout_sec=0.05)
            
            if self.cable_status_received:
                status = self.cable_status
                
                # Debug: mostrar valor atual
                self.get_logger().info(f"📈 Leitura: {status}")
                
                if status == 1:
                    consec_ones += 1
                    consec_zeros = 0
                    if consec_ones >= required_consecutive:
                        self.get_logger().info(f"✅ Detecção estável: TEM CABO ({consec_ones} leituras consecutivas de 1)")
                        return True
                elif status == 0:
                    consec_zeros += 1
                    consec_ones = 0
                    if consec_zeros >= required_consecutive:
                        self.get_logger().info(f"✅ Detecção estável: NÃO TEM CABO ({consec_zeros} leituras consecutivas de 0)")
                        return False
            
            time.sleep(0.05)  # 20 Hz
        
        self.get_logger().warning("⚠️  Timeout na detecção de cabo")
        return None

def main():
    """Main function for Primitive 1"""
    rclpy.init()
    
    print("\n" + "="*70)
    print("🤖 PRIMITIVE 1: HOMING + GRIPPER CLOSE + CABLE DETECTION")
    print("="*70)
    
    # Create node
    node = Primitive1Node()
    
    exit_code = 2  # Default: error
    
    try:
        # Wait for action servers
        if not node.wait_for_servers(timeout_sec=10.0):
            print("\n❌ ERROR: Action servers not available")
            print("⚠️  Make sure the robot controllers are running:")
            print("   - Joint trajectory controller: /joint_trajectory_controller/follow_joint_trajectory")
            print("   - Gripper action server: /panda_gripper/gripper_action")
            exit_code = 2
            return
        
        # Step 1: Move to homing position
        print("\n📍 Step 1/3: Moving to homing position...")
        print(f"   Target joints: {HOMING_JOINT_POSITIONS}")
        print(f"   ⚠️  Using slow and safe trajectory (10 seconds)")
        
        if not node.move_to_joint_positions(HOMING_JOINT_POSITIONS, duration_sec=10.0):
            print("❌ ERROR: Failed to reach homing position")
            exit_code = 2
            return
        
        print("✅ Reached homing position")
        
        # Wait a bit for stability
        print("   Waiting for stability (0.5s)...")
        time.sleep(0.5)
        
        # Step 2: Close gripper
        print("\n🤏 Step 2/3: Closing gripper...")
        
        if not node.close_gripper(force=20.0):
            print("❌ ERROR: Failed to close gripper")
            exit_code = 2
            return
        
        print("✅ Gripper closed")
        
        # Wait for gripper to settle and sensors to stabilize
        print("\n⏳ Waiting for sensors to stabilize (2 seconds)...")
        time.sleep(2.0)
        
        # Step 3: Detect cable
        print("\n🔍 Step 3/3: Detecting cable...")
        cable_detected = node.wait_for_cable_detection(timeout_sec=5.0, stable_time_sec=1.0)

        if cable_detected is None:
            print("\n❌ ERROR: Cable detection timeout")
            print("⚠️  Make sure the cable_detector node is running:")
            print("   ros2 run tactile_cameras cable_detector.py")
            exit_code = 2
        elif cable_detected:
            print("\n" + "="*70)
            print("✅ SUCCESS: CABLE DETECTED")
            print("="*70)
            exit_code = 0
        else:
            print("\n" + "="*70)
            print("❌ RESULT: NO CABLE DETECTED")
            print("="*70)
            exit_code = 1
            
    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted by user")
        exit_code = 2
    except Exception as e:
        print(f"\n❌ ERROR: {str(e)}")
        node.get_logger().error(f"Exception: {str(e)}")
        import traceback
        traceback.print_exc()
        exit_code = 2
    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Primitive 1 complete\n")
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
