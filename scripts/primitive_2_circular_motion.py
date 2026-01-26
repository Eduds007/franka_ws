#!/usr/bin/env python3
"""
Primitive 2: Circular Motion with Cable Detection

This script performs the following sequence:
1. Verify cable is detected (from Primitive 1)
2. Start circular motion in joint space
3. Continuously monitor cable detection
4. Stop immediately when cable is lost
5. Return to safe position

Usage:
    ./primitive_2_circular_motion.py
    
Returns:
    Exit code 0: Motion completed successfully (cable lost as expected)
    Exit code 1: Cable not detected at start
    Exit code 2: Error during execution
"""

import sys
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray


# Base position for circular motion (similar to home but adjusted)
BASE_JOINT_POSITIONS = [
    0.0,      # panda_joint1
    -0.785,   # panda_joint2 (-45 degrees)
    0.0,      # panda_joint3
    -2.356,   # panda_joint4 (-135 degrees)
    0.0,      # panda_joint5
    1.571,    # panda_joint6 (90 degrees)
    0.785     # panda_joint7 (45 degrees)
]

# Circular motion parameters
CIRCLE_RADIUS = 0.32      # Radius in joint space variation (radians) - REDUZIDO para segurança
CIRCLE_PERIOD = 4.0      # Period for one complete circle (seconds) - AUMENTADO para movimento mais lento
PUBLISH_RATE = 1000.0       # Hz - command publishing rate - REDUZIDO para evitar communication_constraints_violation
CHECK_RATE = 100.0         # Hz - cable detection check rate
RAMP_UP_TIME = 3.0        # Seconds to gradually increase to full radius

# Cable detection parameters
CABLE_LOST_THRESHOLD = 5  # Number of consecutive "no cable" readings to stop


class Primitive2Node(Node):
    """Node for Primitive 2: Circular Motion with Cable Monitoring"""
    
    def __init__(self):
        super().__init__('primitive_2_circular_motion')
        
        # Cable detection state
        self.cable_status = None
        self.cable_status_received = False
        self.consecutive_no_cable = 0
        
        # Motion state
        self.motion_active = False
        self.start_time = None
        
        # Publisher for joint commands (admittance controller)
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray,
            '/admittance_example_controller/joint_command',
            10
        )
        
        # Subscriber for cable detection
        self.cable_sub = self.create_subscription(
            Int32,
            '/cable_status',
            self.cable_callback,
            10
        )
        
        self.get_logger().info("🔄 Primitive 2: Circular Motion initialized")
    
    def cable_callback(self, msg):
        """Callback for cable detection status"""
        self.cable_status = msg.data
        self.cable_status_received = True
        
        # Track consecutive "no cable" readings
        if msg.data == 0:
            self.consecutive_no_cable += 1
        else:
            self.consecutive_no_cable = 0
    
    def wait_for_cable_detection(self, timeout_sec=5.0):
        """
        Wait for cable detection status
        
        Returns:
            True if cable detected, False if no cable, None if timeout
        """
        self.get_logger().info("⏳ Aguardando status do cabo...")
        
        start_time = time.time()
        stable_count = 0
        required_stable = 3  # Need 3 consecutive "cable present" readings
        
        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            
            if self.cable_status_received:
                if self.cable_status == 1:
                    stable_count += 1
                    if stable_count >= required_stable:
                        self.get_logger().info(f"✅ Cabo detectado ({stable_count} leituras)")
                        return True
                else:
                    stable_count = 0
            
            time.sleep(0.05)
        
        if self.cable_status_received and self.cable_status == 0:
            self.get_logger().warning("❌ Cabo NÃO detectado")
            return False
        
        self.get_logger().warning("⚠️  Timeout aguardando detecção do cabo")
        return None
    
    def publish_joint_command(self, joint_positions):
        """Publish joint position command"""
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.joint_command_pub.publish(msg)
    
    def move_to_base_position(self, duration_sec=3.0):
        """
        Move to base position for circular motion
        
        Args:
            duration_sec: Time to reach position (gradual movement)
        """
        self.get_logger().info("📍 Movendo para posição base...")
        
        # Publish base position multiple times for smooth transition
        rate = 10  # 10 Hz
        iterations = int(duration_sec * rate)
        
        for i in range(iterations):
            self.publish_joint_command(BASE_JOINT_POSITIONS)
            time.sleep(1.0 / rate)
            rclpy.spin_once(self, timeout_sec=0.0)
        
        self.get_logger().info("✅ Posição base alcançada")
    
    def execute_circular_motion(self):
        """
        Execute circular motion while monitoring cable detection
        Uses gradual ramp-up to avoid communication_constraints_violation
        
        Returns:
            True if motion stopped due to cable loss (success)
            False if error occurred
        """
        self.get_logger().info("🔄 Iniciando movimento circular...")
        self.get_logger().info(f"   Raio: {CIRCLE_RADIUS:.3f} rad")
        self.get_logger().info(f"   Período: {CIRCLE_PERIOD:.1f} s")
        self.get_logger().info(f"   Taxa de publicação: {PUBLISH_RATE:.0f} Hz")
        self.get_logger().info(f"   Tempo de ramp-up: {RAMP_UP_TIME:.1f} s")
        
        # Reset cable detection counter
        self.consecutive_no_cable = 0
        
        # Start motion
        self.motion_active = True
        self.start_time = time.time()
        
        omega = 2.0 * math.pi / CIRCLE_PERIOD
        dt_publish = 1.0 / PUBLISH_RATE
        dt_check = 1.0 / CHECK_RATE
        
        last_check_time = 0.0
        last_log_time = 0.0
        log_interval = 2.0  # Log every 2 seconds
        
        # Store last command to ensure smooth transitions
        last_joint_positions = BASE_JOINT_POSITIONS.copy()
        
        iteration = 0
        
        try:
            while self.motion_active:
                current_time = time.time()
                elapsed = current_time - self.start_time
                
                # Calculate circular trajectory
                theta = omega * elapsed
                
                # Gradual ramp-up: scale radius from 0 to full over RAMP_UP_TIME
                if elapsed < RAMP_UP_TIME:
                    radius_scale = elapsed / RAMP_UP_TIME  # 0 to 1
                    # Use smooth sine curve for even gentler ramp
                    radius_scale = math.sin(radius_scale * math.pi / 2)
                else:
                    radius_scale = 1.0
                
                current_radius = CIRCLE_RADIUS * radius_scale
                
                # Modify joints 2 and 4 to create circular-like motion
                # This is a simplified approach - ideally use inverse kinematics
                joint_positions = BASE_JOINT_POSITIONS.copy()
                joint_positions[1] += current_radius * math.cos(theta)  # Joint 2
                joint_positions[3] += current_radius * math.sin(theta)  # Joint 4
                
                # Smooth transition: limit maximum change per iteration
                MAX_JOINT_CHANGE = 0.01  # radians per iteration (safety limit)
                for i in range(len(joint_positions)):
                    delta = joint_positions[i] - last_joint_positions[i]
                    if abs(delta) > MAX_JOINT_CHANGE:
                        joint_positions[i] = last_joint_positions[i] + math.copysign(MAX_JOINT_CHANGE, delta)
                
                # Publish command
                self.publish_joint_command(joint_positions)
                last_joint_positions = joint_positions.copy()
                
                # Check cable status periodically
                if current_time - last_check_time >= dt_check:
                    rclpy.spin_once(self, timeout_sec=0.0)
                    last_check_time = current_time
                    
                    # Check if cable is lost
                    if self.consecutive_no_cable >= CABLE_LOST_THRESHOLD:
                        self.get_logger().info(f"🛑 Cabo perdido! ({self.consecutive_no_cable} leituras consecutivas)")
                        self.get_logger().info(f"   Tempo de movimento: {elapsed:.2f}s")
                        self.motion_active = False
                        return True
                
                # Periodic logging
                if current_time - last_log_time >= log_interval:
                    circles_completed = elapsed / CIRCLE_PERIOD
                    ramp_status = f"(ramp: {radius_scale*100:.0f}%)" if elapsed < RAMP_UP_TIME else ""
                    self.get_logger().info(
                        f"🔄 Movimento ativo: {elapsed:.1f}s {ramp_status} | "
                        f"Círculos: {circles_completed:.2f} | "
                        f"Cabo: {'✅ OK' if self.consecutive_no_cable == 0 else f'⚠️  {self.consecutive_no_cable}'}"
                    )
                    last_log_time = current_time
                
                # Sleep to maintain publishing rate
                time.sleep(dt_publish)
                iteration += 1
        
        except KeyboardInterrupt:
            self.get_logger().info("🛑 Movimento interrompido pelo usuário")
            self.motion_active = False
            return False
        
        return True
    
    def stop_motion(self):
        """Stop motion and hold current position"""
        self.get_logger().info("🛑 Parando movimento...")
        self.motion_active = False
        
        # Hold current position by publishing it multiple times
        for _ in range(10):
            self.publish_joint_command(BASE_JOINT_POSITIONS)
            time.sleep(0.1)
        
        self.get_logger().info("✅ Movimento parado")


def main():
    """Main function for Primitive 2"""
    rclpy.init()
    
    print("\n" + "="*70)
    print("🔄 PRIMITIVE 2: CIRCULAR MOTION WITH CABLE MONITORING")
    print("="*70)
    
    node = Primitive2Node()
    exit_code = 2  # Default: error
    
    try:
        # Step 1: Verify cable is detected
        print("\n🔍 Step 1/4: Verificando detecção do cabo...")
        cable_detected = node.wait_for_cable_detection(timeout_sec=5.0)
        
        if cable_detected is None:
            print("\n❌ ERROR: Timeout aguardando status do cabo")
            print("⚠️  Certifique-se que o cable_detector está rodando:")
            print("   ros2 run tactile_cameras cable_detector.py")
            exit_code = 2
            return
        
        if not cable_detected:
            print("\n❌ ERROR: Cabo não detectado!")
            print("⚠️  Execute primeiro a Primitiva 1 para detectar o cabo")
            exit_code = 1
            return
        
        print("✅ Cabo detectado! Pronto para movimento circular")
        
        # Step 2: Move to base position
        print("\n📍 Step 2/4: Movendo para posição base...")
        node.move_to_base_position(duration_sec=3.0)
        
        # Wait for stability
        print("   Aguardando estabilização (1s)...")
        time.sleep(1.0)
        
        # Step 3: Execute circular motion
        print("\n🔄 Step 3/4: Executando movimento circular...")
        print("⚠️  O movimento continuará até o cabo ser perdido")
        print("   Pressione Ctrl+C para parar manualmente\n")
        
        success = node.execute_circular_motion()
        
        if success:
            print("\n✅ Movimento circular completado (cabo perdido)")
        else:
            print("\n⚠️  Movimento interrompido")
        
        # Step 4: Stop and stabilize
        print("\n🛑 Step 4/4: Parando e estabilizando...")
        node.stop_motion()
        
        if success:
            print("\n" + "="*70)
            print("✅ SUCCESS: PRIMITIVE 2 COMPLETED")
            print("="*70)
            exit_code = 0
        else:
            exit_code = 2
    
    except KeyboardInterrupt:
        print("\n\n🛑 Interrompido pelo usuário")
        node.stop_motion()
        exit_code = 2
    
    except Exception as e:
        print(f"\n❌ ERROR: {str(e)}")
        node.get_logger().error(f"Exception: {str(e)}")
        import traceback
        traceback.print_exc()
        exit_code = 2
    
    finally:
        # Cleanup
        print("\n🧹 Limpando...")
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Primitive 2 complete\n")
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
