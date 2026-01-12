#!/usr/bin/env python3
# filepath: /home/nuc_6g_life_3/franka_ws/src/franka_moveit_example/franka_moveit_example/moveit_real_robot.py
"""
Exemplo de controle do robô Franka Panda REAL usando MoveIt2.
Este script se conecta ao robô físico no IP 172.16.0.3 e permite
planejar e executar movimentos usando o MoveIt2 e visualizar no RViz.
"""

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
import time
from threading import Thread

def main(args=None):
    rclpy.init(args=args)

    # Create a node for this example
    node = Node("moveit2_real_robot")
    
    # Spin the node in the background
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Create a MoveIt2 object
    moveit2 = MoveIt2(
        node=node,
        joint_names=panda.joint_names(),
        base_link_name=panda.base_link_name(),
        end_effector_name=panda.end_effector_name(),
        group_name="panda_arm",
    )

    # Wait for initialization
    node.get_logger().info("="*70)
    node.get_logger().info("EXEMPLO MOVEIT2 COM FRANKA PANDA - ROBÔ REAL (172.16.0.3)")
    node.get_logger().info("="*70)
    node.get_logger().info("Aguardando inicialização do MoveIt2...")
    time.sleep(3.0)
    
    node.get_logger().info("Sistema pronto! Iniciando sequência de movimentos...")
    node.get_logger().info("")
    
    try:
        # Movimento 1: Posição HOME
        node.get_logger().info("🏠 MOVIMENTO 1: Indo para posição HOME")
        joint_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        node.get_logger().info(f"   Posições das juntas: {joint_positions}")
        
        moveit2.move_to_configuration(joint_positions)
        moveit2.wait_until_executed()
        
        node.get_logger().info("   ✓ Movimento 1 concluído!")
        node.get_logger().info("")
        time.sleep(2.0)

        # Movimento 2: Posição Cartesiana 1
        node.get_logger().info("📍 MOVIMENTO 2: Posição Cartesiana")
        position = [0.3, 0.0, 0.6]
        orientation = [1.0, 0.0, 0.0, 0.0]  # quaternion [x, y, z, w]
        node.get_logger().info(f"   Posição XYZ: {position}")
        node.get_logger().info(f"   Orientação (quat): {orientation}")
        
        moveit2.move_to_pose(position=position, quat_xyzw=orientation, cartesian=False)
        moveit2.wait_until_executed()
        
        node.get_logger().info("   ✓ Movimento 2 concluído!")
        node.get_logger().info("")
        time.sleep(2.0)
        
        
        # Retornar à posição HOME
        node.get_logger().info("🏠 MOVIMENTO 4: Retornando à posição HOME")
        moveit2.move_to_configuration(joint_positions)
        moveit2.wait_until_executed()
        
        node.get_logger().info("   ✓ Movimento 4 concluído!")
        node.get_logger().info("")
        
    except KeyboardInterrupt:
        node.get_logger().info("⚠️  Interrompido pelo usuário!")
    except Exception as e:
        node.get_logger().error(f"❌ Erro durante execução: {str(e)}")
    
    node.get_logger().info("="*70)
    node.get_logger().info("SEQUÊNCIA DE MOVIMENTOS CONCLUÍDA!")
    node.get_logger().info("="*70)
    
    try:
        rclpy.shutdown()
    except:
        pass
    
    try:
        executor_thread.join(timeout=1.0)
    except:
        pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⚠️  Programa interrompido pelo usuário!")
    except Exception as e:
        print(f"\n❌ Erro: {e}")
