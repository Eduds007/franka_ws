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
from std_msgs.msg import Float32MultiArray
import time
import math
from threading import Thread

# Variável global para controlar pausa temporária do robô
pause_movement = False
last_force_values = []


def force_callback(msg, node):
    """
    Callback para monitorar os valores de força do tópico /feats/cam1/total_force.
    Sinaliza para pausar próximo movimento se algum valor for menor que -1.0.
    """
    global pause_movement, last_force_values
    
    last_force_values = list(msg.data)
    
    # Verificar se algum valor é menor que -1.0 (igual ao total_force_listener)
    valores_excessivos = [v for v in msg.data if v < -1.0]
    
    if valores_excessivos:
        if not pause_movement:
            node.get_logger().warn(f"🚨 TOQUE DETECTADO!")
            node.get_logger().warn(f"   ⚠️  Valores menores que -1.0: {valores_excessivos}")
            node.get_logger().warn(f"   Todos os valores: {[f'{v:.4f}' for v in msg.data]}")
            pause_movement = True
    else:
        # Valores normalizaram, pode continuar
        if pause_movement:
            pause_movement = False

def draw_circle(moveit2, node, center_x, center_y, center_z, radius, num_points=16, orientation=None):
    """
    Desenha um círculo no plano ZY com o end effector.
    X é mantido fixo enquanto Y e Z variam para criar o círculo vertical.
    
    Args:
        moveit2: Objeto MoveIt2
        node: Nó ROS2
        center_x: Coordenada X fixa do círculo
        center_y, center_z: Coordenadas do centro do círculo no plano YZ
        radius: Raio do círculo em metros
        num_points: Número de pontos no círculo (mais pontos = movimento mais suave)
        orientation: Orientação do end effector [x, y, z, w] (quaternion)
    """
    if orientation is None:
        orientation = [1.0, 0.0, 0.0, 0.0]  # Orientação padrão
    
    global pause_movement, last_force_values
    
    node.get_logger().info(f"⭕ Desenhando círculo no plano ZY:")
    node.get_logger().info(f"   X fixo: {center_x:.3f} m")
    node.get_logger().info(f"   Centro YZ: [{center_y:.3f}, {center_z:.3f}]")
    node.get_logger().info(f"   Raio: {radius:.3f} m")
    node.get_logger().info(f"   Pontos: {num_points}")
    
    for i in range(num_points + 1):  # +1 para fechar o círculo
        angle = 2 * math.pi * i / num_points
        
        # Calcular posição no círculo (plano ZY - vertical)
        # X permanece fixo
        # Y e Z variam para formar o círculo
        x = center_x  # X fixo
        y = center_y + radius * math.cos(angle)  # Y varia em círculo
        z = center_z + radius * math.sin(angle)  # Z varia em círculo
        
        node.get_logger().info(f"   → Ponto {i+1}/{num_points+1}: [{x:.3f}, {y:.3f}, {z:.3f}]")
        
        # Continua enviando o mesmo ponto enquanto houver força excessiva
        movement_successful = False
        retry_count = 0
        
        while not movement_successful:
            try:
                # Verificar se há força excessiva
                if pause_movement:
                    retry_count += 1
                    if retry_count == 1:  # Só loga na primeira vez
                        node.get_logger().warn(f"   ⚠️  Força excessiva detectada! Mantendo posição atual...")
                    
                    # Envia o MESMO ponto novamente
                    moveit2.move_to_pose(position=[x, y, z], quat_xyzw=orientation, cartesian=True)
                    moveit2.wait_until_executed()
                    
                    # Aguarda um pouco antes de tentar novamente
                    time.sleep(0.1)
                else:
                    # Sem força excessiva, move normalmente
                    if retry_count > 0:
                        node.get_logger().info(f"   ✅ Força normalizada após {retry_count} tentativas. Continuando...")
                    
                    moveit2.move_to_pose(position=[x, y, z], quat_xyzw=orientation, cartesian=True)
                    moveit2.wait_until_executed()
                    movement_successful = True
                    
            except Exception as e:
                node.get_logger().error(f"   ❌ Erro no movimento: {e}")
                node.get_logger().warn(f"   ⚠️  Aguardando 2s antes de continuar...")
                time.sleep(2.0)
                # Em caso de erro, tenta continuar
                movement_successful = True

    
    node.get_logger().info("   ✓ Círculo completo!")


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

    # Criar subscriber para monitorar força do tópico /feats/cam1/total_force
    force_subscriber = node.create_subscription(
        Float32MultiArray,
        '/feats/cam1/total_force',
        lambda msg: force_callback(msg, node),
        10
    )
    
    node.get_logger().info("📡 Subscriber criado para /feats/cam1/total_force")
    node.get_logger().info("   Monitorando valores de força (limiar: valor < -1.0)")
    node.get_logger().info("   Modo: PAUSA TEMPORÁRIA (movimento continua após normalização)")
    node.get_logger().info("")

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
        
        try:
            moveit2.move_to_configuration(joint_positions)
            moveit2.wait_until_executed()
            node.get_logger().info("   ✓ Movimento 1 concluído!")
        except Exception as e:
            node.get_logger().error(f"   ❌ Erro no movimento HOME: {e}")
        
        node.get_logger().info("")
        time.sleep(2.0)

        # Movimento 2: Posição Cartesiana 1
        node.get_logger().info("📍 MOVIMENTO 2: Posição Cartesiana")
        position = [0.3, 0.0, 0.6]
        orientation = [1.0, 0.0, 0.0, 0.0]  # quaternion [x, y, z, w]
        node.get_logger().info(f"   Posição XYZ: {position}")
        node.get_logger().info(f"   Orientação (quat): {orientation}")
        
        try:
            moveit2.move_to_pose(position=position, quat_xyzw=orientation, cartesian=False)
            moveit2.wait_until_executed()
            node.get_logger().info("   ✓ Movimento 2 concluído!")
        except Exception as e:
            node.get_logger().error(f"   ❌ Erro no movimento cartesiano: {e}")
        
        node.get_logger().info("")
        time.sleep(2.0)
        
        # Movimento 3: Desenhar um círculo
        node.get_logger().info("⭕ MOVIMENTO 3: Desenhando um círculo no plano ZY")
        node.get_logger().info("")
        
        # Definir parâmetros do círculo no plano ZY
        circle_center_x = 0.5  # X fixo (distância do robô)
        circle_center_y = 0.0  # Centro Y do círculo
        circle_center_z = 0.5  # Centro Z do círculo (altura)
        circle_radius = 0.08   # Raio do círculo (8 cm)
        circle_points = 20     # Número de pontos (mais = mais suave)
        
        # Desenhar o círculo
        draw_circle(
            moveit2=moveit2,
            node=node,
            center_x=circle_center_x,
            center_y=circle_center_y,
            center_z=circle_center_z,
            radius=circle_radius,
            num_points=circle_points,
            orientation=[1.0, 0.0, 0.0, 0.0]
        )
        
        node.get_logger().info("")
        time.sleep(2.0)
        
        # Retornar à posição HOME
        node.get_logger().info("🏠 MOVIMENTO 4: Retornando à posição HOME")
        try:
            moveit2.move_to_configuration(joint_positions)
            moveit2.wait_until_executed()
            node.get_logger().info("   ✓ Movimento 4 concluído!")
        except Exception as e:
            node.get_logger().error(f"   ❌ Erro retornando ao HOME: {e}")
        
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
