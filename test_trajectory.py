#!/usr/bin/env python3
"""
Script de teste rápido para verificar o envio de trajetórias.
Este script envia uma trajetória muito pequena e segura para testar a funcionalidade.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


def main():
    rclpy.init()
    node = Node('test_trajectory_sender')
    publisher = node.create_publisher(JointTrajectory, '/joint_trajectory', 10)
    
    node.get_logger().info('⏳ Aguardando conexão com o tópico...')
    time.sleep(1.0)
    
    # Criar trajetória de teste ULTRA SUAVE - SEM MOVER JUNTAS DO PULSO
    msg = JointTrajectory()
    msg.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
    
    # IMPORTANTE: Juntas 5, 6, 7 (pulso e rotação do efetuador) ficam FIXAS
    # Apenas as juntas do braço (1-4) se movem
    
    # Ponto 0: Estado atual (pausa inicial)
    point0 = JointTrajectoryPoint()
    point0.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    point0.velocities = [0.0] * 7
    point0.time_from_start = Duration(sec=60, nanosec=0)
    
    # Ponto 1: Pequeno movimento APENAS na junta 1 (base) - MUITO LENTO
    point1 = JointTrajectoryPoint()
    point1.positions = [0.05, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # Apenas 0.05 rad = ~2.8°
    point1.velocities = [0.0] * 7  # Velocidade zero no final
    point1.time_from_start = Duration(sec=120, nanosec=0)  # 5 segundos para mover 2.8° = 0.01 rad/s
    
    # Ponto 2: Pausa na posição
    point2 = JointTrajectoryPoint()
    point2.positions = [0.05, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    point2.velocities = [0.0] * 7
    point2.time_from_start = Duration(sec=180, nanosec=0)
    
    # Ponto 3: Retorna MUITO lentamente à posição inicial
    point3 = JointTrajectoryPoint()
    point3.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    point3.velocities = [0.0] * 7
    point3.time_from_start = Duration(sec=240, nanosec=0)  # Mais 5 segundos para voltar
    
    # Ponto 4: Pausa final
    point4 = JointTrajectoryPoint()
    point4.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    point4.velocities = [0.0] * 7
    point4.time_from_start = Duration(sec=300, nanosec=0)
    
    msg.points = [point0, point1, point2, point3, point4]
    
    # Publicar
    node.get_logger().info('📤 Enviando trajetória ULTRA SUAVE (SEM MOVER PULSO)...')
    node.get_logger().info(f'   - Pontos: {len(msg.points)}')
    node.get_logger().info(f'   - Duração total: 100 segundos')
    node.get_logger().info(f'   - Movimento: APENAS junta 1 (base) - 0.05 rad (~2.8°)')
    node.get_logger().info(f'   - Velocidade máxima: ~0.01 rad/s (MUITO LENTA)')
    node.get_logger().info(f'   - Juntas 5, 6, 7 (pulso): FIXAS - NÃO SE MOVEM')
    node.get_logger().info(f'   - Interpolação: Cúbica Hermite com velocidade zero nos extremos')
    
    publisher.publish(msg)
    
    node.get_logger().info('✅ Trajetória enviada com sucesso!')
    node.get_logger().info('   Monitore o robô para verificar o movimento.')
    node.get_logger().info('   Use "ros2 topic echo /rosout" para ver os logs do controlador.')
    
    time.sleep(1.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
