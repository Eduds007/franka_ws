#!/usr/bin/env python3
"""
Script para publicar comandos de posição articular para o controlador de admitância.

Uso:
    python3 publish_joint_command.py [mode]
    
Modos:
    - home: Envia o robô para a posição home padrão
    - circular: Gera movimento circular no plano YZ
    - custom: Envia posições customizadas
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import sys
import time


class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        
        # Publisher para comandos de junta
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/admittance_example_controller/joint_command',
            10
        )
        
        self.get_logger().info('Joint Command Publisher iniciado')
        self.get_logger().info('Publicando em: /admittance_example_controller/joint_command')
    
    def publish_home_position(self):
        """Publica a posição home padrão do Franka Panda"""
        msg = Float64MultiArray()
        # Home position: [0, -π/4, 0, -3π/4, 0, π/2, π/4]
        msg.data = [
            0.0,
            -math.pi / 4,
            0.0,
            -3 * math.pi / 4,
            0.0,
            math.pi / 2,
            math.pi / 4
        ]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Comando HOME publicado: {[f"{x:.3f}" for x in msg.data]}')
    
    def publish_circular_motion(self, radius=0.1, period=8.0, duration=30.0):
        """
        Publica comandos para movimento circular no plano YZ
        
        Args:
            radius: Raio do círculo em metros
            period: Período para completar um círculo em segundos
            duration: Duração total do movimento em segundos
        """
        self.get_logger().info(f'Iniciando movimento circular:')
        self.get_logger().info(f'  - Raio: {radius} m')
        self.get_logger().info(f'  - Período: {period} s')
        self.get_logger().info(f'  - Duração: {duration} s')
        
        # Home position
        q_home = [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4]
        
        start_time = time.time()
        dt = 0.02  # 50 Hz
        
        omega = 2.0 * math.pi / period
        
        while (time.time() - start_time) < duration:
            elapsed = time.time() - start_time
            theta = omega * elapsed
            
            # Calcular offsets no plano YZ
            y_offset = radius * math.cos(theta)
            z_offset = radius * math.sin(theta)
            
            # Para movimento cartesiano, precisaríamos da cinemática inversa
            # Neste exemplo simplificado, vamos apenas variar algumas juntas
            msg = Float64MultiArray()
            msg.data = q_home.copy()
            
            # Modificar juntas para aproximar movimento circular
            # (isso é uma aproximação - idealmente usaríamos IK)
            msg.data[1] += y_offset * 2.0  # Junta 2
            msg.data[3] += z_offset * 1.5  # Junta 4
            
            self.publisher.publish(msg)
            
            # Log a cada 2 segundos
            if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.02:
                self.get_logger().info(f'Círculo em progresso... {elapsed:.1f}s / {duration:.1f}s')
            
            # Sleep para controlar a taxa de publicação
            time.sleep(dt)
            
            # Processar callbacks pendentes do ROS2
            rclpy.spin_once(self, timeout_sec=0)
        
        self.get_logger().info('Movimento circular concluído!')
    
    def publish_custom_position(self, joint_positions):
        """
        Publica uma posição articular customizada
        
        Args:
            joint_positions: Lista com 7 valores (posições em radianos)
        """
        if len(joint_positions) != 7:
            self.get_logger().error(f'Erro: esperado 7 posições, recebido {len(joint_positions)}')
            return
        
        msg = Float64MultiArray()
        msg.data = joint_positions
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Comando CUSTOM publicado: {[f"{x:.3f}" for x in msg.data]}')


def main(args=None):
    rclpy.init(args=args)
    
    publisher = JointCommandPublisher()
    
    # Verificar argumentos da linha de comando
    if len(sys.argv) < 2:
        mode = 'home'
    else:
        mode = sys.argv[1].lower()
    
    try:
        if mode == 'home':
            print('\n=== MODO: HOME ===')
            print('Enviando robô para posição home...\n')
            publisher.publish_home_position()
            time.sleep(2)
        
        elif mode == 'circular':
            print('\n=== MODO: MOVIMENTO CIRCULAR ===')
            print('Primeiro indo para home, depois circular...\n')
            publisher.publish_home_position()
            time.sleep(5)  # Esperar chegar no home
            publisher.publish_circular_motion(radius=0.1, period=8.0, duration=30.0)
        
        elif mode == 'custom':
            print('\n=== MODO: POSIÇÃO CUSTOMIZADA ===')
            print('Enviando posição de exemplo...\n')
            # Exemplo: posição ligeiramente diferente do home
            custom_pos = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.7]
            publisher.publish_custom_position(custom_pos)
            time.sleep(2)
        
        else:
            print(f'Modo desconhecido: {mode}')
            print('Modos disponíveis: home, circular, custom')
            return
        
        # Manter o node ativo
        print('\nNode ativo. Pressione Ctrl+C para sair.\n')
        rclpy.spin(publisher)
    
    except KeyboardInterrupt:
        print('\n\nInterrompido pelo usuário.')
    
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
