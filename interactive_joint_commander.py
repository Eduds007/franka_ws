#!/usr/bin/env python3
"""
Script interativo para controlar o robô Franka via comandos de junta.
Permite enviar comandos manualmente ou seguir trajetórias pré-definidas.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import sys


class InteractiveJointCommander(Node):
    def __init__(self):
        super().__init__('interactive_joint_commander')
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/admittance_example_controller/joint_command',
            10
        )
        
        # Posições pré-definidas
        self.positions = {
            'home': [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4],
            'ready': [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.7],
            'up': [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.7],
            'down': [0.0, -1.0, 0.0, -2.5, 0.0, 1.8, 0.7],
            'left': [0.5, -0.5, 0.0, -2.0, 0.0, 1.5, 0.7],
            'right': [-0.5, -0.5, 0.0, -2.0, 0.0, 1.5, 0.7],
        }
        
        self.get_logger().info('Interactive Joint Commander iniciado!')
    
    def publish_position(self, name, position):
        """Publica uma posição articular"""
        msg = Float64MultiArray()
        msg.data = position
        self.publisher.publish(msg)
        self.get_logger().info(f'Comando enviado: {name}')
        self.get_logger().info(f'  Posição: {[f"{x:.3f}" for x in position]}')
    
    def show_menu(self):
        """Mostra o menu de opções"""
        print("\n" + "="*60)
        print("COMANDOS DISPONÍVEIS:")
        print("="*60)
        print("\nPosições pré-definidas:")
        for name in self.positions.keys():
            print(f"  {name}")
        
        print("\nOutros comandos:")
        print("  custom  - Entrar posição manualmente")
        print("  current - Mostrar última posição enviada")
        print("  list    - Listar todas as posições detalhadas")
        print("  help    - Mostrar este menu")
        print("  quit    - Sair")
        print("="*60 + "\n")
    
    def list_positions(self):
        """Lista todas as posições detalhadas"""
        print("\n" + "="*60)
        print("POSIÇÕES DISPONÍVEIS (em radianos):")
        print("="*60)
        for name, pos in self.positions.items():
            print(f"\n{name.upper()}:")
            print(f"  {[f'{x:.3f}' for x in pos]}")
            print(f"  Graus: {[f'{math.degrees(x):.1f}°' for x in pos]}")
        print("="*60 + "\n")
    
    def get_custom_position(self):
        """Obtém posição customizada do usuário"""
        print("\nEntre com 7 valores (em radianos) separados por espaço:")
        print("Exemplo: 0.0 -0.785 0.0 -2.356 0.0 1.571 0.785")
        
        try:
            values = input(">>> ").strip().split()
            if len(values) != 7:
                print(f"Erro: esperado 7 valores, recebido {len(values)}")
                return None
            
            position = [float(v) for v in values]
            
            # Verificar limites básicos (±2.9 rad para a maioria das juntas)
            for i, val in enumerate(position):
                if abs(val) > 3.0:
                    print(f"Aviso: Junta {i+1} com valor alto ({val:.3f} rad = {math.degrees(val):.1f}°)")
            
            return position
        
        except ValueError as e:
            print(f"Erro ao converter valores: {e}")
            return None
    
    def run(self):
        """Loop principal interativo"""
        self.show_menu()
        last_position = None
        
        while rclpy.ok():
            try:
                command = input("Comando >>> ").strip().lower()
                
                if not command:
                    continue
                
                if command == 'quit' or command == 'q' or command == 'exit':
                    print("\nEncerrando...")
                    break
                
                elif command == 'help' or command == 'h':
                    self.show_menu()
                
                elif command == 'list' or command == 'l':
                    self.list_positions()
                
                elif command == 'current' or command == 'c':
                    if last_position:
                        print(f"\nÚltima posição enviada:")
                        print(f"  {[f'{x:.3f}' for x in last_position]}")
                    else:
                        print("\nNenhuma posição enviada ainda.")
                
                elif command == 'custom':
                    position = self.get_custom_position()
                    if position:
                        self.publish_position('CUSTOM', position)
                        last_position = position
                
                elif command in self.positions:
                    position = self.positions[command]
                    self.publish_position(command.upper(), position)
                    last_position = position
                
                else:
                    print(f"\nComando desconhecido: '{command}'")
                    print("Digite 'help' para ver comandos disponíveis.")
                
            except KeyboardInterrupt:
                print("\n\nInterrompido pelo usuário.")
                break
            
            except Exception as e:
                print(f"\nErro: {e}")


def main():
    print("\n" + "="*60)
    print("INTERACTIVE JOINT COMMANDER")
    print("Controlador de Juntas Interativo para Franka Emika Panda")
    print("="*60)
    
    rclpy.init()
    commander = InteractiveJointCommander()
    
    try:
        commander.run()
    finally:
        commander.destroy_node()
        rclpy.shutdown()
        print("\nEncerrando ROS2 node...")


if __name__ == '__main__':
    main()
