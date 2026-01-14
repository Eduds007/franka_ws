#!/usr/bin/env python3
"""
Script simples para monitorar os valores de força do tópico /feats/cam1/total_force.
Apenas se inscreve no tópico e exibe os valores recebidos.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ForceMonitor(Node):
    def __init__(self):
        super().__init__('force_monitor')
        
        # Criar subscriber para o tópico de força
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/feats/cam1/total_force',
            self.force_callback,
            10
        )
        
        self.get_logger().info("="*70)
        self.get_logger().info("MONITOR DE FORÇA - Tópico /feats/cam1/total_force")
        self.get_logger().info("="*70)
        self.get_logger().info("📡 Aguardando mensagens do tópico...")
        self.get_logger().info("")
        
        self.message_count = 0
    
    def force_callback(self, msg):
        """Callback executado quando uma mensagem é recebida do tópico."""
        self.message_count += 1
        
        # Converter para lista para facilitar manipulação
        force_values = list(msg.data)
        
        # Formatar valores para exibição
        formatted_values = [f"{v:+.4f}" for v in force_values]
        
        # Verificar se algum valor excede o limiar de 1.0
        max_value = max(abs(v) for v in force_values)
        alert = "🚨 ALERTA!" if max_value > 1.0 else "✓"
        
        # Exibir informações
        self.get_logger().info(f"Msg #{self.message_count} {alert}")
        self.get_logger().info(f"  Valores: {formatted_values}")
        self.get_logger().info(f"  Máximo (abs): {max_value:.4f}")
        
        # Se houver alerta, mostrar quais valores ultrapassaram
        if max_value > 1.0:
            exceeded = [i for i, v in enumerate(force_values) if abs(v) > 1.0]
            self.get_logger().warn(f"  ⚠️  Índices que excederam limiar (|v| > 1.0): {exceeded}")
        
        self.get_logger().info("")


def main(args=None):
    rclpy.init(args=args)
    
    force_monitor = ForceMonitor()
    
    try:
        rclpy.spin(force_monitor)
    except KeyboardInterrupt:
        force_monitor.get_logger().info("⚠️  Interrompido pelo usuário!")
    finally:
        force_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
