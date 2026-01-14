#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class TotalForceListener(Node):

    def __init__(self):
        super().__init__('total_force_listener')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'feats/cam1/total_force',
            self.listener_callback,
            10
        )
        self.get_logger().info("Inscrito no tópico feats/cam1/total_force...")

    def listener_callback(self, msg: Float32MultiArray):
        # msg.data é uma lista de floats (ex: [f1, f2, f3, ...])
        self.get_logger().info(f"Valores recebidos: {msg.data}")
        
        # Verifica se algum valor é maior que 1
        valores_maiores_que_1 = [val for val in msg.data if val < -1.0]
        if valores_maiores_que_1:
            self.get_logger().warn(f"⚠️  ALERTA: Valores maiores que 1 detectados: {valores_maiores_que_1}")
            self.get_logger().info(f"Total de valores > 1: {len(valores_maiores_que_1)}")

def main(args=None):
    rclpy.init(args=args)
    node = TotalForceListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
