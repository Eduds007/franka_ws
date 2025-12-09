#!/usr/bin/env python3
"""
Teste do Detector GelSight com Câmeras Reais
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
import time


class GelSightTestMonitor(Node):
    """Monitor de teste para o detector GelSight"""
    
    def __init__(self):
        super().__init__('gelsight_test_monitor')
        
        # Subscribers para monitorar a detecção
        self.detection_sub = self.create_subscription(
            Bool, '/tension_control/object_detected', self.detection_callback, 10)
        self.confidence_sub = self.create_subscription(
            Float32, '/tension_control/object_confidence', self.confidence_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/tension_control/detection_status', self.status_callback, 10)
        
        self.last_detection = False
        self.last_confidence = 0.0
        self.last_status = ""
        
        # Timer para mostrar status
        self.status_timer = self.create_timer(1.0, self.show_status)
        
        self.get_logger().info("🔍 GelSight Test Monitor iniciado")
        print("\n" + "="*60)
        print("🔍 Monitor de Detecção GelSight")
        print("="*60)
        print("Aguardando dados de detecção...")
        print("✋ Pressione objetos nas superfícies GelSight para testar")
        print("⌨️  Press Ctrl+C para parar")
        print("="*60)
    
    def detection_callback(self, msg):
        """Callback para detecção de objetos"""
        if msg.data != self.last_detection:
            if msg.data:
                print(f"\n🎯 OBJETO DETECTADO! Timestamp: {time.strftime('%H:%M:%S')}")
            else:
                print(f"\n✋ Objeto removido. Timestamp: {time.strftime('%H:%M:%S')}")
        self.last_detection = msg.data
    
    def confidence_callback(self, msg):
        """Callback para confiança da detecção"""
        self.last_confidence = msg.data
    
    def status_callback(self, msg):
        """Callback para status da detecção"""
        self.last_status = msg.data
    
    def show_status(self):
        """Mostra status atual"""
        status_icon = "🔴" if self.last_detection else "🟢"
        timestamp = time.strftime('%H:%M:%S')
        print(f"\r[{timestamp}] {status_icon} {self.last_status} | Confiança: {self.last_confidence:.4f}", end="", flush=True)


def main():
    rclpy.init()
    
    try:
        monitor = GelSightTestMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n🛑 Teste finalizado")
    finally:
        if 'monitor' in locals():
            monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
