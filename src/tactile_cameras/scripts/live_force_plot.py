#!/usr/bin/env python3
"""
Live Force Plot Node
Subscribes to total_force from both GelSight cameras and plots the sum in real-time
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading


class LiveForcePlotNode(Node):
    """Node for plotting combined force from both GelSight cameras"""
    
    def __init__(self):
        super().__init__('live_force_plot')
        
        # Data buffers (store last 100 points)
        self.total_force_data = deque(maxlen=100)
        self.force1_data = deque(maxlen=100)
        self.force2_data = deque(maxlen=100)
        self.time_data = deque(maxlen=100)
        
        # Current force values
        self.force1 = 0.0
        self.force2 = 0.0
        self.data_lock = threading.Lock()
        
        # Start time
        self.start_time = None
        
        # Subscribers
        self.force1_sub = self.create_subscription(
            Float32MultiArray,
            '/feats/cam1/total_force',
            self.force1_callback,
            10
        )
        
        self.force2_sub = self.create_subscription(
            Float32MultiArray,
            '/feats/cam2/total_force',
            self.force2_callback,
            10
        )
        
        self.get_logger().info("📊 Live Force Plot Node initialized")
        self.get_logger().info("📡 Subscribing to:")
        self.get_logger().info("   • /feats/cam1/total_force")
        self.get_logger().info("   • /feats/cam2/total_force")
    
    def force1_callback(self, msg):
        """Callback for camera 1 total force"""
        with self.data_lock:
            # Float32MultiArray - pegar o primeiro valor ou a soma
            if len(msg.data) > 0:
                self.force1 = msg.data[0] if len(msg.data) == 1 else sum(msg.data)
            self._update_data()
    
    def force2_callback(self, msg):
        """Callback for camera 2 total force"""
        with self.data_lock:
            # Float32MultiArray - pegar o primeiro valor ou a soma
            if len(msg.data) > 0:
                self.force2 = msg.data[0] if len(msg.data) == 1 else sum(msg.data)
            self._update_data()
    
    def _update_data(self):
        """Update data buffers with current force values"""
        # Initialize start time on first data
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds
        
        # Calculate total force (sum of both sensors)
        total_force = self.force1 + self.force2
        
        # Append to buffers
        self.time_data.append(elapsed)
        self.force1_data.append(self.force1)
        self.force2_data.append(self.force2)
        self.total_force_data.append(total_force)
    
    def get_plot_data(self):
        """Get current plot data (thread-safe)"""
        with self.data_lock:
            return (
                list(self.time_data),
                list(self.force1_data),
                list(self.force2_data),
                list(self.total_force_data)
            )


def update_plot(frame, node, lines, ax):
    """Update function for matplotlib animation"""
    # Get current data
    time_data, force1_data, force2_data, total_force_data = node.get_plot_data()
    
    if len(time_data) == 0:
        return lines
    
    # Update lines
    lines[0].set_data(time_data, total_force_data)  # Total force (bold)
    lines[1].set_data(time_data, force1_data)       # Camera 1
    lines[2].set_data(time_data, force2_data)       # Camera 2
    
    # Auto-scale axes
    ax.relim()
    ax.autoscale_view()
    
    return lines


def main():
    """Main function"""
    rclpy.init()
    
    # Create node
    node = LiveForcePlotNode()
    
    # Setup matplotlib
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Create three lines: total, cam1, cam2
    line_total, = ax.plot([], [], 'b-', lw=2.5, label='Total Force (Cam1 + Cam2)')
    line_cam1, = ax.plot([], [], 'g--', lw=1.5, alpha=0.7, label='Camera 1')
    line_cam2, = ax.plot([], [], 'r--', lw=1.5, alpha=0.7, label='Camera 2')
    lines = [line_total, line_cam1, line_cam2]
    
    # Configure plot
    ax.set_xlabel("Tempo (s)", fontsize=12)
    ax.set_ylabel("Força (N)", fontsize=12)
    ax.set_title("Força Total dos Sensores GelSight em Tempo Real", fontsize=14, fontweight='bold')
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)
    
    print("\n" + "="*60)
    print("📊 Live Force Plot - GelSight Sensors")
    print("="*60)
    print("📡 Aguardando dados dos tópicos:")
    print("   • /gelsight_cam1/total_force")
    print("   • /gelsight_cam2/total_force")
    print("\n📈 Gráfico:")
    print("   🔵 Azul (sólido)  = Força Total (Cam1 + Cam2)")
    print("   🟢 Verde (tracejado) = Força Camera 1")
    print("   🔴 Vermelho (tracejado) = Força Camera 2")
    print("\n⌨️  Feche a janela do gráfico para sair")
    print("="*60 + "\n")
    
    # Start ROS2 spinning in separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Create animation
    ani = animation.FuncAnimation(
        fig,
        update_plot,
        fargs=(node, lines, ax),
        interval=100,  # Update every 100ms
        blit=True
    )
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Node shutdown complete")


if __name__ == '__main__':
    main()
