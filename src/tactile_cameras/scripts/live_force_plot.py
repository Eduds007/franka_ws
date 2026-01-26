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
        self.time_data = deque(maxlen=100)
        
        # Force components for camera 1 (x, y, z)
        self.force1_x_data = deque(maxlen=100)
        self.force1_y_data = deque(maxlen=100)
        self.force1_z_data = deque(maxlen=100)
        
        # Force components for camera 2 (x, y, z)
        self.force2_x_data = deque(maxlen=100)
        self.force2_y_data = deque(maxlen=100)
        self.force2_z_data = deque(maxlen=100)
        
        # Total force magnitude
        self.total_force_data = deque(maxlen=100)
        
        # Current force values
        self.force1_x = 0.0
        self.force1_y = 0.0
        self.force1_z = 0.0
        self.force2_x = 0.0
        self.force2_y = 0.0
        self.force2_z = 0.0
        
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
            # Float32MultiArray - extrair componentes x, y, z
            if len(msg.data) >= 3:
                self.force1_x = msg.data[0]
                self.force1_y = msg.data[1]
                self.force1_z = msg.data[2]
            self._update_data()
    
    def force2_callback(self, msg):
        """Callback for camera 2 total force"""
        with self.data_lock:
            # Float32MultiArray - extrair componentes x, y, z
            if len(msg.data) >= 3:
                self.force2_x = msg.data[0]
                self.force2_y = msg.data[1]
                self.force2_z = msg.data[2]
            self._update_data()
    
    def _update_data(self):
        """Update data buffers with current force values"""
        # Initialize start time on first data
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds
        
        # Calculate total force magnitude (soma das magnitudes de cada sensor)
        import math
        mag1 = math.sqrt(self.force1_x**2 + self.force1_y**2 + self.force1_z**2)
        mag2 = math.sqrt(self.force2_x**2 + self.force2_y**2 + self.force2_z**2)
        total_force = mag1 + mag2
        
        # Append to buffers
        self.time_data.append(elapsed)
        
        self.force1_x_data.append(self.force1_x)
        self.force1_y_data.append(self.force1_y)
        self.force1_z_data.append(self.force1_z)  # Mantém para compatibilidade
        
        self.force2_x_data.append(self.force2_x)
        self.force2_y_data.append(self.force2_y)
        self.force2_z_data.append(self.force2_z)  # Mantém para compatibilidade

        self.total_force_data.append(total_force)
    
    def get_plot_data(self):
        """Get current plot data (thread-safe)"""
        with self.data_lock:
            return (
                list(self.time_data),
                list(self.force1_x_data),
                list(self.force1_y_data),
                list(self.force1_z_data),
                list(self.force2_x_data),
                list(self.force2_y_data),
                list(self.force2_z_data),
                list(self.total_force_data)
            )


def update_plot(frame, node, lines, ax):
    """Update function for matplotlib animation"""
    # Get current data
    (time_data, force1_x_data, force1_y_data, force1_z_data,
     force2_x_data, force2_y_data, force2_z_data, total_force_data) = node.get_plot_data()
    
    if len(time_data) == 0:
        return lines
    
    # Update lines - only X, Y components + total (Z hidden)
    lines[0].set_data(time_data, force1_x_data)  # Cam1 X
    lines[1].set_data(time_data, force1_y_data)  # Cam1 Y
    # lines[2] = Cam1 Z (hidden)
    lines[2].set_data(time_data, force2_x_data)  # Cam2 X
    lines[3].set_data(time_data, force2_y_data)  # Cam2 Y
    # lines[5] = Cam2 Z (hidden)
    #lines[4].set_data(time_data, total_force_data)  # Total magnitude
    
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
    fig, ax = plt.subplots(figsize=(12, 7))
    
    # Create 5 lines: 2 for cam1 (x,y), 2 for cam2 (x,y), 1 for total (Z hidden)
    line_cam1_x, = ax.plot([], [], 'r-', lw=1.5, alpha=0.7, label='Cam1 X')
    line_cam1_y, = ax.plot([], [], 'g-', lw=1.5, alpha=0.7, label='Cam1 Y')
    
    line_cam2_x, = ax.plot([], [], 'r--', lw=1.5, alpha=0.7, label='Cam2 X')
    line_cam2_y, = ax.plot([], [], 'g--', lw=1.5, alpha=0.7, label='Cam2 Y')
    
    #line_total, = ax.plot([], [], 'k-', lw=3, label='Total Magnitude')
    
    lines = [line_cam1_x, line_cam1_y,
             line_cam2_x, line_cam2_y,
             #line_total
            ]
    
    # Configure plot
    ax.set_xlabel("Tempo (s)", fontsize=12)
    ax.set_ylabel("Força (N)", fontsize=12)
    ax.set_title("Componentes de Força dos Sensores GelSight (X, Y)", fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    print("\n" + "="*60)
    print("📊 Live Force Plot - GelSight Sensors (X, Y)")
    print("="*60)
    print("📡 Aguardando dados dos tópicos:")
    print("   • /feats/cam1/total_force")
    print("   • /feats/cam2/total_force")
    print("\n📈 Gráfico:")
    print("   🔴 Vermelho (sólido)    = Camera 1 - Força X")
    print("   🟢 Verde (sólido)       = Camera 1 - Força Y")
    print("   🔴 Vermelho (tracejado) = Camera 2 - Força X")
    print("   🟢 Verde (tracejado)    = Camera 2 - Força Y")
    print("   ⚫ Preto (grosso)       = Magnitude Total")
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
