#!/usr/bin/env python3
"""
Script para publicar trajetórias para o HybridPositionForceController.

Exemplo de uso:
    python3 publish_trajectory.py --mode simple
    python3 publish_trajectory.py --mode circular
    python3 publish_trajectory.py --mode custom
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import argparse


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.get_logger().info('Trajectory Publisher initialized')
        
    def send_simple_trajectory(self):
        """Envia uma trajetória simples de 3 pontos com velocidades suaves."""
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        
        # Ponto 1: posição inicial (home) - velocidade zero
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        point1.velocities = [0.0] * 7
        point1.time_from_start = Duration(sec=0, nanosec=0)
        
        # Ponto 2: posição intermediária - velocidade zero para parar suavemente
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, -0.5, 0.3, -2.0, 0.2, 1.8, 0.5]
        point2.velocities = [0.0] * 7
        point2.time_from_start = Duration(sec=5, nanosec=0)  # Tempo maior para movimento suave
        
        # Ponto 3: pausa na posição intermediária
        point3 = JointTrajectoryPoint()
        point3.positions = [0.5, -0.5, 0.3, -2.0, 0.2, 1.8, 0.5]
        point3.velocities = [0.0] * 7
        point3.time_from_start = Duration(sec=7, nanosec=0)
        
        # Ponto 4: retorna para home - velocidade zero no final
        point4 = JointTrajectoryPoint()
        point4.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        point4.velocities = [0.0] * 7
        point4.time_from_start = Duration(sec=12, nanosec=0)
        
        msg.points = [point1, point2, point3, point4]
        self.publisher.publish(msg)
        self.get_logger().info(f'Simple trajectory published with {len(msg.points)} points (SMOOTH)')
        self.get_logger().info('  All waypoints have zero velocity for smooth starts/stops')
        
    def send_circular_trajectory(self, radius=0.3, num_points=20, duration=10.0):
        """Envia uma trajetória circular (movimento primariamente nas juntas 1 e 3) com velocidades suaves."""
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        
        # Posição central
        q_center = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        
        for i in range(num_points + 1):
            point = JointTrajectoryPoint()
            
            # Movimento circular nas juntas 1 e 3
            theta = 2 * np.pi * i / num_points
            q = q_center.copy()
            q[0] = q_center[0] + radius * np.cos(theta)
            q[2] = q_center[2] + radius * np.sin(theta)
            
            point.positions = q.tolist()
            
            # Calcular velocidades para movimento suave
            # Velocidade angular constante
            omega = 2 * np.pi / duration
            v = np.zeros(7)
            v[0] = -radius * omega * np.sin(theta)
            v[2] = radius * omega * np.cos(theta)
            
            # Suavizar início e fim
            if i == 0 or i == num_points:
                v *= 0.0  # Velocidade zero no início e fim
            
            point.velocities = v.tolist()
            
            # Tempo proporcional
            t = duration * i / num_points
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            
            msg.points.append(point)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Circular trajectory published with {len(msg.points)} points (SMOOTH)')
        self.get_logger().info(f'  Radius: {radius} rad, Duration: {duration}s')
        
    def send_sinusoidal_trajectory(self, amplitude=0.4, frequency=0.5, duration=10.0, num_points=50):
        """Envia uma trajetória sinusoidal com velocidades calculadas."""
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        
        # Posição central
        q_center = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        
        for i in range(num_points + 1):
            point = JointTrajectoryPoint()
            
            t = duration * i / num_points
            q = q_center.copy()
            
            # Movimento sinusoidal na junta 1
            q[0] = q_center[0] + amplitude * np.sin(2 * np.pi * frequency * t)
            # Movimento sinusoidal defasado na junta 3
            q[2] = q_center[2] + amplitude * 0.5 * np.sin(2 * np.pi * frequency * t + np.pi/2)
            
            point.positions = q.tolist()
            
            # Calcular velocidades analiticamente
            v = np.zeros(7)
            v[0] = amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * t)
            v[2] = amplitude * 0.5 * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * t + np.pi/2)
            
            # Suavizar início e fim
            if i == 0 or i == num_points:
                v *= 0.0
            
            point.velocities = v.tolist()
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            
            msg.points.append(point)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Sinusoidal trajectory published with {len(msg.points)} points (SMOOTH)')
        self.get_logger().info(f'  Amplitude: {amplitude} rad, Frequency: {frequency} Hz')
        
    def send_custom_trajectory(self, waypoints, total_duration):
        """
        Envia uma trajetória customizada.
        
        Args:
            waypoints: Lista de posições das juntas (cada uma com 7 valores)
            total_duration: Duração total da trajetória em segundos
        """
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        
        num_waypoints = len(waypoints)
        for i, waypoint in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            
            # Distribuir tempo uniformemente
            t = total_duration * i / (num_waypoints - 1) if num_waypoints > 1 else 0.0
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            
            msg.points.append(point)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Custom trajectory published with {len(msg.points)} points')


def main():
    parser = argparse.ArgumentParser(description='Publish joint trajectories to Franka robot')
    parser.add_argument('--mode', type=str, default='simple',
                       choices=['simple', 'circular', 'sinusoidal', 'custom'],
                       help='Trajectory mode to use')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Total duration of trajectory in seconds')
    parser.add_argument('--radius', type=float, default=0.3,
                       help='Radius for circular trajectory')
    parser.add_argument('--amplitude', type=float, default=0.4,
                       help='Amplitude for sinusoidal trajectory')
    parser.add_argument('--frequency', type=float, default=0.5,
                       help='Frequency for sinusoidal trajectory')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = TrajectoryPublisher()
    
    # Pequeno delay para garantir que o publisher está pronto
    import time
    time.sleep(0.5)
    
    if args.mode == 'simple':
        node.send_simple_trajectory()
    elif args.mode == 'circular':
        node.send_circular_trajectory(radius=args.radius, duration=args.duration)
    elif args.mode == 'sinusoidal':
        node.send_sinusoidal_trajectory(amplitude=args.amplitude, 
                                       frequency=args.frequency,
                                       duration=args.duration)
    elif args.mode == 'custom':
        # Exemplo de trajetória customizada
        waypoints = [
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],  # Home
            [0.5, -1.0, 0.5, -2.0, 0.3, 1.8, 0.5],          # Posição 2
            [-0.3, -0.5, -0.2, -2.5, -0.1, 1.4, 0.8],       # Posição 3
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],  # Retorna home
        ]
        node.send_custom_trajectory(waypoints, args.duration)
    
    node.get_logger().info('Trajectory sent successfully!')
    
    # Manter o nó vivo por um curto período
    time.sleep(1.0)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
