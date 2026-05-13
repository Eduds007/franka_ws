#!/usr/bin/env python3
# Publishes a circular EE trajectory to /multi_priority_controller/desired_ee_pose
# centered on the EE position at the moment this node receives its first robot state.
# Orientation is held at the captured value. The Franka multi_priority_controller
# tracks this pose in the null-space of the cable-tension task.

import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaRobotState


def rot_to_quat(R):
    # Numerically stable rotation matrix → quaternion (w, x, y, z).
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return w / n, x / n, y / n, z / n


class CircleTrajectoryPublisher(Node):
    def __init__(self, radius, period, rate, plane, frame_id):
        super().__init__("circle_trajectory_publisher")
        self.radius = radius
        self.omega = 2.0 * math.pi / period
        self.plane = plane
        self.frame_id = frame_id

        self.center = None      # np.array(3,)
        self.qx_axis = None     # circle in-plane axis 1 (unit)
        self.qy_axis = None     # circle in-plane axis 2 (unit)
        self.quat = None        # held orientation (w, x, y, z)
        self.t0 = None

        self.pub = self.create_publisher(
            PoseStamped, "/multi_priority_controller/desired_ee_pose", 10
        )
        self.state_sub = self.create_subscription(
            FrankaRobotState,
            "/franka_robot_state_broadcaster/robot_state",
            self._on_state,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Waiting for first robot state… radius={radius:.3f} m, period={period:.2f} s, plane={plane}"
        )

    def _on_state(self, msg: FrankaRobotState):
        if self.center is not None:
            return
        # o_t_ee is a 4x4 homogeneous transform in column-major order (libfranka convention).
        T = np.array(msg.o_t_ee, dtype=float).reshape(4, 4, order="F")
        R = T[:3, :3]
        p = T[:3, 3]

        if self.plane == "xy":
            ax1, ax2 = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
        elif self.plane == "xz":
            ax1, ax2 = np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])
        elif self.plane == "yz":
            ax1, ax2 = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
        else:
            raise ValueError(f"unknown plane: {self.plane}")

        self.center = p
        self.qx_axis = ax1
        self.qy_axis = ax2
        self.quat = rot_to_quat(R)
        self.t0 = self.get_clock().now()
        self.get_logger().info(
            f"Captured center=[{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] — starting circle"
        )

    def _tick(self):
        if self.center is None:
            return
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        c, s = math.cos(self.omega * t), math.sin(self.omega * t)
        offset = self.radius * (c * self.qx_axis + s * self.qy_axis)
        p = self.center + offset

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.position.z = float(p[2])
        w, x, y, z = self.quat
        msg.pose.orientation.w = w
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=0.05, help="circle radius [m]")
    ap.add_argument("--period", type=float, default=12.0, help="orbit period [s]")
    ap.add_argument("--rate", type=float, default=100.0, help="publish rate [Hz]")
    ap.add_argument("--plane", choices=["xy", "xz", "yz"], default="xy")
    ap.add_argument("--frame", default="panda_link0", help="header frame_id")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = CircleTrajectoryPublisher(args.radius, args.period, args.rate, args.plane, args.frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
