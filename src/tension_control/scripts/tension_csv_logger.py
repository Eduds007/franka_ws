#!/usr/bin/env python3
"""CSV logger for multi_priority_controller.

Subscribes to /multi_priority_controller/log_state (std_msgs/Float64MultiArray)
and writes one row per message to a timestamped CSV file. The controller
publishes at ~100 Hz with the layout:

    [t_sec, T_des_target, T_des_active, T_meas, ee_x, ee_y, ee_z]

Default output path: ~/franka_ws/logs/tension_<YYYYMMDD_HHMMSS>.csv
Override with the ROS param `output_dir` (string).
"""

import csv
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

DEFAULT_OUTPUT_DIR = str(Path.home() / "franka_ws" / "logs")
TOPIC = "/multi_priority_controller/log_state"
COLUMNS = ["t_sec", "T_des_target_N", "T_des_active_N", "T_meas_N",
           "ee_x_m", "ee_y_m", "ee_z_m"]


class TensionCsvLogger(Node):
    def __init__(self):
        super().__init__("tension_csv_logger")

        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        out_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        os.makedirs(out_dir, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(out_dir, f"tension_{stamp}.csv")

        self._file = open(self.csv_path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(COLUMNS)
        self._file.flush()

        self._row_count = 0
        self.sub = self.create_subscription(
            Float64MultiArray, TOPIC, self._on_msg, 50)

        self.get_logger().info(f"Logging to {self.csv_path}")
        self.get_logger().info(f"Subscribed to {TOPIC}")

    def _on_msg(self, msg: Float64MultiArray):
        if len(msg.data) != len(COLUMNS):
            self.get_logger().warn_once(
                f"Expected {len(COLUMNS)} fields, got {len(msg.data)} — writing anyway")
        self._writer.writerow(list(msg.data))
        self._row_count += 1
        if self._row_count % 500 == 0:  # ~5 s at 100 Hz
            self._file.flush()

    def shutdown(self):
        try:
            self._file.flush()
            self._file.close()
        except Exception as exc:
            self.get_logger().error(f"CSV close failed: {exc}")
        self.get_logger().info(
            f"Wrote {self._row_count} rows to {self.csv_path}")


def main():
    rclpy.init()
    node = TensionCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
