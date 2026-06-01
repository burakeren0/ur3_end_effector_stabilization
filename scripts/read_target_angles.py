#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class TargetAnglesReader(Node):
    def __init__(self, topic: str, output_file: Path | None):
        super().__init__("target_angles_reader")
        self.get_logger().info(f"Subscribing to target angles: {topic}")

        self.subscription = self.create_subscription(
            Float64MultiArray,
            topic,
            self.target_angles_callback,
            10,
        )
        self.subscription

        self.csv_file = None
        self.csv_writer = None
        if output_file is not None:
            output_file.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = open(output_file, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["time", "q1", "q2", "q3", "q4", "q5", "q6"])
            self.get_logger().info(f"Logging target angles to: {output_file}")

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()

    def target_angles_callback(self, msg: Float64MultiArray):
        angles = list(msg.data[:6])
        if len(angles) < 6:
            self.get_logger().warn(
                f"Beklenenden daha kısa hedef açı verisi aldı: {len(angles)} öğe"
            )
            angles += [0.0] * (6 - len(angles))

        self.get_logger().info(
            "Hedef eklem açıları: [" + ", ".join(f"{angle:.6f}" for angle in angles) + "]"
        )

        if self.csv_writer is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            self.csv_writer.writerow([f"{now:.9f}", *angles])


def main():
    parser = argparse.ArgumentParser(
        description="Read target angles for 6 joints from /target_angles during simulation"
    )
    parser.add_argument(
        "--topic",
        default="/target_angles",
        help="ROS 2 topic to read target joint angles from",
    )
    parser.add_argument(
        "--output-file",
        default="target_angles_log.csv",
        help="Optional CSV file to save received target angles",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TargetAnglesReader(args.topic, Path(args.output_file) if args.output_file else None)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
