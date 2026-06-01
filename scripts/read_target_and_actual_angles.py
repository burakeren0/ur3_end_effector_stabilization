#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_NAME_ALIASES = {
    "shoulder_pan_joint": ["shoulder_pan_joint", "ur_shoulder_pan_joint"],
    "shoulder_lift_joint": ["shoulder_lift_joint", "ur_shoulder_lift_joint"],
    "elbow_joint": ["elbow_joint", "ur_elbow_joint"],
    "wrist_1_joint": ["wrist_1_joint", "ur_wrist_1_joint"],
    "wrist_2_joint": ["wrist_2_joint", "ur_wrist_2_joint"],
    "wrist_3_joint": ["wrist_3_joint", "ur_wrist_3_joint"],
}


def resolve_joint_position(name_to_position, joint_name):
    # Support both raw UR joint names and prefixed names used by controllers.
    for alias in JOINT_NAME_ALIASES.get(joint_name, [joint_name]):
        if alias in name_to_position:
            return float(name_to_position[alias])
    # Accept suffix match as fallback (e.g. "/ur_shoulder_pan_joint" or namespaced strings).
    for actual_name, position in name_to_position.items():
        if actual_name.endswith(joint_name):
            return float(position)
    return None


class JointAnglesMonitor(Node):
    def __init__(self, target_topic: str, actual_topic: str, output_file: Path | None, sample_rate: float):
        super().__init__("joint_angles_monitor")
        self.get_logger().info(
            f"Subscribing to target: {target_topic}, actual: {actual_topic}"
        )

        self.latest_target = None
        self.latest_actual = None

        self.create_subscription(
            Float64MultiArray,
            target_topic,
            self.target_callback,
            10,
        )
        self.create_subscription(
            JointState,
            actual_topic,
            self.actual_callback,
            10,
        )

        self.csv_file = None
        self.csv_writer = None
        if output_file is not None:
            output_file.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = open(output_file, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            header = ["time"] + [f"target_{name}" for name in JOINT_NAMES] + [f"actual_{name}" for name in JOINT_NAMES]
            self.csv_writer.writerow(header)
            self.get_logger().info(f"Logging target+actual joint angles to: {output_file}")

        self.timer = self.create_timer(1.0 / sample_rate, self.timer_callback)

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()

    def target_callback(self, msg: Float64MultiArray):
        angles = [float(value) for value in msg.data[:6]]
        if len(angles) < 6:
            angles += [0.0] * (6 - len(angles))
        self.latest_target = angles

    def actual_callback(self, msg: JointState):
        positions = [0.0] * 6
        name_to_position = {name: pos for name, pos in zip(msg.name, msg.position)}
        for index, joint_name in enumerate(JOINT_NAMES):
            resolved = resolve_joint_position(name_to_position, joint_name)
            if resolved is not None:
                positions[index] = resolved
            else:
                self.get_logger().debug(
                    f"Actual joint state missing {joint_name} (or alias), using 0.0"
                )

        if all(pos == 0.0 for pos in positions) and len(msg.position) >= 6:
            self.get_logger().debug(
                "No joint name matches found; falling back to first 6 positions from JointState."
            )
            positions = [float(pos) for pos in msg.position[:6]]

        self.latest_actual = positions

    def timer_callback(self):
        if self.latest_target is None or self.latest_actual is None:
            return
        time_s = self.get_clock().now().nanoseconds * 1e-9

        # Normalize actual angles to be the nearest equivalent to target angles
        adjusted_actual = []
        two_pi = 2.0 * math.pi
        for t, a in zip(self.latest_target, self.latest_actual):
            # compute how many 2pi multiples separate actual from target
            diff = a - t
            shift = round(diff / two_pi)
            a_adj = a - shift * two_pi
            adjusted_actual.append(a_adj)

        target_str = ", ".join(f"{angle:.6f}" for angle in self.latest_target)
        actual_str = ", ".join(f"{angle:.6f}" for angle in adjusted_actual)

        self.get_logger().info(
            f"Target angles: [{target_str}] | Actual (adj): [{actual_str}]"
        )

        if self.csv_writer is not None:
            self.csv_writer.writerow(
                [f"{time_s:.9f}", *self.latest_target, *adjusted_actual]
            )


def main():
    parser = argparse.ArgumentParser(
        description="Read both target and actual joint angles for UR3 and log them."
    )
    parser.add_argument(
        "--target-topic",
        default="/target_angles",
        help="ROS 2 topic to read target joint angles from",
    )
    parser.add_argument(
        "--actual-topic",
        default="/joint_states",
        help="ROS 2 topic to read actual joint states from",
    )
    parser.add_argument(
        "--output-file",
        default="joint_angles_target_actual_log.csv",
        help="Optional CSV file to save target and actual joint angles",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Logging rate in Hz",
    )
    args = parser.parse_args()

    rclpy.init()
    node = JointAnglesMonitor(
        args.target_topic,
        args.actual_topic,
        Path(args.output_file) if args.output_file else None,
        args.rate,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
