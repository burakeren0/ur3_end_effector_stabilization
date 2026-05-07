#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def normalize_quaternion(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def conjugate_quaternion(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def multiply_quaternions(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return normalize_quaternion((
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ))


def quaternion_to_rpy(q):
    x, y, z, w = normalize_quaternion(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rpy_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


class ImuTool0Stabilizer(Node):
    def __init__(self):
        super().__init__("imu_tool0_stabilizer")

        self.declare_parameter("imu_topic", "/imu_data")
        self.declare_parameter("base_frame", "ur_base_link_inertia")
        self.declare_parameter("stabilized_child_frame", "ur_tool0_stabilized")
        self.declare_parameter("pose_topic", "/ur_tool0_stabilized_pose")
        self.declare_parameter("compensate_yaw", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("print_rate_hz", 0.0)
        self.declare_parameter("verbose", False)
        self.declare_parameter("position", [-0.29515, -0.11235, 0.48090])
        self.declare_parameter("flat_tool0_quaternion", [0.5, 0.5, -0.5, -0.5])

        self.imu_topic = self.get_parameter("imu_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.child_frame = self.get_parameter("stabilized_child_frame").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.compensate_yaw = bool(self.get_parameter("compensate_yaw").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.verbose = bool(self.get_parameter("verbose").value)

        self.position = tuple(float(v) for v in self.get_parameter("position").value)
        self.flat_tool0_q = normalize_quaternion(
            tuple(float(v) for v in self.get_parameter("flat_tool0_quaternion").value)
        )

        self.initial_base_q = None
        self.current_base_q = None
        self.current_tool0_q = self.flat_tool0_q
        self.last_print_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)

        rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_stabilized_pose)
        print_rate_hz = max(0.0, float(self.get_parameter("print_rate_hz").value))
        self.print_period_sec = (1.0 / print_rate_hz) if print_rate_hz > 0.0 else None

        if self.verbose:
            self.get_logger().info(
                f"IMU tool0 stabilizer started. Pose: {self.pose_topic}, TF: {self.base_frame}->{self.child_frame}"
            )

    def imu_callback(self, msg):
        q = normalize_quaternion((
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ))

        if not self.compensate_yaw:
            roll, pitch, _ = quaternion_to_rpy(q)
            q = rpy_to_quaternion(roll, pitch, 0.0)

        if self.initial_base_q is None:
            self.initial_base_q = q
            if self.verbose:
                self.get_logger().info("Initial IMU orientation calibrated as flat reference.")

        self.current_base_q = q

        base_delta = multiply_quaternions(
            self.current_base_q,
            conjugate_quaternion(self.initial_base_q),
        )
        self.current_tool0_q = multiply_quaternions(
            conjugate_quaternion(base_delta),
            self.flat_tool0_q,
        )

    def publish_stabilized_pose(self):
        if self.current_base_q is None:
            return

        now = self.get_clock().now().to_msg()
        x, y, z = self.position
        qx, qy, qz, qw = self.current_tool0_q

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.base_frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)
        self.print_target_if_due(now)

        if self.tf_broadcaster is None:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = self.child_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

    def print_target_if_due(self, stamp):
        if self.print_period_sec is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_print_time).nanoseconds * 1e-9
        if elapsed < self.print_period_sec:
            return

        self.last_print_time = now
        roll, pitch, yaw = quaternion_to_rpy(self.current_tool0_q)
        x, y, z = self.position
        qx, qy, qz, qw = self.current_tool0_q
        self.get_logger().info(
            "New target "
            f"[{self.base_frame} -> {self.child_frame}] "
            f"stamp={stamp.sec}.{stamp.nanosec:09d} "
            f"pos=({x:.5f}, {y:.5f}, {z:.5f}) "
            f"quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}) "
            f"rpy=({roll:.4f}, {pitch:.4f}, {yaw:.4f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuTool0Stabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
