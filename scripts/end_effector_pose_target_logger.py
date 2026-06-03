#!/usr/bin/env python3
# Log UR3 end-effector target/actual position and orientation for MATLAB plots.

import argparse  # Command-line options configure topics, frames, output path, and rate.
import csv  # CSV writer stores one target/actual sample per row.
import math  # Trigonometric helpers convert quaternions to roll-pitch-yaw angles.
import sys  # ROS and custom command-line arguments are read from sys.argv.
import time  # Monotonic time throttles repeated TF startup warnings.
from pathlib import Path  # Path keeps output-file handling platform independent.

import rclpy  # ROS 2 Python client library.
from rclpy.executors import ExternalShutdownException  # Jazzy raises this when launch shuts the process down.
from rclpy.node import Node  # Node base class for subscriptions, timers, and logging.
from rclpy.qos import qos_profile_sensor_data  # Match the best-effort QoS used by the Gazebo IMU bridge.
from rclpy.time import Time  # Zero-valued Time asks TF for the latest available transform.
from rclpy.utilities import remove_ros_args  # Keep argparse from rejecting ROS-specific command-line options.
from sensor_msgs.msg import Imu  # /imu_data provides the UR base orientation relative to the Gazebo world.
from std_msgs.msg import Float64MultiArray  # /target_pose carries [x, y, z, qx, qy, qz, qw].
from tf2_ros import Buffer, TransformException  # TF buffer stores transforms and raises lookup errors.
from tf2_ros.transform_listener import TransformListener  # Transform listener fills the TF buffer.


def normalize_quaternion(q):  # Return a unit quaternion in (x, y, z, w) order.
    x, y, z, w = q  # Split the tuple so the normalization math stays readable.
    norm = math.sqrt(x * x + y * y + z * z + w * w)  # Compute quaternion length.
    if norm < 1e-12:  # Guard against invalid all-zero quaternion messages.
        return (0.0, 0.0, 0.0, 1.0)  # Fall back to identity orientation.
    return (x / norm, y / norm, z / norm, w / norm)  # Return normalized components.


def multiply_quaternion(a, b):  # Compose orientations with the Hamilton product a * b.
    ax, ay, az, aw = a  # Unpack the first quaternion.
    bx, by, bz, bw = b  # Unpack the second quaternion.
    return normalize_quaternion((  # Normalize the result to avoid numerical drift.
        aw * bx + ax * bw + ay * bz - az * by,  # X component of the composed quaternion.
        aw * by - ax * bz + ay * bw + az * bx,  # Y component of the composed quaternion.
        aw * bz + ax * by - ay * bx + az * bw,  # Z component of the composed quaternion.
        aw * bw - ax * bx - ay * by - az * bz,  # W component of the composed quaternion.
    ))  # Close the normalized Hamilton-product tuple.


def quaternion_from_transform(transform_msg):  # Extract a normalized quaternion from a TF transform message.
    rotation = transform_msg.transform.rotation  # TF rotation is stored as geometry_msgs/Quaternion.
    return normalize_quaternion((rotation.x, rotation.y, rotation.z, rotation.w))  # Return (x, y, z, w).


def euler_from_quaternion(q):  # Convert quaternion (x, y, z, w) to fixed-frame roll, pitch, yaw.
    x, y, z, w = normalize_quaternion(q)  # Normalize before applying Euler formulas.
    sinr_cosp = 2.0 * (w * x + y * z)  # Roll numerator term.
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)  # Roll denominator term.
    roll = math.atan2(sinr_cosp, cosr_cosp)  # Roll angle around the fixed X axis.
    sinp = 2.0 * (w * y - z * x)  # Pitch sine term.
    sinp = max(-1.0, min(1.0, sinp))  # Clamp for numerical safety near +/-90 degrees.
    pitch = math.asin(sinp)  # Pitch angle around the fixed Y axis.
    siny_cosp = 2.0 * (w * z + x * y)  # Yaw numerator term.
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)  # Yaw denominator term.
    yaw = math.atan2(siny_cosp, cosy_cosp)  # Yaw angle around the fixed Z axis.
    return roll, pitch, yaw  # Return angles in radians.


def resolve_output_path(output_file):  # Resolve relative CSV paths in a repo-friendly way.
    raw_path = Path(output_file).expanduser()  # Expand a possible leading "~" in the path.
    if raw_path.is_absolute():  # Absolute paths should be used exactly as requested.
        return raw_path  # Return the absolute output path.
    package_root = Path(__file__).resolve().parents[1]  # Source-tree scripts live one level below the package root.
    if (package_root / "package.xml").exists():  # Detect direct execution from the source checkout.
        return package_root / raw_path  # Keep the default CSV under this package's analysis directory.
    return Path.cwd() / raw_path  # Installed execution falls back to the shell working directory.


class EndEffectorPoseTargetLogger(Node):  # ROS node that joins TF actuals with /target_pose targets.
    def __init__(self, args):  # Configure subscriptions, TF listener, timer, and CSV output.
        super().__init__("end_effector_pose_target_logger")  # Give the logger a unique ROS node name.
        self.target_pose_topic = args.target_pose_topic  # Store the target pose topic name.
        self.imu_topic = args.imu_topic  # Store the base IMU topic used for world-relative orientation.
        self.base_frame = args.base_frame  # Store the UR3 base frame used for x, y, z positions.
        self.ee_frame = args.ee_frame  # Store the UR3 end-effector frame name.
        self.sample_period = 1.0 / args.rate  # Convert logging frequency to timer period.
        self.latest_target_position = None  # Last target [x, y, z] in the UR base frame.
        self.latest_target_quaternion_base = None  # Last target orientation in the UR base frame.
        self.latest_base_quaternion_world = None  # Last IMU orientation of the UR base relative to Gazebo world.
        self.last_tf_warning_time = 0.0  # Track the last TF warning so startup logs stay readable.
        self.output_path = resolve_output_path(args.output_file)  # Resolve the CSV output path.
        self.output_path.parent.mkdir(parents=True, exist_ok=True)  # Create the output directory if needed.
        self.csv_file = open(self.output_path, "w", newline="", encoding="utf-8")  # Open CSV for deterministic UTF-8 output.
        self.csv_writer = csv.writer(self.csv_file)  # Create a standard CSV writer.
        self.csv_writer.writerow([  # Write columns that map directly to the six MATLAB plots.
            "time",  # Sample time from this ROS node's clock.
            "target_x_base",  # Target end-effector X position relative to the UR base frame.
            "actual_x_base",  # Actual end-effector X position relative to the UR base frame.
            "target_y_base",  # Target end-effector Y position relative to the UR base frame.
            "actual_y_base",  # Actual end-effector Y position relative to the UR base frame.
            "target_z_base",  # Target end-effector Z position relative to the UR base frame.
            "actual_z_base",  # Actual end-effector Z position relative to the UR base frame.
            "target_roll_world_rad",  # Target end-effector roll angle relative to the world frame.
            "actual_roll_world_rad",  # Actual end-effector roll angle relative to the world frame.
            "target_pitch_world_rad",  # Target end-effector pitch angle relative to the world frame.
            "actual_pitch_world_rad",  # Actual end-effector pitch angle relative to the world frame.
            "target_yaw_world_rad",  # Target end-effector yaw angle relative to the world frame.
            "actual_yaw_world_rad",  # Actual end-effector yaw angle relative to the world frame.
        ])  # Finish writing the CSV header.
        self.tf_buffer = Buffer()  # Allocate the transform buffer.
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Start listening to /tf and /tf_static.
        self.target_pose_sub = self.create_subscription(Float64MultiArray, self.target_pose_topic, self.target_pose_callback, 10)  # Subscribe to target pose commands.
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)  # Subscribe to base orientation relative to Gazebo world.
        self.timer = self.create_timer(self.sample_period, self.timer_callback)  # Periodically sample target and actual values.
        self.get_logger().info(f"Logging end-effector pose target/actual CSV to: {self.output_path}")  # Report output path.
        self.get_logger().info(f"Orientation source: {self.imu_topic}; frames: base={self.base_frame}, end_effector={self.ee_frame}")  # Report orientation and frame configuration.

    def destroy_node(self):  # Close the CSV cleanly when the process shuts down.
        self.csv_file.close()  # Flush and close the output file.
        super().destroy_node()  # Let rclpy clean up subscriptions and timers.

    def target_pose_callback(self, msg):  # Store the latest [x, y, z, qx, qy, qz, qw] target pose.
        if len(msg.data) < 7:  # Ignore malformed target-pose messages.
            self.get_logger().warning(f"target_pose must contain 7 values, got {len(msg.data)}")  # Explain the bad message.
            return  # Wait for the next valid target message.
        self.latest_target_position = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))  # Save target XYZ in base frame.
        self.latest_target_quaternion_base = normalize_quaternion((float(msg.data[3]), float(msg.data[4]), float(msg.data[5]), float(msg.data[6])))  # Save target orientation in base frame.

    def imu_callback(self, msg):  # Store the UR base orientation measured relative to Gazebo world.
        self.latest_base_quaternion_world = normalize_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))  # Save world-to-base orientation from the base-mounted IMU.

    def timer_callback(self):  # Write one CSV row whenever both target and TF actual data are available.
        if self.latest_target_position is None or self.latest_target_quaternion_base is None or self.latest_base_quaternion_world is None:  # Do not log before target pose and IMU orientation arrive.
            return  # Keep waiting for /target_pose and /imu_data.
        try:  # TF lookups can fail until the simulator and robot_state_publisher are ready.
            ee_in_base = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, Time())  # Actual EE pose in UR base frame.
        except TransformException as exc:  # Missing TF data is expected during startup.
            now_monotonic = time.monotonic()  # Read a clock unaffected by simulation-time resets.
            if now_monotonic - self.last_tf_warning_time >= 5.0:  # Limit repeated TF warnings to once every five seconds.
                self.get_logger().warning(f"TF lookup failed, waiting for frames: {exc}")  # Show the exact missing transform.
                self.last_tf_warning_time = now_monotonic  # Remember when this warning was emitted.
            return  # Skip this sample and try again at the next timer tick.
        target_x, target_y, target_z = self.latest_target_position  # Unpack target position in base frame.
        actual_translation = ee_in_base.transform.translation  # Translation of EE origin expressed in the base frame.
        actual_x = actual_translation.x  # Actual X position in base frame.
        actual_y = actual_translation.y  # Actual Y position in base frame.
        actual_z = actual_translation.z  # Actual Z position in base frame.
        actual_quaternion_base = quaternion_from_transform(ee_in_base)  # Read actual EE orientation relative to the UR base frame.
        target_quaternion_world = multiply_quaternion(self.latest_base_quaternion_world, self.latest_target_quaternion_base)  # Convert target orientation from base frame to Gazebo world.
        actual_quaternion_world = multiply_quaternion(self.latest_base_quaternion_world, actual_quaternion_base)  # Convert actual orientation from base frame to Gazebo world.
        target_roll, target_pitch, target_yaw = euler_from_quaternion(target_quaternion_world)  # Convert target world quaternion to RPY.
        actual_roll, actual_pitch, actual_yaw = euler_from_quaternion(actual_quaternion_world)  # Convert actual world quaternion to RPY.
        time_s = self.get_clock().now().nanoseconds * 1e-9  # Use ROS time when use_sim_time is enabled, otherwise system time.
        self.csv_writer.writerow([  # Persist the current target/actual pose sample.
            f"{time_s:.9f}",  # Time column with nanosecond-level formatting.
            f"{target_x:.9f}",  # Target X in meters.
            f"{actual_x:.9f}",  # Actual X in meters.
            f"{target_y:.9f}",  # Target Y in meters.
            f"{actual_y:.9f}",  # Actual Y in meters.
            f"{target_z:.9f}",  # Target Z in meters.
            f"{actual_z:.9f}",  # Actual Z in meters.
            f"{target_roll:.9f}",  # Target roll in radians.
            f"{actual_roll:.9f}",  # Actual roll in radians.
            f"{target_pitch:.9f}",  # Target pitch in radians.
            f"{actual_pitch:.9f}",  # Actual pitch in radians.
            f"{target_yaw:.9f}",  # Target yaw in radians.
            f"{actual_yaw:.9f}",  # Actual yaw in radians.
        ])  # Finish writing one CSV row.
        self.csv_file.flush()  # Keep data visible even if the simulation is interrupted.


def main():  # Parse arguments and run the logger node.
    parser = argparse.ArgumentParser(description="Log UR3 end-effector target/actual pose for MATLAB plotting.")  # Build the CLI parser.
    parser.add_argument("--target-pose-topic", default="/target_pose", help="Topic containing [x, y, z, qx, qy, qz, qw].")  # Target pose topic option.
    parser.add_argument("--imu-topic", default="/imu_data", help="Base IMU topic providing orientation relative to Gazebo world.")  # World-orientation source option.
    parser.add_argument("--base-frame", default="ur_base_link_inertia", help="Frame used for x, y, z position plots.")  # UR base frame option.
    parser.add_argument("--ee-frame", default="ur_tool0", help="End-effector frame to measure.")  # End-effector frame option.
    parser.add_argument("--output-file", default="analysis/end_effector_pose_target_log.csv", help="CSV output path.")  # Output file option.
    parser.add_argument("--rate", type=float, default=20.0, help="Logging rate in Hz.")  # Sample-rate option.
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])  # Parse only non-ROS CLI arguments.
    if args.rate <= 0.0:  # Timer periods must be positive.
        parser.error("--rate must be greater than zero")  # Stop early for invalid rates.
    rclpy.init(args=sys.argv)  # Initialize ROS 2 with parameters and remappings supplied by launch.
    node = EndEffectorPoseTargetLogger(args)  # Create the logger node.
    try:  # Keep spinning until the user stops the process.
        rclpy.spin(node)  # Process subscriptions, TF listener callbacks, and timer callbacks.
    except KeyboardInterrupt:  # Ctrl+C is the normal interactive shutdown path.
        if rclpy.ok():  # Log only while the ROS context can still publish to rosout.
            node.get_logger().info("Shutdown requested")  # Report clean shutdown.
    except ExternalShutdownException:  # Launch can stop the ROS context before spin returns.
        pass  # Avoid publishing a log message through an already-closed ROS context.
    finally:  # Always release ROS and file resources.
        node.destroy_node()  # Close CSV and destroy ROS entities.
        if rclpy.ok():  # Avoid shutting down a context that launch already stopped.
            rclpy.shutdown()  # Shutdown ROS 2 when the context is still active.


if __name__ == "__main__":  # Run main only when executed as a script.
    main()  # Start the logger.
