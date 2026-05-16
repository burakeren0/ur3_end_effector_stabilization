#include <cmath>  // Trigonometric helpers are used for RPY logging.
#include <memory>  // std::make_shared is used in main.
#include <stdexcept>  // Parameter validation throws runtime_error.
#include <string>  // Topic parameters are stored as strings.
#include <vector>  // Vector parameters hold pose values.

#include <rclcpp/rclcpp.hpp>  // ROS 2 node API.
#include <sensor_msgs/msg/imu.hpp>  // IMU orientation input message.
#include <std_msgs/msg/float64_multi_array.hpp>  // Target pose output message.

using std::placeholders::_1;  // std::bind placeholder for the IMU callback.

namespace  // File-local math helpers keep this node self-contained.
{
constexpr double kPi = 3.14159265358979323846;  // Pi constant for angle wrapping.

struct Quaternion  // Minimal Hamilton quaternion storage in [x, y, z, w] order.
{
  double x;  // X vector component.
  double y;  // Y vector component.
  double z;  // Z vector component.
  double w;  // Scalar component.
};

struct Rpy  // Roll, pitch, yaw tuple used only for readable logs.
{
  double roll;  // Rotation around fixed X axis.
  double pitch;  // Rotation around fixed Y axis.
  double yaw;  // Rotation around fixed Z axis.
};

Quaternion normalize_quaternion(const Quaternion & q)  // Keeps quaternion math numerically stable.
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);  // Quaternion length.
  if (norm < 1e-12) {  // Invalid zero-length quaternions fall back to identity.
    return {0.0, 0.0, 0.0, 1.0};  // Identity rotation.
  }

  return {q.x / norm, q.y / norm, q.z / norm, q.w / norm};  // Unit quaternion result.
}

Quaternion conjugate_quaternion(const Quaternion & q)  // Unit-quaternion inverse helper.
{
  return {-q.x, -q.y, -q.z, q.w};  // Hamilton conjugate.
}

Quaternion multiply_quaternion(const Quaternion & a, const Quaternion & b)  // Hamilton product.
{
  return normalize_quaternion({  // Normalize after composition to avoid drift.
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,  // X component.
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,  // Y component.
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,  // Z component.
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,  // W component.
  });
}

double wrap_to_pi(double angle)  // Keeps log angles in [-pi, pi].
{
  while (angle > kPi) {  // Positive overflow.
    angle -= 2.0 * kPi;  // Wrap downward.
  }
  while (angle < -kPi) {  // Negative overflow.
    angle += 2.0 * kPi;  // Wrap upward.
  }
  return angle;  // Wrapped angle.
}

Rpy quaternion_to_rpy(const Quaternion & input_q)  // Converts Hamilton quaternion to fixed-frame RPY.
{
  const Quaternion q = normalize_quaternion(input_q);  // Work with a unit quaternion.

  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);  // Roll numerator.
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);  // Roll denominator.
  const double roll = std::atan2(sinr_cosp, cosr_cosp);  // Fixed-frame roll.

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);  // Pitch sine term.
  double pitch = 0.0;  // Fixed-frame pitch result.
  if (std::abs(sinp) >= 1.0) {  // Gimbal-limit guard.
    pitch = std::copysign(kPi / 2.0, sinp);  // Clamp to +/-90 degrees.
  } else {
    pitch = std::asin(sinp);  // Normal pitch calculation.
  }

  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);  // Yaw numerator.
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);  // Yaw denominator.
  const double yaw = std::atan2(siny_cosp, cosy_cosp);  // Fixed-frame yaw.

  return {wrap_to_pi(roll), wrap_to_pi(pitch), wrap_to_pi(yaw)};  // Wrapped RPY tuple.
}
}  // namespace

class TargetPoseFullRpy : public rclcpp::Node  // Publishes a target pose that compensates roll, pitch, and yaw drift.
{
public:
  TargetPoseFullRpy()  // Constructor declares parameters and wires ROS interfaces.
  : Node("target_pose_full_rpy")  // Separate node name avoids colliding with the existing target_pose node.
  {
    this->declare_parameter<std::string>("imu_topic", "/imu_data");  // IMU input topic.
    this->declare_parameter<std::string>("output_topic", "/target_pose");  // IK input topic.
    this->declare_parameter<std::vector<double>>(  // Fixed target position parameter.
      "target_position", {-0.29515, -0.11235, 0.48090});  // Default tool0 target position.
    this->declare_parameter<std::vector<double>>(  // Initial tool orientation parameter.
      "initial_tool0_quaternion", {0.5, 0.5, -0.5, -0.5});  // Default tool0 quaternion.

    const auto imu_topic = this->get_parameter("imu_topic").as_string();  // Read IMU topic parameter.
    const auto output_topic = this->get_parameter("output_topic").as_string();  // Read output topic parameter.

    const auto target_position =  // Read target position parameter.
      this->get_parameter("target_position").as_double_array();  // Parameter vector access.
    if (target_position.size() != 3) {  // Validate [X, Y, Z].
      throw std::runtime_error("target_position parameter must contain [X, Y, Z]");  // Fail fast on bad configuration.
    }
    target_x_ = target_position[0];  // Store target X.
    target_y_ = target_position[1];  // Store target Y.
    target_z_ = target_position[2];  // Store target Z.

    const auto initial_q_values =  // Read initial tool quaternion parameter.
      this->get_parameter("initial_tool0_quaternion").as_double_array();  // Parameter vector access.
    if (initial_q_values.size() != 4) {  // Validate [Qx, Qy, Qz, Qw].
      throw std::runtime_error(  // Fail fast on bad configuration.
        "initial_tool0_quaternion parameter must contain [Qx, Qy, Qz, Qw]");  // Expected format.
    }

    initial_tool_q_ = normalize_quaternion(  // Store normalized starting tool orientation.
      {initial_q_values[0], initial_q_values[1], initial_q_values[2], initial_q_values[3]});  // [Qx, Qy, Qz, Qw].
    initial_tool_rpy_ = quaternion_to_rpy(initial_tool_q_);  // Cache readable initial RPY.

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(  // Subscribe to IMU orientation.
      imu_topic, rclcpp::SensorDataQoS(), std::bind(&TargetPoseFullRpy::imu_callback, this, _1));  // Sensor QoS.

    target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(  // Publish IK target pose.
      output_topic, 10);  // Small reliable queue.

    RCLCPP_INFO(  // Startup log.
      this->get_logger(),  // Node logger.
      "Target pose full RPY node started. IMU: %s, output: %s, initial RPY: [%.4f, %.4f, %.4f]",  // Log format.
      imu_topic.c_str(), output_topic.c_str(),  // Topic names.
      initial_tool_rpy_.roll, initial_tool_rpy_.pitch, initial_tool_rpy_.yaw);  // Initial tool RPY.
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)  // Handles each IMU orientation sample.
  {
    const Quaternion imu_q = normalize_quaternion(  // Normalize current IMU orientation.
      {msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w});  // ROS quaternion order.

    if (!imu_reference_ready_) {  // First IMU sample becomes the flat/reference attitude.
      imu_reference_q_ = imu_q;  // Store reference IMU orientation.
      imu_reference_ready_ = true;  // Mark reference as ready.
    }

    const Quaternion reference_to_current = multiply_quaternion(  // IMU attitude drift from reference to now.
      conjugate_quaternion(imu_reference_q_), imu_q);  // q_delta = inverse(q_ref) * q_current.
    const Quaternion compensation = conjugate_quaternion(reference_to_current);  // Inverse drift preserves world RPY.
    const Quaternion compensated_q = multiply_quaternion(compensation, initial_tool_q_);  // Apply full roll/pitch/yaw compensation.

    publish_target_pose(compensated_q);  // Send the stabilized target pose to IK.
  }

  void publish_target_pose(const Quaternion & compensated_q)  // Publishes [X, Y, Z, Qx, Qy, Qz, Qw].
  {
    std_msgs::msg::Float64MultiArray out;  // Output message for inverse_kinematics.cpp.
    out.layout.dim.resize(1);  // One-dimensional pose vector.
    out.layout.dim[0].label = "x_y_z_qx_qy_qz_qw";  // Existing layout label.
    out.layout.dim[0].size = 7;  // Seven values are published.
    out.layout.dim[0].stride = 7;  // Single row stride.
    out.layout.data_offset = 0;  // No offset in the data vector.
    out.data = {  // Preserve the existing target_pose message contract.
      target_x_,  // Target X.
      target_y_,  // Target Y.
      target_z_,  // Target Z.
      compensated_q.x,  // Stabilized quaternion X.
      compensated_q.y,  // Stabilized quaternion Y.
      compensated_q.z,  // Stabilized quaternion Z.
      compensated_q.w,  // Stabilized quaternion W.
    };

    target_pub_->publish(out);  // Publish to the IK node.
  }

  double target_x_{-0.29515};  // Target X position.
  double target_y_{-0.11235};  // Target Y position.
  double target_z_{0.48090};  // Target Z position.
  bool imu_reference_ready_{false};  // Indicates whether the reference IMU quaternion is captured.
  Quaternion initial_tool_q_{0.0, 0.0, 0.0, 1.0};  // Desired tool orientation at the reference attitude.
  Quaternion imu_reference_q_{0.0, 0.0, 0.0, 1.0};  // First IMU orientation sample.
  Rpy initial_tool_rpy_{0.0, 0.0, 0.0};  // Readable startup log copy of the desired tool orientation.

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;  // IMU subscription handle.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;  // Target pose publisher handle.
};

int main(int argc, char * argv[])  // ROS 2 executable entry point.
{
  rclcpp::init(argc, argv);  // Initialize ROS 2.
  rclcpp::spin(std::make_shared<TargetPoseFullRpy>());  // Run the full-RPY target pose node.
  rclcpp::shutdown();  // Shutdown ROS 2 cleanly.
  return 0;  // Successful process exit.
}
