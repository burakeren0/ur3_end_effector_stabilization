#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Quaternion
{
  double x;
  double y;
  double z;
  double w;
};

struct Rpy
{
  double roll;
  double pitch;
  double yaw;
};

Quaternion normalize_quaternion(const Quaternion & q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1e-12) {
    return {0.0, 0.0, 0.0, 1.0};
  }

  return {q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

double wrap_to_pi(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

Rpy quaternion_to_rpy(const Quaternion & input_q)
{
  const Quaternion q = normalize_quaternion(input_q);

  // Hamilton quaternion -> fixed-frame roll/pitch/yaw dönüşümü.
  // roll  = X ekseni etrafındaki dönüş
  // pitch = Y ekseni etrafındaki dönüş
  // yaw   = Z ekseni etrafındaki dönüş
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  double pitch = 0.0;
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(kPi / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

Quaternion rpy_to_quaternion(const Rpy & rpy)
{
  // Fixed-frame roll/pitch/yaw -> Hamilton quaternion dönüşümü.
  const double cr = std::cos(rpy.roll * 0.5);
  const double sr = std::sin(rpy.roll * 0.5);
  const double cp = std::cos(rpy.pitch * 0.5);
  const double sp = std::sin(rpy.pitch * 0.5);
  const double cy = std::cos(rpy.yaw * 0.5);
  const double sy = std::sin(rpy.yaw * 0.5);

  return normalize_quaternion({
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  });
}
}  // namespace

class TargetPose : public rclcpp::Node
{
public:
  TargetPose()
  : Node("target_pose")
  {
    this->declare_parameter<std::string>("imu_topic", "/imu_data");
    this->declare_parameter<std::string>("output_topic", "/target_pose");
    this->declare_parameter<std::vector<double>>(
      "target_position", {-0.29515, -0.11235, 0.48090});
    this->declare_parameter<std::vector<double>>(
      "initial_tool0_quaternion", {0.5, 0.5, -0.5, -0.5});

    const auto imu_topic = this->get_parameter("imu_topic").as_string();
    const auto output_topic = this->get_parameter("output_topic").as_string();

    const auto target_position =
      this->get_parameter("target_position").as_double_array();
    if (target_position.size() != 3) {
      throw std::runtime_error("target_position parametresi 3 elemanli olmali: [X, Y, Z]");
    }
    target_x_ = target_position[0];
    target_y_ = target_position[1];
    target_z_ = target_position[2];

    const auto initial_q_values =
      this->get_parameter("initial_tool0_quaternion").as_double_array();
    if (initial_q_values.size() != 4) {
      throw std::runtime_error(
        "initial_tool0_quaternion parametresi 4 elemanli olmali: [Qx, Qy, Qz, Qw]");
    }

    const Quaternion initial_tool_q = normalize_quaternion(
      {initial_q_values[0], initial_q_values[1], initial_q_values[2], initial_q_values[3]});
    initial_tool_rpy_ = quaternion_to_rpy(initial_tool_q);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(), std::bind(&TargetPose::imu_callback, this, _1));

    target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      output_topic, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "Target pose node started. IMU: %s, output: %s, initial RPY: [%.4f, %.4f, %.4f]",
      imu_topic.c_str(), output_topic.c_str(),
      initial_tool_rpy_.roll, initial_tool_rpy_.pitch, initial_tool_rpy_.yaw);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const Quaternion imu_q = normalize_quaternion(
      {msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w});
    const Rpy imu_rpy = quaternion_to_rpy(imu_q);

    if (!imu_reference_ready_) {
      imu_reference_rpy_ = imu_rpy;
      imu_reference_ready_ = true;
    }

    // IMU'nun ilk ölçümü "düz/başlangıç" referansı kabul edilir.
    // Arazideki anlık gövde sapması:
    const double imu_roll_delta = wrap_to_pi(imu_rpy.roll - imu_reference_rpy_.roll);
    const double imu_pitch_delta = wrap_to_pi(imu_rpy.pitch - imu_reference_rpy_.pitch);

    // Kompanzasyon mantığı:
    // 1) IMU Y ekseni etrafında dönerse bu pitch sapmasıdır.
    //    End-effector bunu -X ekseni yönünde telafi etsin:
    //    X ekseni Euler karşılığı roll olduğundan roll hedefinden pitch sapmasını çıkarıyoruz.
    const double compensated_roll = wrap_to_pi(initial_tool_rpy_.roll - imu_pitch_delta);

    // 2) IMU X ekseni etrafında dönerse bu roll sapmasıdır.
    //    End-effector bunu Z ekseni yönündeki açıyla telafi etsin:
    //    Z ekseni Euler karşılığı yaw olduğundan yaw hedefinden roll sapmasını çıkarıyoruz.
    const double compensated_yaw = wrap_to_pi(initial_tool_rpy_.yaw - imu_roll_delta);

    // İstenen kurala göre pitch hedefini başlangıç değerinde sabit tutuyoruz.
    const double compensated_pitch = initial_tool_rpy_.pitch;
    const Quaternion compensated_q = rpy_to_quaternion(
      {compensated_roll, compensated_pitch, compensated_yaw});

    std_msgs::msg::Float64MultiArray out;
    out.layout.dim.resize(1);
    out.layout.dim[0].label = "x_y_z_qx_qy_qz_qw";
    out.layout.dim[0].size = 7;
    out.layout.dim[0].stride = 7;
    out.layout.data_offset = 0;
    out.data = {
      target_x_,
      target_y_,
      target_z_,
      compensated_q.x,
      compensated_q.y,
      compensated_q.z,
      compensated_q.w,
    };

    target_pub_->publish(out);
  }

  double target_x_{-0.29515};
  double target_y_{-0.11235};
  double target_z_{0.48090};
  bool imu_reference_ready_{false};
  Rpy initial_tool_rpy_{0.0, 0.0, 0.0};
  Rpy imu_reference_rpy_{0.0, 0.0, 0.0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetPose>());
  rclcpp::shutdown();
  return 0;
}