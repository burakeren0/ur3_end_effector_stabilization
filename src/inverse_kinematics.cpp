#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double ZERO_THRESH = 1e-8;

int sign(double x)
{
  return (x > 0.0) - (x < 0.0);
}

double clamp(double value, double min_value, double max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

double normalize_angle(double angle)
{
  while (angle > PI - ZERO_THRESH) {
    angle -= 2.0 * PI;
  }
  while (angle <= -PI - ZERO_THRESH) {
    angle += 2.0 * PI;
  }
  if (std::fabs(angle) < ZERO_THRESH) {
    return 0.0;
  }
  return angle;
}

struct Mat4
{
  double m[4][4]{};

  static Mat4 identity()
  {
    Mat4 result;
    for (int i = 0; i < 4; ++i) {
      result.m[i][i] = 1.0;
    }
    return result;
  }

  double * operator[](int row) {return m[row];}
  const double * operator[](int row) const {return m[row];}
};

Mat4 operator*(const Mat4 & a, const Mat4 & b)
{
  Mat4 result;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      for (int k = 0; k < 4; ++k) {
        result[row][col] += a[row][k] * b[k][col];
      }
    }
  }
  return result;
}

struct DHRow
{
  double a;
  double alpha;
  double d;
};

constexpr std::array<DHRow, 6> DH_UR3{{
  {0.0,      PI / 2.0,  0.1519},
  {-0.24365, 0.0,       0.0},
  {-0.21325, 0.0,       0.0},
  {0.0,      PI / 2.0,  0.11235},
  {0.0,     -PI / 2.0,  0.08535},
  {0.0,      0.0,       0.0819},
}};

Mat4 compute_transform_matrix(int joint_index, const std::array<double, 6> & q)
{
  const int i = joint_index - 1;
  const double ct = std::cos(q[i]);
  const double st = std::sin(q[i]);
  const double ca = std::cos(DH_UR3[i].alpha);
  const double sa = std::sin(DH_UR3[i].alpha);
  const double a = DH_UR3[i].a;
  const double d = DH_UR3[i].d;

  Mat4 result = Mat4::identity();
  result[0][0] = ct;
  result[0][1] = -st * ca;
  result[0][2] = st * sa;
  result[0][3] = a * ct;
  result[1][0] = st;
  result[1][1] = ct * ca;
  result[1][2] = -ct * sa;
  result[1][3] = a * st;
  result[2][0] = 0.0;
  result[2][1] = sa;
  result[2][2] = ca;
  result[2][3] = d;
  return result;
}

Mat4 forward_kinematics_tool0(const std::array<double, 6> & q)
{
  Mat4 result = Mat4::identity();
  for (int joint = 1; joint <= 6; ++joint) {
    result = result * compute_transform_matrix(joint, q);
  }
  return result;
}

void quaternion_to_rotation(double qx, double qy, double qz, double qw, double R[3][3])
{
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm > ZERO_THRESH) {
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;
  }

  R[0][0] = 1.0 - 2.0 * (qy * qy + qz * qz);
  R[0][1] = 2.0 * (qx * qy - qz * qw);
  R[0][2] = 2.0 * (qx * qz + qy * qw);
  R[1][0] = 2.0 * (qx * qy + qz * qw);
  R[1][1] = 1.0 - 2.0 * (qx * qx + qz * qz);
  R[1][2] = 2.0 * (qy * qz - qx * qw);
  R[2][0] = 2.0 * (qx * qz - qy * qw);
  R[2][1] = 2.0 * (qy * qz + qx * qw);
  R[2][2] = 1.0 - 2.0 * (qx * qx + qy * qy);
}

void pose_to_ur_solver_format(
  double x, double y, double z,
  double qx, double qy, double qz, double qw,
  double T[16])
{
  double R[3][3];
  quaternion_to_rotation(qx, qy, qz, qw, R);

  // Reorder rotation/position into the flat array layout expected by
  // the UR analytical solver. This mapping matches the ROS
  // ur_base_link_inertia -> ur_tool0 pose convention.
  T[0] = -R[0][2];
  T[1] = R[0][0];
  T[2] = R[0][1];
  T[3] = -x;
  T[4] = -R[1][2];
  T[5] = R[1][0];
  T[6] = R[1][1];
  T[7] = -y;
  T[8] = R[2][2];
  T[9] = -R[2][0];
  T[10] = -R[2][1];
  T[11] = z;
  T[12] = 0.0;
  T[13] = 0.0;
  T[14] = 0.0;
  T[15] = 1.0;
}

int inverse_ur3_tool0(const double * T, double * q_sols, double q6_des)
{
  constexpr double d1 = 0.1519;
  constexpr double a2 = -0.24365;
  constexpr double a3 = -0.21325;
  constexpr double d4 = 0.11235;
  constexpr double d5 = 0.08535;
  constexpr double d6 = 0.0819;

  int num_sols = 0;

  double T02 = -*T; T++;
  double T00 = *T; T++;
  double T01 = *T; T++;
  double T03 = -*T; T++;
  double T12 = -*T; T++;
  double T10 = *T; T++;
  double T11 = *T; T++;
  double T13 = -*T; T++;
  double T22 = *T; T++;
  double T20 = -*T; T++;
  double T21 = -*T; T++;
  double T23 = *T;

  double q1[2]{};
  {
    const double A = d6 * T12 - T13;
    const double B = d6 * T02 - T03;
    const double R = A * A + B * B;
    if (std::fabs(A) < ZERO_THRESH) {
      double div;
      if (std::fabs(std::fabs(d4) - std::fabs(B)) < ZERO_THRESH) {
        div = -sign(d4) * sign(B);
      } else {
        div = -d4 / B;
      }
      double arcsin = std::asin(clamp(div, -1.0, 1.0));
      if (std::fabs(arcsin) < ZERO_THRESH) {
        arcsin = 0.0;
      }
      q1[0] = (arcsin < 0.0) ? arcsin + 2.0 * PI : arcsin;
      q1[1] = PI - arcsin;
    } else if (std::fabs(B) < ZERO_THRESH) {
      double div;
      if (std::fabs(std::fabs(d4) - std::fabs(A)) < ZERO_THRESH) {
        div = sign(d4) * sign(A);
      } else {
        div = d4 / A;
      }
      const double arccos = std::acos(clamp(div, -1.0, 1.0));
      q1[0] = arccos;
      q1[1] = 2.0 * PI - arccos;
    } else if (d4 * d4 > R + ZERO_THRESH) {
      return num_sols;
    } else {
      const double arccos = std::acos(clamp(d4 / std::sqrt(R), -1.0, 1.0));
      const double arctan = std::atan2(-B, A);
      double pos = arccos + arctan;
      double neg = -arccos + arctan;
      if (std::fabs(pos) < ZERO_THRESH) {
        pos = 0.0;
      }
      if (std::fabs(neg) < ZERO_THRESH) {
        neg = 0.0;
      }
      q1[0] = (pos >= 0.0) ? pos : 2.0 * PI + pos;
      q1[1] = (neg >= 0.0) ? neg : 2.0 * PI + neg;
    }
  }

  double q5[2][2]{};
  for (int i = 0; i < 2; ++i) {
    const double numer = T03 * std::sin(q1[i]) - T13 * std::cos(q1[i]) - d4;
    double div;
    if (std::fabs(std::fabs(numer) - std::fabs(d6)) < ZERO_THRESH) {
      div = sign(numer) * sign(d6);
    } else {
      div = numer / d6;
    }
    const double arccos = std::acos(clamp(div, -1.0, 1.0));
    q5[i][0] = arccos;
    q5[i][1] = 2.0 * PI - arccos;
  }

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      const double c1 = std::cos(q1[i]);
      const double s1 = std::sin(q1[i]);
      const double c5 = std::cos(q5[i][j]);
      const double s5 = std::sin(q5[i][j]);

      double q6;
      if (std::fabs(s5) < ZERO_THRESH) {
        q6 = q6_des;
      } else {
        q6 = std::atan2(
          sign(s5) * -(T01 * s1 - T11 * c1),
          sign(s5) * (T00 * s1 - T10 * c1));
        if (std::fabs(q6) < ZERO_THRESH) {
          q6 = 0.0;
        }
        if (q6 < 0.0) {
          q6 += 2.0 * PI;
        }
      }

      const double c6 = std::cos(q6);
      const double s6 = std::sin(q6);
      const double x04x = -s5 * (T02 * c1 + T12 * s1)
        - c5 * (s6 * (T01 * c1 + T11 * s1)
        - c6 * (T00 * c1 + T10 * s1));
      const double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
      const double p13x = d5 * (s6 * (T00 * c1 + T10 * s1)
        + c6 * (T01 * c1 + T11 * s1))
        - d6 * (T02 * c1 + T12 * s1) + T03 * c1 + T13 * s1;
      const double p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6);

      double c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3)
        / (2.0 * a2 * a3);

      if (std::fabs(std::fabs(c3) - 1.0) < ZERO_THRESH) {
        c3 = sign(c3);
      } else if (std::fabs(c3) > 1.0 + ZERO_THRESH) {
        continue;
      } else if (std::fabs(c3) > 1.0) {
        c3 = sign(c3);
      }

      const double arccos = std::acos(clamp(c3, -1.0, 1.0));
      double q3[2] = {arccos, 2.0 * PI - arccos};
      const double denom = a2 * a2 + a3 * a3 + 2.0 * a2 * a3 * c3;
      const double s3 = std::sin(arccos);
      const double A = a2 + a3 * c3;
      const double B = a3 * s3;

      double q2[2];
      q2[0] = std::atan2(
        (A * p13y - B * p13x) / denom,
        (A * p13x + B * p13y) / denom);
      q2[1] = std::atan2(
        (A * p13y + B * p13x) / denom,
        (A * p13x - B * p13y) / denom);

      const double c23_0 = std::cos(q2[0] + q3[0]);
      const double s23_0 = std::sin(q2[0] + q3[0]);
      const double c23_1 = std::cos(q2[1] + q3[1]);
      const double s23_1 = std::sin(q2[1] + q3[1]);

      double q4[2];
      q4[0] = std::atan2(
        c23_0 * x04y - s23_0 * x04x,
        x04x * c23_0 + x04y * s23_0);
      q4[1] = std::atan2(
        c23_1 * x04y - s23_1 * x04x,
        x04x * c23_1 + x04y * s23_1);

      for (int k = 0; k < 2; ++k) {
        if (std::fabs(q2[k]) < ZERO_THRESH) {
          q2[k] = 0.0;
        } else if (q2[k] < 0.0) {
          q2[k] += 2.0 * PI;
        }

        if (std::fabs(q4[k]) < ZERO_THRESH) {
          q4[k] = 0.0;
        } else if (q4[k] < 0.0) {
          q4[k] += 2.0 * PI;
        }

        q_sols[num_sols * 6 + 0] = normalize_angle(q1[i]);
        q_sols[num_sols * 6 + 1] = normalize_angle(q2[k]);
        q_sols[num_sols * 6 + 2] = normalize_angle(q3[k]);
        q_sols[num_sols * 6 + 3] = normalize_angle(q4[k]);
        q_sols[num_sols * 6 + 4] = normalize_angle(q5[i][j]);
        q_sols[num_sols * 6 + 5] = normalize_angle(q6);
        ++num_sols;
      }
    }
  }

  return num_sols;
}

constexpr std::array<double, 6> REFERENCE_Q{{
  0.0,
  -PI / 2.0,
  PI / 2.0,
  -PI,
  -PI / 2.0,
  0.0,
}};

double angular_distance_squared_to_reference(const double * q, const std::array<double, 6> & reference)  // score IK solution against a continuity reference
{
  double score = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double delta = normalize_angle(q[i] - reference[i]);  // square the shortest angular distance to the reference
    score += delta * delta;
  }
  return score;
}

int select_closest_solution(  // Choose the IK solution that best preserves continuity
  const double * q_sols, int num_sols, const std::array<double, 6> & reference)  // use the previous target as the continuity reference
{
  int best_index = -1;
  double best_score = std::numeric_limits<double>::infinity();

  for (int i = 0; i < num_sols; ++i) {
    const double score = angular_distance_squared_to_reference(&q_sols[i * 6], reference);  // scoring uses continuity reference instead of a fixed pose
    if (score < best_score) {
      best_score = score;
      best_index = i;
    }
  }

  return best_index;
}

std::array<double, 6> unwrap_solution_to_reference(  // Convert normalized IK angles to angles near the reference for continuity
  const double * q, const std::array<double, 6> & reference)  // unwrap each joint angle to the equivalent angle closest to the reference
{
  std::array<double, 6> unwrapped{};  // The published targets are adjusted to be near the previous target
  for (int i = 0; i < 6; ++i) {  // pick the 2*pi-equivalent angle closest to the continuity reference
    unwrapped[i] = reference[i] + normalize_angle(q[i] - reference[i]);  // publish the equivalent angle nearest to the reference
  }
  return unwrapped;  // return continuous joint targets
}

}  // namespace

class InverseKinNode : public rclcpp::Node  // Corrects the IK node class spelling.
{
public:
  InverseKinNode()  // Corrects the constructor spelling to match the class name.
  : Node("inverse_kin")  // Corrects the ROS node name spelling.
  {
    this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
    this->declare_parameter<std::string>("target_angles_topic", "/target_angles");
    this->declare_parameter<double>("q6_des", 0.0);

    target_pose_topic_ = this->get_parameter("target_pose_topic").as_string();
    target_angles_topic_ = this->get_parameter("target_angles_topic").as_string();
    q6_des_ = this->get_parameter("q6_des").as_double();

    target_pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      target_pose_topic_, 10, std::bind(&InverseKinNode::target_pose_callback, this, _1));  // Corrects the callback owner type spelling.
    target_angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      target_angles_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "Inverse IK node started. Subscribing: %s, publishing: %s",  // Corrects the startup log spelling.
      target_pose_topic_.c_str(), target_angles_topic_.c_str());
  }

private:
  void target_pose_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 7) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "target_pose must contain [X, Y, Z, Qx, Qy, Qz, Qw], got %zu values.",
        msg->data.size());
      return;
    }

    const double target_x = msg->data[0];
    const double target_y = msg->data[1];
    const double target_z = msg->data[2];
    const double target_qx = msg->data[3];
    const double target_qy = msg->data[4];
    const double target_qz = msg->data[5];
    const double target_qw = msg->data[6];

    double T[16]{};
    pose_to_ur_solver_format(
      target_x, target_y, target_z,
      target_qx, target_qy, target_qz, target_qw,
      T);

    double q_sols[8 * 6]{};
    const int num_sols = inverse_ur3_tool0(T, q_sols, q6_des_);
    if (num_sols <= 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No valid IK solution for target_pose [%.5f, %.5f, %.5f, %.4f, %.4f, %.4f, %.4f].",
        target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw);
      return;
    }

    const std::array<double, 6> & continuity_reference = last_target_ready_ ? last_target_q_ : REFERENCE_Q;  // use initial reference on first message, then the last published target for continuity
    const int best_index = select_closest_solution(q_sols, num_sols, continuity_reference);  // select the IK branch closest to the continuity reference
    if (best_index < 0) {
      return;
    }

    const double * best_q = &q_sols[best_index * 6];
    const std::array<double, 6> unwrapped_q = unwrap_solution_to_reference(best_q, continuity_reference);  // Unwrap +/-pi transitions to keep continuity with the previous published target.

    std_msgs::msg::Float64MultiArray out;
    out.layout.dim.resize(1);
    out.layout.dim[0].label = "ur_target_angles";
    out.layout.dim[0].size = 6;
    out.layout.dim[0].stride = 6;
    out.layout.data_offset = 0;
    out.data = {
      unwrapped_q[0],  // continuous 1st joint target
      unwrapped_q[1],  // continuous 2nd joint target
      unwrapped_q[2],  // continuous 3rd joint target
      unwrapped_q[3],  // continuous 4th joint target
      unwrapped_q[4],  // continuous 5th joint target
      unwrapped_q[5],  // continuous 6th joint target
    };
    target_angles_pub_->publish(out);
    last_target_q_ = unwrapped_q;  // store this published target as continuity reference for the next IK call
    last_target_ready_ = true;  // after the first valid target, use last_target_q_ instead of REFERENCE_Q

    const std::array<double, 6> q = unwrapped_q;  // run FK check against the published continuous target
    const Mat4 fk = forward_kinematics_tool0(q);
    const double position_error = std::sqrt(
      (fk[0][3] - target_x) * (fk[0][3] - target_x) +
      (fk[1][3] - target_y) * (fk[1][3] - target_y) +
      (fk[2][3] - target_z) * (fk[2][3] - target_z));

    RCLCPP_DEBUG(
      this->get_logger(),
      "Published target_angles solution %d/%d, FK position error %.9f m",
      best_index + 1, num_sols, position_error);
  }

  std::string target_pose_topic_;
  std::string target_angles_topic_;
  double q6_des_{0.0};
  std::array<double, 6> last_target_q_{REFERENCE_Q};  // Stores the last published joint-target used for IK solution continuity selection.
  bool last_target_ready_{false};  // When false, REFERENCE_Q is used until the first valid target has been published.

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_angles_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinNode>());  // Corrects the instantiated IK node type spelling.
  rclcpp::shutdown();
  return 0;
}
