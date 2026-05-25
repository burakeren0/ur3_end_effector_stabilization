#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <array>

using std::placeholders::_1;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class PIDController : public rclcpp::Node
{
public:
    PIDController()
    : Node("pid_controller_node"), data_received_(false), integral_(Vector6d::Zero())
    {
        Vector6d Kp_diag, Ki_diag, Kd_diag;
        Kp_diag << 120.0, 120.0, 120.0, 45.0, 30.0, 20.0;
        Ki_diag << 5.0, 5.0, 5.0, 1.0, 0.8, 0.5;
        Kd_diag << 25.0, 25.0, 25.0, 12.0, 9.0, 6.0;

        Kp_ = Kp_diag.asDiagonal();
        Ki_ = Ki_diag.asDiagonal();
        Kd_ = Kd_diag.asDiagonal();

        q_d_ << 0.0, -M_PI / 2.0, M_PI / 2.0, -M_PI, -M_PI / 2.0, 0.0;
        q_.setZero();
        dq_.setZero();
        prev_error_.setZero();
        have_joint_state_.fill(false);

        joint_order_ = {
            "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
            "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"
        };

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&PIDController::joint_state_callback, this, _1));

        target_angles_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/target_angles", 10, std::bind(&PIDController::target_angles_callback, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/ur_effort_controller/commands", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&PIDController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "PID Kontrolcüsü Başlatıldı: /ur_effort_controller/commands yayınlanıyor.");
    }

private:
    Vector6d q_, dq_, q_d_, integral_, prev_error_;
    Matrix6d Kp_, Ki_, Kd_;
    bool data_received_;
    std::vector<std::string> joint_order_;
    std::array<bool, 6> have_joint_state_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_angles_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const double dt_sec_ = 0.002;
    const double integral_limit_ = 5.0;
    const double torque_limit_ = 80.0;

    inline double wrap_to_pi(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void target_angles_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 6) {
            for (int i = 0; i < 6; ++i) {
                q_d_(i) = msg->data[i];
            }
        }
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = std::find(joint_order_.begin(), joint_order_.end(), msg->name[i]);
            if (it != joint_order_.end()) {
                int idx = std::distance(joint_order_.begin(), it);
                if (msg->position.size() <= i) { continue; }
                q_(idx) = msg->position[i];
                have_joint_state_[idx] = true;
                if (msg->velocity.size() > i) {
                    dq_(idx) = msg->velocity[i];
                }
            }
        }
        data_received_ = std::all_of(have_joint_state_.begin(), have_joint_state_.end(), [](bool received) {
            return received;
        });
    }

    void control_loop()
    {
        if (!data_received_) {
            return;
        }

        Vector6d error;
        Vector6d derivative;
        for (int i = 0; i < 6; ++i) {
            error(i) = wrap_to_pi(q_d_(i) - q_(i));
            derivative(i) = -dq_(i);
        }

        integral_ += error * dt_sec_;
        for (int i = 0; i < 6; ++i) {
            integral_(i) = std::clamp(integral_(i), -integral_limit_, integral_limit_);
        }

        Vector6d tau = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
        for (int i = 0; i < 6; ++i) {
            tau(i) = std::clamp(tau(i), -torque_limit_, torque_limit_);
        }

        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(tau.data(), tau.data() + tau.size());
        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
