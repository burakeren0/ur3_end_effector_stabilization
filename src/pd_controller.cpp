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

// PDController node
// - Subscribes to: `/joint_states` (sensor_msgs/JointState) and `/target_angles` (Float64MultiArray)
// - Publishes to: `/ur_effort_controller/commands` (Float64MultiArray) containing 6 joint torques
// - Implements a simple PD law in joint space with angle wrapping to handle discontinuities at +/-pi
// - The controller runs at a fixed timer period (`dt_sec_`) and clamps torques to `torque_limit_`.

class PDController : public rclcpp::Node
{
public:
    PDController()
    : Node("pd_controller_node"), data_received_(false)
    {
        Vector6d Kp_diag, Kd_diag;
        Kp_diag << 120.0, 120.0, 120.0, 45.0, 30.0, 20.0;
        Kd_diag << 25.0, 25.0, 25.0, 12.0, 9.0, 6.0;

        Kp_ = Kp_diag.asDiagonal();
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
            "/joint_states", 10, std::bind(&PDController::joint_state_callback, this, _1));

        target_angles_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/target_angles", 10, std::bind(&PDController::target_angles_callback, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/ur_effort_controller/commands", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&PDController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "PD Controller started: publishing to /ur_effort_controller/commands");
    }

private:
    Vector6d q_, dq_, q_d_, prev_error_;
    Matrix6d Kp_, Kd_;
    bool data_received_;
    std::vector<std::string> joint_order_;
    std::array<bool, 6> have_joint_state_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_angles_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const double dt_sec_ = 0.002;
    const double torque_limit_ = 80.0;

    // Wrap an angle to the range [-pi, +pi]. This prevents the controller
    // from producing large error values when the target crosses the +/-pi boundary.
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

        // Compute joint-space error (wrapped) and derivative term
        Vector6d error;
        Vector6d derivative;
        for (int i = 0; i < 6; ++i) {
            // shortest-path angular error
            error(i) = wrap_to_pi(q_d_(i) - q_(i));
            // derivative approximated as negative measured velocity (target velocity assumed zero)
            derivative(i) = -dq_(i);
        }

        // PD law: tau = Kp * error + Kd * derivative
        Vector6d tau = Kp_ * error + Kd_ * derivative;
        // clamp torques to protect actuators/sim
        for (int i = 0; i < 6; ++i) {
            tau(i) = std::clamp(tau(i), -torque_limit_, torque_limit_);
        }

        // Publish the torque vector as a Float64MultiArray (6 elements)
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(tau.data(), tau.data() + tau.size());
        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
