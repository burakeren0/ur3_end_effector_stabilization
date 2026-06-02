#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <array>  // used to track reception of the full UR joint state array

using std::placeholders::_1;

// Eigen typedefs for fixed-size 6x1 and 6x6 matrices (fixed-size Eigen matrices are faster)
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class ComputedTorqueController : public rclcpp::Node
{
public:
    ComputedTorqueController() : Node("computed_torque_node"), data_received_(false)
    {
        // --- 1. CONTROLLER GAINS / SETTINGS ---
        Vector6d Kp_diag, Kv_diag;
        Kp_diag << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
        //Kp_diag /= 5;  // example: scale gains by a constant
        
        // Critical damping choice: Kv = 2 * sqrt(Kp)
        for (int i = 0; i < 6; ++i) {
            Kv_diag(i) =  2.0 * std::sqrt(Kp_diag(i));           
        }

        Kp_ = Kp_diag.asDiagonal();
        Kv_ = Kv_diag.asDiagonal();

        // Desired joint angles (radians)
        q_d_ << 0, -M_PI/2.0, M_PI/2.0, -M_PI, -M_PI/2.0, 0.0;

        q_.setZero();
        dq_.setZero();
        have_joint_state_.fill(false); // Wait for all 6 UR joints to publish before enabling control.

        joint_order_ = {
            "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
            "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"
        };

        // --- 2. ROS SUBSCRIPTIONS AND PUBLISHERS ---
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ComputedTorqueController::joint_state_callback, this, _1));

        target_angles_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/target_angles", 10, std::bind(&ComputedTorqueController::target_angles_callback, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/ur_effort_controller/commands", 10);

        // 500 Hz control loop (2 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&ComputedTorqueController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "C++ Computed Torque Controller started!");
    }

private:
    Vector6d q_, dq_, q_d_;
    Matrix6d Kp_, Kv_;
    bool data_received_;
    std::vector<std::string> joint_order_;
    std::array<bool, 6> have_joint_state_; // Prevent publishing torque commands until a complete joint_states message is available

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_angles_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Math helper short aliases (keeps expressions concise and readable)
    inline double c(double angle) { return std::cos(angle); }
    inline double s(double angle) { return std::sin(angle); }
    inline double sq(double val) { return val * val; } // shorthand for square
    inline double wrap_to_pi(double angle) { // wrap angular error to the shortest path within [-pi, +pi]
        while (angle > M_PI) { angle -= 2.0 * M_PI; } // wrap angles above +pi back into range
        while (angle < -M_PI) { angle += 2.0 * M_PI; } // wrap angles below -pi back into range
        return angle; // returns the wrapped angle suitable for shortest-path error
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
                if (msg->position.size() <= i) { continue; } // safely ignore joint messages that lack position data
                q_(idx) = msg->position[i];
                have_joint_state_[idx] = true; // mark that this UR joint has a valid position reading
                if (msg->velocity.size() > i) {
                    dq_(idx) = msg->velocity[i];
                }
            }
        }
        data_received_ = std::all_of(have_joint_state_.begin(), have_joint_state_.end(), [](bool received) { return received; }); // only enable control loop after all 6 joints have been received
    }

    void robot_dynamics(const Vector6d& q, const Vector6d& dq, Matrix6d& D, Matrix6d& C, Vector6d& g)
    {
        double th2 = q(1), th3 = q(2), th4 = q(3), th5 = q(4); // extract angles used in dynamics expressions
        double th1_dot = dq(0), th2_dot = dq(1), th3_dot = dq(2), th4_dot = dq(3), th5_dot = dq(4), th6_dot = dq(5);

        for(int i=0; i<6; i++) {
            g(i) = 0.0;
            for(int j=0; j<6; j++) {
                D(i, j) = 0.0;
                C(i, j) = 0.0;
            }
        }

        D(0,0) = 0.002949*std::sin(th3 + th4 + th5) - 4.494e-5*std::cos(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) - 4.494e-5*std::cos(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) - 0.001033*std::cos(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) + 0.002949*std::sin(2.0*th2 + th3 + th4 + th5) + 0.1578*std::cos(2.0*th2 + th3) - 0.002581*std::sin(th4 - 1.0*th5) - 0.02126*std::sin(2.0*th2 + 2.0*th3 + th4) + 0.001033*std::cos(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.1224*std::cos(2.0*th2) + 8.989e-5*std::cos(2.0*th5) - 0.002581*std::sin(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) - 0.02429*std::sin(th3 + th4) + 0.002581*std::sin(th4 + th5) - 0.02429*std::sin(2.0*th2 + th3 + th4) - 0.002949*std::sin(th3 + th4 - 1.0*th5) + 0.1578*std::cos(th3) + 0.005439*std::cos(th5) - 0.002949*std::sin(2.0*th2 + th3 + th4 - 1.0*th5) + 0.002581*std::sin(2.0*th2 + 2.0*th3 + th4 + th5) - 0.02126*std::sin(th4) + 0.06319*std::cos(2.0*th2 + 2.0*th3) - 0.003516*std::cos(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.3019;
        D(0,1) = 0.0112*std::cos(th2 + th3 + th4) + 0.002581*std::sin(th2 + th3 + th5) + 0.002393*std::cos(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::cos(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::cos(th2 + th3 + th4 + 2.0*th5) + 0.002949*std::sin(th2 - 1.0*th5) - 0.0003268*std::cos(th2 + th3 + th4 + th5) + 0.05199*std::sin(th2 + th3) + 0.002949*std::sin(th2 + th5) + 0.002581*std::sin(th2 + th3 - 1.0*th5) + 0.1066*std::sin(th2);
        D(0,2) = 0.0112*std::cos(th2 + th3 + th4) + 0.002581*std::sin(th2 + th3 + th5) + 0.002393*std::cos(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::cos(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::cos(th2 + th3 + th4 + 2.0*th5) - 0.0003268*std::cos(th2 + th3 + th4 + th5) + 0.05199*std::sin(th2 + th3) + 0.002581*std::sin(th2 + th3 - 1.0*th5);
        D(0,3)= 0.0112*std::cos(th2 + th3 + th4) + 0.002393*std::cos(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::cos(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::cos(th2 + th3 + th4 + 2.0*th5) - 0.0003268*std::cos(th2 + th3 + th4 + th5);
        D(0,4) = 0.002949*std::sin(th2 - 1.0*th5) - 0.002581*std::sin(th2 + th3 + th5) - 0.0003268*std::cos(th2 + th3 + th4 - 1.0*th5) - 0.002789*std::cos(th2 + th3 + th4) - 0.002393*std::cos(th2 + th3 + th4 + th5) - 0.002949*std::sin(th2 + th5) + 0.002581*std::sin(th2 + th3 - 1.0*th5);
        D(0,5) = 8.95e-5*std::cos(th2 + th3 + th4 + th5) - 8.95e-5*std::cos(th2 + th3 + th4 - 1.0*th5);
        D(1,0) = 0.0112*std::cos(th2 + th3 + th4) + 0.002581*std::sin(th2 + th3 + th5) + 0.002393*std::cos(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::cos(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::cos(th2 + th3 + th4 + 2.0*th5) + 0.002949*std::sin(th2 - 1.0*th5) - 0.0003268*std::cos(th2 + th3 + th4 + th5) + 0.05199*std::sin(th2 + th3) + 0.002949*std::sin(th2 + th5) + 0.002581*std::sin(th2 + th3 - 1.0*th5) + 0.1066*std::sin(th2);
        D(1,1) = 0.3156*std::cos(th3) - 0.04252*std::sin(th4) - 0.04858*std::cos(th3)*std::sin(th4) - 0.04858*std::cos(th4)*std::sin(th3) + 0.01032*std::cos(th4)*std::sin(th5) - 0.0003596*std::pow(std::cos(th5), 2) - 0.0118*std::sin(th3)*std::sin(th4)*std::sin(th5) + 0.0118*std::cos(th3)*std::cos(th4)*std::sin(th5) + 0.3966;
        D(1,2) = 0.1578*std::cos(th3) - 0.04252*std::sin(th4) - 0.02429*std::cos(th3)*std::sin(th4) - 0.02429*std::cos(th4)*std::sin(th3) + 0.01032*std::cos(th4)*std::sin(th5) - 0.0003596*std::pow(std::cos(th5), 2) - 0.005898*std::sin(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th3)*std::cos(th4)*std::sin(th5) + 0.1422;
        D(1,3) = 0.005162*std::cos(th4)*std::sin(th5) - 0.02429*std::cos(th3)*std::sin(th4) - 0.02429*std::cos(th4)*std::sin(th3) - 0.02126*std::sin(th4) - 0.0003596*std::pow(std::cos(th5), 2) - 0.005898*std::sin(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th3)*std::cos(th4)*std::sin(th5) + 0.01225;
        D(1,4) = 1.21e-6*std::cos(th5)*(4873.0*std::sin(th3 + th4) + 4265.0*std::sin(th4) - 1707.0);
        D(1,5) = 0.000179*std::cos(th5);
        D(2,0) = 0.0112*std::cos(th2 + th3 + th4) + 0.002581*std::sin(th2 + th3 + th5) + 0.002393*std::cos(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::cos(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::cos(th2 + th3 + th4 + 2.0*th5) - 0.0003268*std::cos(th2 + th3 + th4 + th5) + 0.05199*std::sin(th2 + th3) + 0.002581*std::sin(th2 + th3 - 1.0*th5);
        D(2,1) = 0.1578*std::cos(th3) - 0.04252*std::sin(th4) - 0.02429*std::cos(th3)*std::sin(th4) - 0.02429*std::cos(th4)*std::sin(th3) + 0.01032*std::cos(th4)*std::sin(th5) - 0.0003596*std::pow(std::cos(th5), 2) - 0.005898*std::sin(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th3)*std::cos(th4)*std::sin(th5) + 0.1422;
        D(2,2) = 0.01032*std::cos(th4)*std::sin(th5) - 0.04252*std::sin(th4) - 0.0003596*std::pow(std::cos(th5), 2) + 0.1422;
        D(2,3) = 0.005162*std::cos(th4)*std::sin(th5) - 0.02126*std::sin(th4) - 0.0003596*std::pow(std::cos(th5), 2) + 0.01225;
        D(2,4) = 1.21e-6*std::cos(th5)*(4265.0*std::sin(th4) - 1707.0);
        D(2,5) = 0.000179*std::cos(th5);
        D(3,0) = 0.0112*std::cos(th2 + th3 + th4) + 0.002393*std::cos(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::cos(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::cos(th2 + th3 + th4 + 2.0*th5) - 0.0003268*std::cos(th2 + th3 + th4 + th5);
        D(3,1) = 0.005162*std::cos(th4)*std::sin(th5) - 0.02429*std::cos(th3)*std::sin(th4) - 0.02429*std::cos(th4)*std::sin(th3) - 0.02126*std::sin(th4) - 0.0003596*std::pow(std::cos(th5), 2) - 0.005898*std::sin(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th3)*std::cos(th4)*std::sin(th5) + 0.01225;
        D(3,2) = 0.005162*std::cos(th4)*std::sin(th5) - 0.02126*std::sin(th4) - 0.0003596*std::pow(std::cos(th5), 2) + 0.01225;
        D(3,3) = 0.0003596*std::pow(std::sin(th5), 2) + 0.01189;
        D(3,4) = -0.002066*std::cos(th5);
        D(3,5) = 0.000179*std::cos(th5);
        D(4,0) = 0.002949*std::sin(th2 - 1.0*th5) - 0.002581*std::sin(th2 + th3 + th5) - 0.0003268*std::cos(th2 + th3 + th4 - 1.0*th5) - 0.002789*std::cos(th2 + th3 + th4) - 0.002393*std::cos(th2 + th3 + th4 + th5) - 0.002949*std::sin(th2 + th5) + 0.002581*std::sin(th2 + th3 - 1.0*th5);
        D(4,1) = 1.21e-6*std::cos(th5)*(4873.0*std::sin(th3 + th4) + 4265.0*std::sin(th4) - 1707.0);
        D(4,2) = 1.21e-6*std::cos(th5)*(4265.0*std::sin(th4) - 1707.0);
        D(4,3) = -0.002066*std::cos(th5);
        D(4,4) = 0.002789;
        D(5,0) = 8.95e-5*std::cos(th2 + th3 + th4 + th5) - 8.95e-5*std::cos(th2 + th3 + th4 - 1.0*th5);
        D(5,1) = 0.000179*std::cos(th5);
        D(5,2) = 0.000179*std::cos(th5);
        D(5,3) = 0.000179*std::cos(th5);
        D(5,5) = 0.000179;

        // C(q, dq) matrix (Coriolis terms)
        C(0,0) = th5_dot*(0.001474*std::cos(th3 + th4 + th5) - 0.0005165*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) + 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 + th5) + 0.00129*std::cos(th4 - 1.0*th5) + 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) - 0.0005165*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) - 8.989e-5*std::sin(2.0*th5) + 0.00129*std::cos(th4 + th5) + 0.001474*std::cos(th3 + th4 - 1.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) + 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) - 0.00272*std::sin(th5)) - 1.0*th2_dot*(0.1578*std::sin(2.0*th2 + th3) - 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) - 0.002949*std::cos(2.0*th2 + th3 + th4 + th5) - 0.003516*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.02126*std::cos(2.0*th2 + 2.0*th3 + th4) + 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.1224*std::sin(2.0*th2) + 0.02429*std::cos(2.0*th2 + th3 + th4) + 0.002949*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) - 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) + 0.06319*std::sin(2.0*th2 + 2.0*th3)) - 1.0*th3_dot*(0.0789*std::sin(2.0*th2 + th3) - 0.001474*std::cos(th3 + th4 + th5) - 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) - 0.001474*std::cos(2.0*th2 + th3 + th4 + th5) - 0.003516*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.02126*std::cos(2.0*th2 + 2.0*th3 + th4) + 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.01215*std::cos(th3 + th4) + 0.01215*std::cos(2.0*th2 + th3 + th4) + 0.001474*std::cos(th3 + th4 - 1.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) - 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) + 0.0789*std::sin(th3) + 0.06319*std::sin(2.0*th2 + 2.0*th3)) - 1.0*th4_dot*(0.00129*std::cos(th4 - 1.0*th5) - 0.001474*std::cos(th3 + th4 + th5) - 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) - 0.001474*std::cos(2.0*th2 + th3 + th4 + th5) - 0.003516*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.01063*std::cos(2.0*th2 + 2.0*th3 + th4) + 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.01215*std::cos(th3 + th4) - 0.00129*std::cos(th4 + th5) + 0.01215*std::cos(2.0*th2 + th3 + th4) + 0.001474*std::cos(th3 + th4 - 1.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) - 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) + 0.01063*std::cos(th4));
        C(0,1) = th3_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.0112*std::sin(th2 + th3 + th4) - 0.002393*std::sin(th2 + th3 + th4 - 1.0*th5) - 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) + 0.0003268*std::sin(th2 + th3 + th4 + th5) + 0.05199*std::cos(th2 + th3) + 0.002581*std::cos(th2 + th3 - 1.0*th5)) + th2_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.0112*std::sin(th2 + th3 + th4) - 0.002393*std::sin(th2 + th3 + th4 - 1.0*th5) - 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) + 0.002949*std::cos(th2 - 1.0*th5) + 0.0003268*std::sin(th2 + th3 + th4 + th5) + 0.05199*std::cos(th2 + th3) + 0.002949*std::cos(th2 + th5) + 0.002581*std::cos(th2 + th3 - 1.0*th5) + 0.1066*std::cos(th2)) - 1.0*th4_dot*(0.0112*std::sin(th2 + th3 + th4) + 0.002393*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.0003268*std::sin(th2 + th3 + th4 + th5)) + th5_dot*(0.001394*std::sin(th2 + th3 + th4) + 0.00136*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) + 0.00136*std::sin(th2 + th3 + th4 + th5)) + th6_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 1.0*th1_dot*(0.1578*std::sin(2.0*th2 + th3) - 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) - 0.002949*std::cos(2.0*th2 + th3 + th4 + th5) - 0.003516*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.02126*std::cos(2.0*th2 + 2.0*th3 + th4) + 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.1224*std::sin(2.0*th2) + 0.02429*std::cos(2.0*th2 + th3 + th4) + 0.002949*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) - 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) + 0.06319*std::sin(2.0*th2 + 2.0*th3));
        C(0,2) = th2_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.0112*std::sin(th2 + th3 + th4) - 0.002393*std::sin(th2 + th3 + th4 - 1.0*th5) - 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) + 0.0003268*std::sin(th2 + th3 + th4 + th5) + 0.05199*std::cos(th2 + th3) + 0.002581*std::cos(th2 + th3 - 1.0*th5)) + th3_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.0112*std::sin(th2 + th3 + th4) - 0.002393*std::sin(th2 + th3 + th4 - 1.0*th5) - 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) + 0.0003268*std::sin(th2 + th3 + th4 + th5) + 0.05199*std::cos(th2 + th3) + 0.002581*std::cos(th2 + th3 - 1.0*th5)) - 1.0*th4_dot*(0.0112*std::sin(th2 + th3 + th4) + 0.002393*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.0003268*std::sin(th2 + th3 + th4 + th5)) + th5_dot*(0.001394*std::sin(th2 + th3 + th4) + 0.00136*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) + 0.00136*std::sin(th2 + th3 + th4 + th5)) + th6_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 1.0*th1_dot*(0.0789*std::sin(2.0*th2 + th3) - 0.001474*std::cos(th3 + th4 + th5) - 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) - 0.001474*std::cos(2.0*th2 + th3 + th4 + th5) - 0.003516*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.02126*std::cos(2.0*th2 + 2.0*th3 + th4) + 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.01215*std::cos(th3 + th4) + 0.01215*std::cos(2.0*th2 + th3 + th4) + 0.001474*std::cos(th3 + th4 - 1.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) - 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) + 0.0789*std::sin(th3) + 0.06319*std::sin(2.0*th2 + 2.0*th3));
        C(0,3) = th5_dot*(0.001394*std::sin(th2 + th3 + th4) + 0.00136*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) + 0.00136*std::sin(th2 + th3 + th4 + th5)) - 1.0*th2_dot*(0.0112*std::sin(th2 + th3 + th4) + 0.002393*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.0003268*std::sin(th2 + th3 + th4 + th5)) - 1.0*th3_dot*(0.0112*std::sin(th2 + th3 + th4) + 0.002393*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.0003268*std::sin(th2 + th3 + th4 + th5)) - 1.0*th4_dot*(0.0112*std::sin(th2 + th3 + th4) + 0.002393*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) - 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.0003268*std::sin(th2 + th3 + th4 + th5)) - 1.0*th1_dot*(0.00129*std::cos(th4 - 1.0*th5) - 0.001474*std::cos(th3 + th4 + th5) - 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) - 0.001474*std::cos(2.0*th2 + th3 + th4 + th5) - 0.003516*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.01063*std::cos(2.0*th2 + 2.0*th3 + th4) + 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.01215*std::cos(th3 + th4) - 0.00129*std::cos(th4 + th5) + 0.01215*std::cos(2.0*th2 + th3 + th4) + 0.001474*std::cos(th3 + th4 - 1.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) - 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) + 0.01063*std::cos(th4)) + th6_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5));
        C(0,4) = th2_dot*(0.001394*std::sin(th2 + th3 + th4) + 0.00136*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) + 0.00136*std::sin(th2 + th3 + th4 + th5)) - 1.0*th5_dot*(0.002581*std::cos(th2 + th3 + th5) + 0.0003268*std::sin(th2 + th3 + th4 - 1.0*th5) + 0.002949*std::cos(th2 - 1.0*th5) - 0.002393*std::sin(th2 + th3 + th4 + th5) + 0.002949*std::cos(th2 + th5) + 0.002581*std::cos(th2 + th3 - 1.0*th5)) + th3_dot*(0.001394*std::sin(th2 + th3 + th4) + 0.00136*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) + 0.00136*std::sin(th2 + th3 + th4 + th5)) + th4_dot*(0.001394*std::sin(th2 + th3 + th4) + 0.00136*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) + 0.00136*std::sin(th2 + th3 + th4 + th5)) - 1.0*th6_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) + 4.475e-5*std::sin(th2 + th3 + th4 + th5)) + th1_dot*(0.001474*std::cos(th3 + th4 + th5) - 0.0005165*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) + 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 + th5) + 0.00129*std::cos(th4 - 1.0*th5) + 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) - 0.0005165*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) - 8.989e-5*std::sin(2.0*th5) + 0.00129*std::cos(th4 + th5) + 0.001474*std::cos(th3 + th4 - 1.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) + 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) - 0.00272*std::sin(th5));
        C(0,5) = th2_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) + th3_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) + th4_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 1.0*th5_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) + 4.475e-5*std::sin(th2 + th3 + th4 + th5));
        C(1,0) = th5_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.001394*std::sin(th2 + th3 + th4) + 0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.002949*std::cos(th2 - 1.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5) + 0.002949*std::cos(th2 + th5) - 0.002581*std::cos(th2 + th3 - 1.0*th5)) - 1.0*th6_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) + th1_dot*(0.1578*std::sin(2.0*th2 + th3) - 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) - 0.002949*std::cos(2.0*th2 + th3 + th4 + th5) - 0.003516*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.02126*std::cos(2.0*th2 + 2.0*th3 + th4) + 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.1224*std::sin(2.0*th2) + 0.02429*std::cos(2.0*th2 + th3 + th4) + 0.002949*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) - 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) + 0.06319*std::sin(2.0*th2 + 2.0*th3));
        C(1,1) = 2.0e-36*th5_dot*std::cos(th5)*(2.949e+33*std::cos(th3 + th4) + 2.581e+33*std::cos(th4) + 1.798e+32*std::sin(th5)) - 1.0*th3_dot*(0.1578*std::sin(th3) + 0.02429*std::cos(th3)*std::cos(th4) - 0.02429*std::sin(th3)*std::sin(th4) + 0.005898*std::cos(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th4)*std::sin(th3)*std::sin(th5)) - 1.0*th4_dot*(0.02126*std::cos(th4) + 0.02429*std::cos(th3)*std::cos(th4) - 0.02429*std::sin(th3)*std::sin(th4) + 0.005162*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th4)*std::sin(th3)*std::sin(th5));
        C(1,2) = th5_dot*(0.002949*std::cos(th3 + th4 + th5) + 0.002581*std::cos(th4 - 1.0*th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5) + 0.002949*std::cos(th3 + th4 - 1.0*th5)) - 1.0*th2_dot*(0.02429*std::cos(th3 + th4) - 0.002949*std::cos(th3 + th4 + th5) + 0.002949*std::cos(th3 + th4 - 1.0*th5) + 0.1578*std::sin(th3)) - 1.0*th3_dot*(0.02429*std::cos(th3 + th4) - 0.002949*std::cos(th3 + th4 + th5) + 0.002949*std::cos(th3 + th4 - 1.0*th5) + 0.1578*std::sin(th3)) - 1.0*th4_dot*(0.002581*std::cos(th4 - 1.0*th5) - 0.002949*std::cos(th3 + th4 + th5) + 0.02429*std::cos(th3 + th4) - 0.002581*std::cos(th4 + th5) + 0.002949*std::cos(th3 + th4 - 1.0*th5) + 0.02126*std::cos(th4));
        C(1,3) = th5_dot*(0.0001798*std::sin(2.0*th5) + 0.005162*std::cos(th4)*std::cos(th5) - 0.005898*std::cos(th5)*std::sin(th3)*std::sin(th4) + 0.005898*std::cos(th3)*std::cos(th4)*std::cos(th5)) - 1.0*th3_dot*(0.02126*std::cos(th4) + 0.02429*std::cos(th3)*std::cos(th4) - 0.02429*std::sin(th3)*std::sin(th4) + 0.005162*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th4)*std::sin(th3)*std::sin(th5)) - 1.0*th4_dot*(0.02126*std::cos(th4) + 0.02429*std::cos(th3)*std::cos(th4) - 0.02429*std::sin(th3)*std::sin(th4) + 0.005162*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th4)*std::sin(th3)*std::sin(th5)) - 1.0*th2_dot*(0.02126*std::cos(th4) + 0.02429*std::cos(th3)*std::cos(th4) - 0.02429*std::sin(th3)*std::sin(th4) + 0.005162*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th4)*std::sin(th3)*std::sin(th5));
        C(1,4) = th1_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.001394*std::sin(th2 + th3 + th4) + 0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.002949*std::cos(th2 - 1.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5) + 0.002949*std::cos(th2 + th5) - 0.002581*std::cos(th2 + th3 - 1.0*th5)) + th2_dot*(0.002949*std::cos(th3 + th4 + th5) + 0.002581*std::cos(th4 - 1.0*th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5) + 0.002949*std::cos(th3 + th4 - 1.0*th5)) + th3_dot*(0.002949*std::cos(th3 + th4 + th5) + 0.002581*std::cos(th4 - 1.0*th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5) + 0.002949*std::cos(th3 + th4 - 1.0*th5)) + th4_dot*(0.002949*std::cos(th3 + th4 + th5) + 0.002581*std::cos(th4 - 1.0*th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5) + 0.002949*std::cos(th3 + th4 - 1.0*th5)) + th5_dot*(0.002949*std::cos(th3 + th4 + th5) - 0.002581*std::cos(th4 - 1.0*th5) + 0.002581*std::cos(th4 + th5) - 0.002949*std::cos(th3 + th4 - 1.0*th5) + 0.002066*std::sin(th5)) - 8.95e-5*th6_dot*std::sin(th5);
        C(1,5) = - 1.0*th1_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 8.95e-5*th5_dot*std::sin(th5);
        C(2,0) = th5_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.001394*std::sin(th2 + th3 + th4) + 0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5) - 0.002581*std::cos(th2 + th3 - 1.0*th5)) - 1.0*th6_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) + th1_dot*(0.0789*std::sin(2.0*th2 + th3) - 0.001474*std::cos(th3 + th4 + th5) - 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) - 0.001474*std::cos(2.0*th2 + th3 + th4 + th5) - 0.003516*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.02126*std::cos(2.0*th2 + 2.0*th3 + th4) + 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.01215*std::cos(th3 + th4) + 0.01215*std::cos(2.0*th2 + th3 + th4) + 0.001474*std::cos(th3 + th4 - 1.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) - 0.002581*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) + 0.0789*std::sin(th3) + 0.06319*std::sin(2.0*th2 + 2.0*th3));
        C(2,1) = th5_dot*(0.0001798*std::sin(2.0*th5) + 0.005162*std::cos(th4)*std::cos(th5) + 2.125e-15*std::cos(th5)*std::sin(th3)*std::sin(th4) - 2.125e-15*std::cos(th3)*std::cos(th4)*std::cos(th5)) - 1.0*th4_dot*(0.02126*std::cos(th4) + 0.005162*std::sin(th4)*std::sin(th5)) + th2_dot*(0.1578*std::sin(th3) + 0.02429*std::cos(th3)*std::cos(th4) - 0.02429*std::sin(th3)*std::sin(th4) + 0.005898*std::cos(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th4)*std::sin(th3)*std::sin(th5));
        C(2,2) = th5_dot*(0.005162*std::cos(th4)*std::cos(th5) + 0.0003596*std::cos(th5)*std::sin(th5)) - 1.0*th4_dot*(0.02126*std::cos(th4) + 0.005162*std::sin(th4)*std::sin(th5));
        C(2,3) = th5_dot*(0.005162*std::cos(th4)*std::cos(th5) + 0.0003596*std::cos(th5)*std::sin(th5)) - 1.0*th2_dot*(0.02126*std::cos(th4) + 0.005162*std::sin(th4)*std::sin(th5)) - 1.0*th3_dot*(0.02126*std::cos(th4) + 0.005162*std::sin(th4)*std::sin(th5)) - 1.0*th4_dot*(0.02126*std::cos(th4) + 0.005162*std::sin(th4)*std::sin(th5));
        C(2,4) = th2_dot*(0.002581*std::cos(th4 - 1.0*th5) - 1.063e-15*std::cos(th3 + th4 + th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5) - 1.063e-15*std::cos(th3 + th4 - 1.0*th5)) + th5_dot*(0.002581*std::cos(th4 + th5) - 0.002581*std::cos(th4 - 1.0*th5) + 0.002066*std::sin(th5)) + th1_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.001394*std::sin(th2 + th3 + th4) + 0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5) - 0.002581*std::cos(th2 + th3 - 1.0*th5)) - 8.95e-5*th6_dot*std::sin(th5) + th3_dot*(0.002581*std::cos(th4 - 1.0*th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5)) + th4_dot*(0.002581*std::cos(th4 - 1.0*th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5));
        C(2,5) = - 1.0*th1_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 8.95e-5*th5_dot*std::sin(th5);
        C(3,0) = th1_dot*(0.00129*std::cos(th4 - 1.0*th5) - 0.001474*std::cos(th3 + th4 + th5) - 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) - 0.001474*std::cos(2.0*th2 + th3 + th4 + th5) - 0.003516*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.01063*std::cos(2.0*th2 + 2.0*th3 + th4) + 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001033*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.01215*std::cos(th3 + th4) - 0.00129*std::cos(th4 + th5) + 0.01215*std::cos(2.0*th2 + th3 + th4) + 0.001474*std::cos(th3 + th4 - 1.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) - 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) + 0.01063*std::cos(th4)) + th5_dot*(0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) - 0.001394*std::sin(th2 + th3 + th4) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5)) - 1.0*th6_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5));
        C(3,1) = th2_dot*(0.02126*std::cos(th4) + 0.02429*std::cos(th3)*std::cos(th4) - 0.02429*std::sin(th3)*std::sin(th4) + 0.005162*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th3)*std::sin(th4)*std::sin(th5) + 0.005898*std::cos(th4)*std::sin(th3)*std::sin(th5)) + th3_dot*(0.02126*std::cos(th4) + 0.005162*std::sin(th4)*std::sin(th5)) + th5_dot*(0.0001798*std::sin(2.0*th5) - 4.623e-15*std::cos(th4)*std::cos(th5) + 2.125e-15*std::cos(th5)*std::sin(th3)*std::sin(th4) - 2.125e-15*std::cos(th3)*std::cos(th4)*std::cos(th5));
        C(3,2) = th2_dot*(0.02126*std::cos(th4) + 0.005162*std::sin(th4)*std::sin(th5)) - 1.0*th5_dot*(4.623e-15*std::cos(th4)*std::cos(th5) - 0.0003596*std::cos(th5)*std::sin(th5)) + th3_dot*(0.02126*std::cos(th4) + 0.005162*std::sin(th4)*std::sin(th5));
        C(3,3) = 0.0001798*th5_dot*std::sin(2.0*th5);
        C(3,4) = th1_dot*(0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) - 0.001394*std::sin(th2 + th3 + th4) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5)) - 1.0*th2_dot*(1.063e-15*std::cos(th3 + th4 + th5) + 2.312e-15*std::cos(th4 - 1.0*th5) - 0.0001798*std::sin(2.0*th5) + 2.312e-15*std::cos(th4 + th5) + 1.063e-15*std::cos(th3 + th4 - 1.0*th5)) + 0.0001798*th4_dot*std::sin(2.0*th5) + 0.002066*th5_dot*std::sin(th5) - 8.95e-5*th6_dot*std::sin(th5) - 1.0*th3_dot*(2.312e-15*std::cos(th4 - 1.0*th5) - 0.0001798*std::sin(2.0*th5) + 2.312e-15*std::cos(th4 + th5));
        C(3,5) = - 1.0*th1_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 8.95e-5*th5_dot*std::sin(th5);
        C(4,0) = th6_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) + 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 1.0*th4_dot*(0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) - 0.001394*std::sin(th2 + th3 + th4) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5)) - 1.0*th2_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.001394*std::sin(th2 + th3 + th4) + 0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.002949*std::cos(th2 - 1.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5) + 0.002949*std::cos(th2 + th5) - 0.002581*std::cos(th2 + th3 - 1.0*th5)) - 1.0*th3_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.001394*std::sin(th2 + th3 + th4) + 0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5) - 0.002581*std::cos(th2 + th3 - 1.0*th5)) - 1.0*th1_dot*(0.001474*std::cos(th3 + th4 + th5) - 0.0005165*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 - 2.0*th5) + 4.494e-5*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + 2.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 + th5) + 0.00129*std::cos(th4 - 1.0*th5) + 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) - 0.0005165*std::sin(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) - 8.989e-5*std::sin(2.0*th5) + 0.00129*std::cos(th4 + th5) + 0.001474*std::cos(th3 + th4 - 1.0*th5) + 0.001474*std::cos(2.0*th2 + th3 + th4 - 1.0*th5) + 0.00129*std::cos(2.0*th2 + 2.0*th3 + th4 + th5) - 0.00272*std::sin(th5));
        C(4,1) = th4_dot*(1.063e-15*std::cos(th3 + th4 + th5) + 2.312e-15*std::cos(th4 - 1.0*th5) - 0.0001798*std::sin(2.0*th5) + 2.312e-15*std::cos(th4 + th5) + 1.063e-15*std::cos(th3 + th4 - 1.0*th5)) - 1.0*th1_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.001394*std::sin(th2 + th3 + th4) + 0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.002949*std::cos(th2 - 1.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5) + 0.002949*std::cos(th2 + th5) - 0.002581*std::cos(th2 + th3 - 1.0*th5)) - 1.0*th3_dot*(0.002581*std::cos(th4 - 1.0*th5) - 1.063e-15*std::cos(th3 + th4 + th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5) - 1.063e-15*std::cos(th3 + th4 - 1.0*th5)) - 1.0*th2_dot*(0.002949*std::cos(th3 + th4 + th5) + 0.002581*std::cos(th4 - 1.0*th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5) + 0.002949*std::cos(th3 + th4 - 1.0*th5)) + 8.95e-5*th6_dot*std::sin(th5);
        C(4,2) = 8.95e-5*th6_dot*std::sin(th5) - 1.0*th1_dot*(0.002581*std::cos(th2 + th3 + th5) - 0.001394*std::sin(th2 + th3 + th4) + 0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5) - 0.002581*std::cos(th2 + th3 - 1.0*th5)) - 1.0*th2_dot*(0.002581*std::cos(th4 - 1.0*th5) - 1.063e-15*std::cos(th3 + th4 + th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5) - 1.063e-15*std::cos(th3 + th4 - 1.0*th5)) + th4_dot*(2.312e-15*std::cos(th4 - 1.0*th5) - 0.0001798*std::sin(2.0*th5) + 2.312e-15*std::cos(th4 + th5)) - 1.0*th3_dot*(0.002581*std::cos(th4 - 1.0*th5) + 0.0001798*std::sin(2.0*th5) + 0.002581*std::cos(th4 + th5));
        C(4,3) = th2_dot*(1.063e-15*std::cos(th3 + th4 + th5) + 2.312e-15*std::cos(th4 - 1.0*th5) - 0.0001798*std::sin(2.0*th5) + 2.312e-15*std::cos(th4 + th5) + 1.063e-15*std::cos(th3 + th4 - 1.0*th5)) - 1.0*th1_dot*(0.001033*std::sin(th2 + th3 + th4 - 1.0*th5) - 0.001394*std::sin(th2 + th3 + th4) + 8.989e-5*std::sin(th2 + th3 + th4 - 2.0*th5) + 8.989e-5*std::sin(th2 + th3 + th4 + 2.0*th5) - 0.001033*std::sin(th2 + th3 + th4 + th5)) - 0.0001798*th4_dot*std::sin(2.0*th5) + 8.95e-5*th6_dot*std::sin(th5) + th3_dot*(2.312e-15*std::cos(th4 - 1.0*th5) - 0.0001798*std::sin(2.0*th5) + 2.312e-15*std::cos(th4 + th5));
        C(4,5) = th1_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) + 4.475e-5*std::sin(th2 + th3 + th4 + th5)) + 8.95e-5*th2_dot*std::sin(th5) + 8.95e-5*th3_dot*std::sin(th5) + 8.95e-5*th4_dot*std::sin(th5);
        C(5,0) = th2_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) + th3_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) + th4_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 1.0*th5_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) + 4.475e-5*std::sin(th2 + th3 + th4 + th5));
        C(5,1) = th1_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 8.95e-5*th5_dot*std::sin(th5);
        C(5,2) = th1_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 8.95e-5*th5_dot*std::sin(th5);
        C(5,3) = th1_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) - 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 8.95e-5*th5_dot*std::sin(th5);
        C(5,4) = - 1.0*th1_dot*(4.475e-5*std::sin(th2 + th3 + th4 - 1.0*th5) + 4.475e-5*std::sin(th2 + th3 + th4 + th5)) - 8.95e-5*th2_dot*std::sin(th5) - 8.95e-5*th3_dot*std::sin(th5) - 8.95e-5*th4_dot*std::sin(th5);

        // g(q) vector (gravity terms)
        g(1) = std::sin(th5)*(0.2375*std::sin(th2 + th3)*std::sin(th4) - 0.2375*std::cos(th2 + th3)*std::cos(th4)) - 11.9*std::cos(th2) - 6.354*std::cos(th2 + th3) + 0.978*std::cos(th2 + th3)*std::sin(th4) + 0.978*std::sin(th2 + th3)*std::cos(th4);
        g(2) = 0.978*std::sin(th2 + th3 + th4) - 6.354*std::cos(th2 + th3) - 0.2375*std::cos(th2 + th3 + th4)*std::sin(th5);
        g(3) = 0.978*std::sin(th2 + th3 + th4) - 0.2375*std::cos(th2 + th3 + th4)*std::sin(th5);
        g(4) = -0.2375*std::sin(th2 + th3 + th4)*std::cos(th5);
}

    void control_loop()
    {
        if (!data_received_) return;

        // 1. Compute errors
        Vector6d e; // joint-space errors (wrapped angular errors, not raw difference)
        for (int i = 0; i < 6; ++i) { // compute shortest-path angular error for each UR joint
            e(i) = wrap_to_pi(q_d_(i) - q_(i)); // prevents 360-degree wrap-around at +/-pi
        }
        Vector6d de = -dq_; 
        
        // 2. Outer-loop command (u = Kp*e + Kv*de)
        Vector6d u = Kp_ * e + Kv_ * de;

        // 3. Compute dynamic matrices
        Matrix6d D, C;
        Vector6d g;
        robot_dynamics(q_, dq_, D, C, g);

        // 4. Torque law: tau = D*u + C*dq + g
        Vector6d tau = D * u + C * dq_ + g;

        // 5. Publish torque command
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(tau.data(), tau.data() + tau.size()); // convert Eigen vector to std::vector<double>
        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComputedTorqueController>();

    // rclcpp spin handles Ctrl+C and orderly shutdown of the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
