#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

// 6x1 ve 6x6 matrisler için Eigen typedef'leri (Sabit boyutlu oldukları için inanılmaz hızlıdır)
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class ComputedTorqueController : public rclcpp::Node
{
public:
    ComputedTorqueController() : Node("computed_torque_node"), data_received_(false)
    {
        // --- 1. KONTROLCÜ AYARLARI ---
        Vector6d Kp_diag, Kv_diag;
        Kp_diag << 15.0, 12.5, 12.5, 12.0, 5.0, 2.0;
        Kp_diag /= 5;  // Tüm elemanları 4'e böler
        
        // Kritik sönüm: Kv = 2 * sqrt(Kp)
        for (int i = 0; i < 6; ++i) {
            Kv_diag(i) =  4.0 * std::sqrt(Kp_diag(i));           
        }

        Kp_ = Kp_diag.asDiagonal();
        Kv_ = Kv_diag.asDiagonal();

        // Hedef Açı (Radyan)
        q_d_ << M_PI/4.0, -M_PI/2.0, 0.0, -M_PI/4.0, M_PI/2.0, 0.0;

        q_.setZero();
        dq_.setZero();

        joint_order_ = {
            "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
            "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"
        };

        // --- 2. ROS ABONELİK VE YAYINLAR ---
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ComputedTorqueController::joint_state_callback, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/ur_effort_controller/commands", 10);

        // 100 Hz Kontrol Döngüsü (10 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&ComputedTorqueController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "C++ Computed Torque Kontrolcüsü Başlatıldı!");
    }

private:
    Vector6d q_, dq_, q_d_;
    Matrix6d Kp_, Kv_;
    bool data_received_;
    std::vector<std::string> joint_order_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Matematiksel kısaltmalar (Numba'daki gibi)
    inline double c(double angle) { return std::cos(angle); }
    inline double s(double angle) { return std::sin(angle); }
    inline double sq(double val) { return val * val; } // Karesini alma kısaltması

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = std::find(joint_order_.begin(), joint_order_.end(), msg->name[i]);
            if (it != joint_order_.end()) {
                int idx = std::distance(joint_order_.begin(), it);
                q_(idx) = msg->position[i];
                if (msg->velocity.size() > i) {
                    dq_(idx) = msg->velocity[i];
                }
            }
        }
        data_received_ = true;
    }

    void robot_dynamics(const Vector6d& q, const Vector6d& dq, Matrix6d& D, Matrix6d& C, Vector6d& g)
    {
        double th1 = q(0), th2 = q(1), th3 = q(2), th4 = q(3), th5 = q(4), th6 = q(5);
        double dth1 = dq(0), dth2 = dq(1), dth3 = dq(2), dth4 = dq(3), dth5 = dq(4), dth6 = dq(5);

        // --- YERÇEKİMİ VEKTÖRÜ (g) ---
        g.setZero();
        g(1) = 0.978*s(th2 + th3 + th4) - 6.354*c(th2 + th3) - 11.905*c(th2) - 0.237*c(th2 + th3 + th4)*s(th5);
        g(2) = 0.978*s(th2 + th3 + th4) - 6.354*c(th2 + th3) - 0.237*c(th2 + th3 + th4)*s(th5);
        g(3) = 0.978*s(th2 + th3 + th4) - 0.237*c(th2 + th3 + th4)*s(th5);
        g(4) = -0.237*s(th2 + th3 + th4)*c(th5);

        // --- INERTIA MATRİSİ D(q) ---
        D.setZero();
        D(0,0) = (0.003*s(th3 + th4 + th5) + 0.003*s(2.0*th2 + th3 + th4 + th5) - 0.003*s(th4 - 1.0*th5) - 0.021*s(2.0*th2 + 2.0*th3 + th4) - 0.003*s(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) - 0.024*s(th3 + th4) + 0.003*s(th4 + th5) - 0.024*s(2.0*th2 + th3 + th4) - 0.003*s(th3 + th4 - 1.0*th5) - 0.003*s(2.0*th2 + th3 + th4 - 1.0*th5) + 0.003*s(2.0*th2 + 2.0*th3 + th4 + th5) - 0.021*s(th4) + 0.126*sq(c(th2 + th3)) + 0.245*sq(c(th2)) + 0.001*sq(c(th5)) - 0.007*sq(c(th2 + th3 + th4)) - 0.002*sq(c(th2 + th3 + th4 - 0.5*th5)) + 0.002*sq(c(th2 + th3 + th4 + 0.5*th5)) + 0.316*sq(c(th2 + 0.5*th3)) + 0.316*sq(c(0.5*th3)) + 0.011*sq(c(0.5*th5)) - 0.241);
        D(0,1) = (0.011*c(th2 + th3 + th4) + 0.003*s(th2 + th3 + th5) + 0.002*c(th2 + th3 + th4 - 1.0*th5) + 0.003*s(th2 - 1.0*th5) + 0.052*s(th2 + th3) + 0.003*s(th2 + th5) + 0.003*s(th2 + th3 - 1.0*th5) + 0.107*s(th2) - 0.013);
        D(0,2) = (0.011*c(th2 + th3 + th4) + 0.003*s(th2 + th3 + th5) + 0.002*c(th2 + th3 + th4 - 1.0*th5) + 0.002*s(th2 - 1.0*th5) + 0.052*s(th2 + th3) + 0.002*s(th2 + th5) + 0.003*s(th2 + th3 - 1.0*th5) + 0.033*s(th2));
        D(0,3) = (0.011*c(th2 + th3 + th4) + 0.001*s(th2 + th3 + th5) + 0.002*c(th2 + th3 + th4 - 1.0*th5) + 0.011*s(th2 + th3) + 0.001*s(th2 + th3 - 1.0*th5));
        D(0,4) = (0.003*s(th2 - 1.0*th5) - 0.003*s(th2 + th3 + th5) - 0.001*c(th2 + th3 + th4) - 0.002*c(th2 + th3 + th4 + th5) - 0.003*s(th2 + th5) + 0.003*s(th2 + th3 - 1.0*th5));

        D(1,0) = D(0,1);
        D(1,1) = (0.006*s(th3 + th4 + th5) - 0.004*c(th2 + th3 + th4) - 0.005*s(th4 - 1.0*th5) - 0.001*c(2.0*th5) - 0.026*s(th2 + th3) - 0.049*s(th3 + th4) + 0.005*s(th4 + th5) - 0.006*s(th3 + th4 - 1.0*th5) + 0.316*c(th3) - 0.049*s(th2) - 0.043*s(th4) + 0.382);
        D(1,2) = (0.005*s(th3 + th4 + th5) - 0.002*c(th2 + th3 + th4) - 0.005*s(th4 - 1.0*th5) - 0.001*c(2.0*th5) - 0.013*s(th2 + th3) - 0.037*s(th3 + th4) + 0.005*s(th4 + th5) - 0.005*s(th3 + th4 - 1.0*th5) + 0.242*c(th3) - 0.009*s(th2) - 0.043*s(th4) + 0.242);
        D(1,3) = (0.003*s(th3 + th4 + th5) - 0.002*c(th2 + th3 + th4) - 0.003*s(th4 - 1.0*th5) - 0.001*c(2.0*th5) - 0.002*s(th2 + th3) - 0.024*s(th3 + th4) + 0.003*s(th4 + th5) - 0.003*s(th3 + th4 - 1.0*th5) + 0.024*c(th3) - 0.026*s(th4) + 0.029);
        D(1,4) = c(th5) * (0.006*s(th3 + th4) + 0.005*s(th4) - 0.002);

        D(2,0) = D(0,2);
        D(2,1) = D(1,2);
        D(2,2) = (0.168*c(th3) - 0.043*s(th4) - 0.026*c(th3)*s(th4) - 0.026*c(th4)*s(th3) + 0.010*c(th4)*s(th5) - 0.001*sq(c(th5)) - 0.006*s(th3)*s(th4)*s(th5) + 0.006*c(th3)*c(th4)*s(th5) + 0.192);
        D(2,3) = (0.013*c(th3) - 0.026*s(th4) - 0.013*c(th3)*s(th4) - 0.013*c(th4)*s(th3) + 0.006*c(th4)*s(th5) - 0.001*sq(c(th5)) - 0.003*s(th3)*s(th4)*s(th5) + 0.003*c(th3)*c(th4)*s(th5) + 0.030);
        D(2,4) = c(th5) * (0.003*s(th3 + th4) + 0.005*s(th4) - 0.002);

        D(3,0) = D(0,3);
        D(3,1) = D(1,3);
        D(3,2) = D(2,3);
        D(3,3) = (0.001*sq(s(th5)) - 0.010*s(th4) - 0.002*s(th5)*(2.0*sq(s(0.5*th4)) - 1.0) + 0.013);
        D(3,4) = c(th5) * (0.001*s(th4) - 0.002);

        D(4,0) = D(0,4);
        D(4,1) = D(1,4);
        D(4,2) = D(2,4);
        D(4,3) = D(3,4);
        D(4,4) = 0.001;
        D(5,5) = 0.0; // Stabilite için eklenebilir

        // --- CORIOLIS MATRİSİ C(q,dq) ---
        C.setZero();
        C(0,0) = (dth5*(0.001*c(th3 + th4 + th5) - 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) + 0.001*c(2.0*th2 + th3 + th4 + th5) + 0.001*c(th4 - 1.0*th5) + 0.001*c(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) - 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.001*c(th4 + th5) + 0.001*c(th3 + th4 - 1.0*th5) + 0.001*c(2.0*th2 + th3 + th4 - 1.0*th5) + 0.001*c(2.0*th2 + 2.0*th3 + th4 + th5) - 0.003*s(th5)) 
                - dth2*(0.158*s(2.0*th2 + th3) - 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 0.003*c(2.0*th2 + th3 + th4 + th5) - 0.004*s(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.021*c(2.0*th2 + 2.0*th3 + th4) + 0.003*c(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.122*s(2.0*th2) + 0.024*c(2.0*th2 + th3 + th4) + 0.003*c(2.0*th2 + th3 + th4 - 1.0*th5) - 0.003*c(2.0*th2 + 2.0*th3 + th4 + th5) + 0.063*s(2.0*th2 + 2.0*th3)) 
                - dth3*(0.079*s(2.0*th2 + th3) - 0.001*c(th3 + th4 + th5) - 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 0.001*c(2.0*th2 + th3 + th4 + th5) - 0.004*s(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.021*c(2.0*th2 + 2.0*th3 + th4) + 0.003*c(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.012*c(th3 + th4) + 0.012*c(2.0*th2 + th3 + th4) + 0.001*c(th3 + th4 - 1.0*th5) + 0.001*c(2.0*th2 + th3 + th4 - 1.0*th5) - 0.003*c(2.0*th2 + 2.0*th3 + th4 + th5) + 0.079*s(th3) + 0.063*s(2.0*th2 + 2.0*th3)) 
                - dth4*(0.001*c(th4 - 1.0*th5) - 0.001*c(th3 + th4 + th5) - 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 0.001*c(2.0*th2 + th3 + th4 + th5) - 0.004*s(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.011*c(2.0*th2 + 2.0*th3 + th4) + 0.001*c(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.012*c(th3 + th4) - 0.001*c(th4 + th5) + 0.012*c(2.0*th2 + th3 + th4) + 0.001*c(th3 + th4 - 1.0*th5) + 0.001*c(2.0*th2 + th3 + th4 - 1.0*th5) - 0.001*c(2.0*th2 + 2.0*th3 + th4 + th5) + 0.011*c(th4)));
        C(0,1) = (dth2*(0.003*c(th2 + th3 + th5) - 0.011*s(th2 + th3 + th4) - 0.002*s(th2 + th3 + th4 - 1.0*th5) + 0.003*c(th2 - 1.0*th5) + 0.052*c(th2 + th3) + 0.003*c(th2 + th5) + 0.003*c(th2 + th3 - 1.0*th5) + 0.107*c(th2)) 
                - dth1*(0.158*s(2.0*th2 + th3) - 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 0.003*c(2.0*th2 + th3 + th4 + th5) - 0.004*s(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.021*c(2.0*th2 + 2.0*th3 + th4) + 0.003*c(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.122*s(2.0*th2) + 0.024*c(2.0*th2 + th3 + th4) + 0.003*c(2.0*th2 + th3 + th4 - 1.0*th5) - 0.003*c(2.0*th2 + 2.0*th3 + th4 + th5) + 0.063*s(2.0*th2 + 2.0*th3)) 
                - dth4*(0.011*s(th2 + th3 + th4) + 0.002*s(th2 + th3 + th4 - 1.0*th5) - 0.005*c(th2 + th3)) 
                - dth3*(0.011*s(th2 + th3 + th4) + 0.002*s(th2 + th3 + th4 - 1.0*th5) - 0.001*c(th2 - 1.0*th5) - 0.052*c(th2 + th3) - 0.001*c(th2 + th5) - 0.003*c(th2 + th3 - 1.0*th5) - 0.016*c(th2)) 
                + dth5*(0.001*s(th2 + th3 + th4) + 0.001*s(th2 + th3 + th4 - 1.0*th5) + 0.001*s(th2 + th3 + th4 + th5)));
        C(0,2) = (dth5*(0.001*s(th2 + th3 + th4) + 0.001*s(th2 + th3 + th4 - 1.0*th5) - 0.001*c(th2 - 1.0*th5) + 0.001*s(th2 + th3 + th4 + th5) + 0.001*c(th2 + th5)) 
                - dth1*(0.079*s(2.0*th2 + th3) - 0.001*c(th3 + th4 + th5) - 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 0.001*c(2.0*th2 + th3 + th4 + th5) - 0.004*s(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.021*c(2.0*th2 + 2.0*th3 + th4) + 0.003*c(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.012*c(th3 + th4) + 0.012*c(2.0*th2 + th3 + th4) + 0.001*c(th3 + th4 - 1.0*th5) + 0.001*c(2.0*th2 + th3 + th4 - 1.0*th5) - 0.003*c(2.0*th2 + 2.0*th3 + th4 + th5) + 0.079*s(th3) + 0.063*s(2.0*th2 + 2.0*th3)) 
                - dth4*(0.011*s(th2 + th3 + th4) + 0.002*s(th2 + th3 + th4 - 1.0*th5) - 0.005*c(th2 + th3)) 
                - dth2*(0.011*s(th2 + th3 + th4) + 0.002*s(th2 + th3 + th4 - 1.0*th5) - 0.001*c(th2 - 1.0*th5) - 0.052*c(th2 + th3) - 0.001*c(th2 + th5) - 0.003*c(th2 + th3 - 1.0*th5) - 0.016*c(th2)) 
                - dth3*(0.011*s(th2 + th3 + th4) + 0.002*s(th2 + th3 + th4 - 1.0*th5) - 0.052*c(th2 + th3) - 0.003*c(th2 + th3 - 1.0*th5)));
        C(0,3) = (dth5*(0.001*s(th2 + th3 + th4) + 0.001*s(th2 + th3 + th4 - 1.0*th5) + 0.001*s(th2 + th3 + th4 + th5)) 
                - dth4*(0.011*s(th2 + th3 + th4) + 0.002*s(th2 + th3 + th4 - 1.0*th5)) 
                - dth1*(0.001*c(th4 - 1.0*th5) - 0.001*c(th3 + th4 + th5) - 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) - 0.001*c(2.0*th2 + th3 + th4 + th5) - 0.004*s(2.0*th2 + 2.0*th3 + 2.0*th4) + 0.011*c(2.0*th2 + 2.0*th3 + th4) + 0.001*c(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) + 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.012*c(th3 + th4) - 0.001*c(th4 + th5) + 0.012*c(2.0*th2 + th3 + th4) + 0.001*c(th3 + th4 - 1.0*th5) + 0.001*c(2.0*th2 + th3 + th4 - 1.0*th5) - 0.001*c(2.0*th2 + 2.0*th3 + th4 + th5) + 0.011*c(th4)) 
                - dth2*(0.011*s(th2 + th3 + th4) + 0.002*s(th2 + th3 + th4 - 1.0*th5) - 0.005*c(th2 + th3)) 
                - dth3*(0.011*s(th2 + th3 + th4) + 0.002*s(th2 + th3 + th4 - 1.0*th5) - 0.005*c(th2 + th3)));
        C(0,4) = (dth1*(0.001*c(th3 + th4 + th5) - 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 - 1.0*th5) + 0.001*c(2.0*th2 + th3 + th4 + th5) + 0.001*c(th4 - 1.0*th5) + 0.001*c(2.0*th2 + 2.0*th3 + th4 - 1.0*th5) - 0.001*s(2.0*th2 + 2.0*th3 + 2.0*th4 + th5) + 0.001*c(th4 + th5) + 0.001*c(th3 + th4 - 1.0*th5) + 0.001*c(2.0*th2 + th3 + th4 - 1.0*th5) + 0.001*c(2.0*th2 + 2.0*th3 + th4 + th5) - 0.003*s(th5)) 
                - dth5*(0.003*c(th2 + th3 + th5) + 0.003*c(th2 - 1.0*th5) - 0.002*s(th2 + th3 + th4 + th5) + 0.003*c(th2 + th5) + 0.003*c(th2 + th3 - 1.0*th5)) 
                + dth4*(0.001*s(th2 + th3 + th4) + 0.001*s(th2 + th3 + th4 - 1.0*th5) + 0.001*s(th2 + th3 + th4 + th5)) 
                + dth3*(0.001*s(th2 + th3 + th4) + 0.001*s(th2 + th3 + th4 - 1.0*th5) - 0.001*c(th2 - 1.0*th5) + 0.001*s(th2 + th3 + th4 + th5) + 0.001*c(th2 + th5)) 
                + dth2*(0.001*s(th2 + th3 + th4) + 0.001*s(th2 + th3 + th4 - 1.0*th5) + 0.001*s(th2 + th3 + th4 + th5)));

        C(1,0) = -C(0,1);
        C(1,1) = (dth5*(0.003*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.003*c(th3 + th4 - 1.0*th5)) 
                - dth3*(0.002*s(th2 + th3 + th4) - 0.003*c(th3 + th4 + th5) + 0.013*c(th2 + th3) + 0.024*c(th3 + th4) + 0.003*c(th3 + th4 - 1.0*th5) + 0.158*s(th3)) 
                - dth2*(0.002*s(th2 + th3 + th4) + 0.013*c(th2 + th3) + 0.024*c(th2)) 
                - dth4*(0.003*c(th4 - 1.0*th5) - 0.002*s(th2 + th3 + th4) - 0.003*c(th3 + th4 + th5) + 0.024*c(th3 + th4) - 0.003*c(th4 + th5) + 0.003*c(th3 + th4 - 1.0*th5) + 0.021*c(th4)));
        C(1,2) = (dth5*(0.004*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.004*c(th3 + th4 - 1.0*th5)) 
                - dth3*(0.002*s(th2 + th3 + th4) - 0.005*c(th3 + th4 + th5) + 0.013*c(th2 + th3) + 0.037*c(th3 + th4) + 0.005*c(th3 + th4 - 1.0*th5) + 0.242*s(th3)) 
                - dth2*(0.002*s(th2 + th3 + th4) - 0.003*c(th3 + th4 + th5) + 0.013*c(th2 + th3) + 0.024*c(th3 + th4) + 0.003*c(th3 + th4 - 1.0*th5) + 0.158*s(th3)) 
                - dth4*(0.003*c(th4 - 1.0*th5) - 0.002*s(th2 + th3 + th4) - 0.004*c(th3 + th4 + th5) + 0.001*c(th2 + th3) + 0.031*c(th3 + th4) - 0.003*c(th4 + th5) + 0.004*c(th3 + th4 - 1.0*th5) + 0.021*c(th4) + 0.012*s(th3)) 
                - dth1*(0.001*c(th2 - 1.0*th5) + 0.001*c(th2 + th5) + 0.016*c(th2)));
        C(1,3) = (dth5*(0.003*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.003*c(th3 + th4 - 1.0*th5)) 
                - dth4*(0.003*c(th4 - 1.0*th5) - 0.002*s(th2 + th3 + th4) - 0.003*c(th3 + th4 + th5) + 0.024*c(th3 + th4) - 0.003*c(th4 + th5) + 0.003*c(th3 + th4 - 1.0*th5) + 0.026*c(th4)) 
                - dth1*(0.005*c(th2 + th3)) 
                - dth2*(0.003*c(th4 - 1.0*th5) - 0.002*s(th2 + th3 + th4) - 0.003*c(th3 + th4 + th5) + 0.024*c(th3 + th4) - 0.003*c(th4 + th5) + 0.003*c(th3 + th4 - 1.0*th5) + 0.021*c(th4)) 
                - dth3*(0.003*c(th4 - 1.0*th5) - 0.002*s(th2 + th3 + th4) - 0.004*c(th3 + th4 + th5) + 0.001*c(th2 + th3) + 0.031*c(th3 + th4) - 0.003*c(th4 + th5) + 0.004*c(th3 + th4 - 1.0*th5) + 0.021*c(th4) + 0.012*s(th3)));
        C(1,4) = (dth5*(0.003*c(th3 + th4 + th5) - 0.003*c(th4 - 1.0*th5) + 0.003*c(th4 + th5) - 0.003*c(th3 + th4 - 1.0*th5) + 0.002*s(th5)) 
                + dth1*(0.003*c(th2 + th3 + th5) + 0.001*s(th2 + th3 + th4 - 1.0*th5) - 0.003*c(th2 - 1.0*th5) - 0.001*s(th2 + th3 + th4 + th5) + 0.003*c(th2 + th5) - 0.003*c(th2 + th3 - 1.0*th5)) 
                + dth2*(0.003*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.003*c(th3 + th4 - 1.0*th5)) 
                + dth4*(0.003*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.003*c(th3 + th4 - 1.0*th5)) 
                + dth3*(0.004*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.004*c(th3 + th4 - 1.0*th5)));

        C(2,0) = -C(0,2);
        C(2,1) = (dth2*(0.158*s(th3) - 0.009*c(th2) + 0.024*c(th3)*c(th4) - 0.024*s(th3)*s(th4) + 0.006*c(th3)*s(th4)*s(th5) + 0.006*c(th4)*s(th3)*s(th5)) 
                - dth4*(0.021*c(th4) - 0.012*s(th3) - 0.001*c(th2)*c(th3) + 0.006*c(th3)*c(th4) + 0.001*s(th2)*s(th3) - 0.006*s(th3)*s(th4) + 0.005*s(th4)*s(th5) + 0.002*c(th3)*s(th4)*s(th5) + 0.002*c(th4)*s(th3)*s(th5)) 
                + dth1*(0.016*c(th2) + 0.002*c(th2)*c(th5)) 
                + dth5*(0.001*s(2.0*th5) + 0.005*c(th4)*c(th5) - 0.002*c(th5)*s(th3)*s(th4) + 0.002*c(th3)*c(th4)*c(th5)));
        C(2,2) = (dth5*c(th5)*(0.003*c(th3 + th4) + 0.005*c(th4) + 0.001*s(th5)) 
                - dth3*(0.084*s(th3) + 0.013*c(th3)*c(th4) - 0.013*s(th3)*s(th4) + 0.003*c(th3)*s(th4)*s(th5) + 0.003*c(th4)*s(th3)*s(th5)) 
                - dth4*(0.021*c(th4) + 0.013*c(th3)*c(th4) - 0.013*s(th3)*s(th4) + 0.005*s(th4)*s(th5) + 0.003*c(th3)*s(th4)*s(th5) + 0.003*c(th4)*s(th3)*s(th5)));
        C(2,3) = (dth5*(0.001*s(2.0*th5) + 0.006*c(th4)*c(th5) - 0.003*c(th5)*s(th3)*s(th4) + 0.003*c(th3)*c(th4)*c(th5)) 
                - dth3*(0.021*c(th4) + 0.013*c(th3)*c(th4) - 0.013*s(th3)*s(th4) + 0.005*s(th4)*s(th5) + 0.003*c(th3)*s(th4)*s(th5) + 0.003*c(th4)*s(th3)*s(th5)) 
                - dth4*(0.026*c(th4) + 0.013*c(th3)*c(th4) - 0.013*s(th3)*s(th4) + 0.006*s(th4)*s(th5) + 0.003*c(th3)*s(th4)*s(th5) + 0.003*c(th4)*s(th3)*s(th5)) 
                - dth2*(0.021*c(th4) - 0.012*s(th3) - 0.001*c(th2)*c(th3) + 0.006*c(th3)*c(th4) + 0.001*s(th2)*s(th3) - 0.006*s(th3)*s(th4) + 0.005*s(th4)*s(th5) + 0.002*c(th3)*s(th4)*s(th5) + 0.002*c(th4)*s(th3)*s(th5)) 
                - dth1*(0.005*c(th2)*c(th3) - 0.005*s(th2)*s(th3) - 0.001*c(th5)*s(th2)*s(th3) + 0.001*c(th2)*c(th3)*c(th5)));
        C(2,4) = (dth2*(0.001*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.001*c(th3 + th4 - 1.0*th5)) 
                + dth3*(0.002*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.002*c(th3 + th4 - 1.0*th5)) 
                + dth4*(0.002*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.002*c(th3 + th4 - 1.0*th5)) 
                + dth5*(0.002*c(th3 + th4 + th5) - 0.003*c(th4 - 1.0*th5) + 0.003*c(th4 + th5) - 0.002*c(th3 + th4 - 1.0*th5) + 0.002*s(th5)) 
                + dth1*(0.003*c(th2 + th3 + th5) + 0.001*s(th2 + th3 + th4 - 1.0*th5) - 0.001*c(th2 - 1.0*th5) - 0.001*s(th2 + th3 + th4 + th5) + 0.001*c(th2 + th5) - 0.003*c(th2 + th3 - 1.0*th5)));

        C(3,0) = -C(0,3);
        C(3,1) = (dth2*(0.003*c(th4 - 1.0*th5) - 0.003*c(th3 + th4 + th5) - 0.002*c(th2 + th3) + 0.024*c(th3 + th4) - 0.003*c(th4 + th5) + 0.003*c(th3 + th4 - 1.0*th5) + 0.021*c(th4)) 
                - dth3*(0.001*c(th3 + th4 + th5) - 0.003*c(th4 - 1.0*th5) + 0.001*c(th2 + th3) - 0.006*c(th3 + th4) + 0.003*c(th4 + th5) - 0.001*c(th3 + th4 - 1.0*th5) - 0.021*c(th4) + 0.012*s(th3)) 
                + dth5*(0.001*s(2.0*th5)) 
                + dth1*(0.005*c(th2 + th3)));
        C(3,2) = (dth2*(0.021*c(th4) - 0.012*s(th3) - 0.001*c(th2)*c(th3) + 0.006*c(th3)*c(th4) + 0.001*s(th2)*s(th3) - 0.006*s(th3)*s(th4) + 0.005*s(th4)*s(th5) + 0.002*c(th3)*s(th4)*s(th5) + 0.002*c(th4)*s(th3)*s(th5)) 
                + dth1*(0.005*c(th2)*c(th3) - 0.005*s(th2)*s(th3) - 0.001*c(th5)*s(th2)*s(th3) + 0.001*c(th2)*c(th3)*c(th5)) 
                + dth5*(0.001*s(2.0*th5) + 0.001*c(th4)*c(th5)) 
                + dth3*(0.021*c(th4) - 0.013*s(th3) + 0.005*s(th4)*s(th5)));
        C(3,3) = (dth5*c(th5)*(0.001*c(th4) + 0.001*s(th5)) - dth4*(0.005*c(th4) + 0.002*c(0.5*th4)*s(0.5*th4)*s(th5)));
        C(3,4) = (dth5*(0.001*c(th4 + th5) - 0.001*c(th4 - 1.0*th5) + 0.002*s(th5)) 
                + dth1*(0.001*s(th2 + th3 + th4 - 1.0*th5) - 0.001*s(th2 + th3 + th4 + th5)) 
                + dth2*(0.001*s(2.0*th5)) 
                + dth3*(0.001*s(2.0*th5)) 
                + dth4*(0.001*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.001*c(th4 + th5)));

        C(4,0) = -C(0,4);
        C(4,1) = (- dth2*(0.003*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.003*c(th3 + th4 - 1.0*th5)) 
                - dth3*(0.001*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.001*c(th3 + th4 - 1.0*th5)) 
                - dth1*(0.003*c(th2 + th3 + th5) + 0.001*s(th2 + th3 + th4 - 1.0*th5) - 0.003*c(th2 - 1.0*th5) - 0.001*s(th2 + th3 + th4 + th5) + 0.003*c(th2 + th5) - 0.003*c(th2 + th3 - 1.0*th5)) 
                - dth4*(0.001*s(2.0*th5)));
        C(4,2) = (- dth2*(0.001*c(th3 + th4 + th5) + 0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5) + 0.001*c(th3 + th4 - 1.0*th5)) 
                - dth4*(0.001*s(2.0*th5)) 
                - dth3*(0.003*c(th4 - 1.0*th5) + 0.001*s(2.0*th5) + 0.003*c(th4 + th5)) 
                - dth1*(0.003*c(th2 + th3 + th5) + 0.001*s(th2 + th3 + th4 - 1.0*th5) - 0.001*c(th2 - 1.0*th5) - 0.001*s(th2 + th3 + th4 + th5) + 0.001*c(th2 + th5) - 0.003*c(th2 + th3 - 1.0*th5)));
        C(4,3) = (- dth1*(0.001*s(th2 + th3 + th4 - 1.0*th5) - 0.001*s(th2 + th3 + th4 + th5)) 
                - 0.001*dth4*s(2.0*th5) 
                - dth2*(0.001*s(2.0*th5)) 
                - dth3*(0.001*s(2.0*th5)));
        C(4,4) = 0.0;
    }

    void control_loop()
    {
        if (!data_received_) return;

        // 1. Hataları Hesapla
        Vector6d e = q_d_ - q_;
        Vector6d de = -dq_; 
        
        // 2. Dış Döngü Komutu (u = Kp*e + Kv*de)
        Vector6d u = Kp_ * e + Kv_ * de;

        // 3. Dinamik Matrisleri Çek
        Matrix6d D, C;
        Vector6d g;
        robot_dynamics(q_, dq_, D, C, g);

        // 4. Tork Kuralı: tau = D*u + C*dq + g
        Vector6d tau = D * u + C * dq_ + g;

        // 5. Yayına Gönder
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(tau.data(), tau.data() + tau.size()); // Eigen Vektörü -> std::vector
        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComputedTorqueController>();
    
    // Ctrl+C yakalamak için try/catch bloğu (C++ rclcpp shutdown yönetimi basittir)
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}