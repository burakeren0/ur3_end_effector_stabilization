#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> // Quaternion dönüşümleri için eklendi
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

// 6x1 ve 6x6 matrisler için Eigen typedef'leri
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class ComputedTorqueController : public rclcpp::Node
{
public:
    ComputedTorqueController() : Node("computed_torque_node"), data_received_(false), imu_received_(false)
    {
        // --- 1. KONTROLCÜ AYARLARI ---
        Vector6d Kp_diag, Kv_diag;
        // Orijinali: Kp_diag << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
        Kp_diag << 100.0, 100.0, 100.0, 400.0, 400.0, 400.0;

        for (int i = 0; i < 6; ++i) {
            Kv_diag(i) =  2.0 * std::sqrt(Kp_diag(i));           
        }

        Kp_ = Kp_diag.asDiagonal();
        Kv_ = Kv_diag.asDiagonal();

        // UR3 DH Parametreleri (a, d, alpha)
        a_ = {0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0};
        d_ = {0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819};
        alpha_ = {M_PI/2.0, 0.0, 0.0, M_PI/2.0, -M_PI/2.0, 0.0};

        // --- BAŞLANGIÇ KONFİGÜRASYONU VE HEDEF BELİRLEME ---
        // Theta 5'i 90 derece (M_PI/2.0) yaparak robotu tekillikten kurtarıyoruz!
        q_d_ << 0.0, -M_PI/2.0, 0.0, -M_PI/2.0, M_PI/2.0, 0.0;
        
        q_.setZero();
        dq_.setZero();

        // Dünyada sabit kalmasını istediğimiz rotasyon matrisini bir kereye mahsus hesaplıyoruz (İleri Kinematik)
        Eigen::Matrix4d T1 = get_dh_matrix(q_d_(0), a_[0], d_[0], alpha_[0]);
        Eigen::Matrix4d T2 = get_dh_matrix(q_d_(1), a_[1], d_[1], alpha_[1]);
        Eigen::Matrix4d T3 = get_dh_matrix(q_d_(2), a_[2], d_[2], alpha_[2]);
        Eigen::Matrix4d T4 = get_dh_matrix(q_d_(3), a_[3], d_[3], alpha_[3]);
        Eigen::Matrix4d T5 = get_dh_matrix(q_d_(4), a_[4], d_[4], alpha_[4]);
        Eigen::Matrix4d T6 = get_dh_matrix(q_d_(5), a_[5], d_[5], alpha_[5]);
        
        Eigen::Matrix4d T_base_EE_init = T1 * T2 * T3 * T4 * T5 * T6;

        // HEDEF DURUŞU KAYDET (R_world_EE_desired_ yerine bunu kullanacağız)
        R_base_EE_init_ = T_base_EE_init.block<3,3>(0,0);
        
        R_world_EE_desired_ = T_base_EE_init.block<3,3>(0,0);
        R_world_base_ = Eigen::Matrix3d::Identity(); // Başlangıçta eğim yok varsayımı

        joint_order_ = {
            "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
            "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"
        };

        // --- 2. ROS ABONELİK VE YAYINLAR ---
        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ComputedTorqueController::joint_state_callback, this, _1));

        // IMU Abonesi (Topic adını sistemindeki IMU topic'ine göre güncelle)
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_data", 10, std::bind(&ComputedTorqueController::imu_callback, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/ur_effort_controller/commands", 10);

        // 100 Hz Kontrol Döngüsü
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ComputedTorqueController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Aktif Oryantasyon Tork Kontrolcüsü Başlatıldı!");
    }

private:
    bool initial_imu_captured_ = false; // IMU sıfırlama bayrağı
    Eigen::Matrix3d R_imu_initial_;     // İlk IMU verisi
    Eigen::Matrix3d R_base_EE_init_;    // Robotun ilk (hedef) duruşu

    Vector6d q_, dq_, q_d_;
    Matrix6d Kp_, Kv_;
    bool data_received_, imu_received_;
    std::vector<std::string> joint_order_;
    
    // DH Dizileri ve Hedef Matrisler
    std::vector<double> a_, d_, alpha_;
    Eigen::Matrix3d R_world_EE_desired_;
    Eigen::Matrix3d R_world_base_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    inline double sq(double val) { return val * val; }

    // En Kısa Açı Mesafesi Hesaplayıcı (Sarmal - Wrap around problemini çözer)
    double shortest_angular_distance(double current, double target) {
        double diff = std::fmod(target - current + M_PI, 2.0 * M_PI);
        if (diff < 0) diff += 2.0 * M_PI;
        return current + (diff - M_PI);
    }

    // Standart DH Matrisi Oluşturucu
    Eigen::Matrix4d get_dh_matrix(double theta, double a, double d, double alpha) {
        Eigen::Matrix4d T;
        T << std::cos(theta), -std::sin(theta)*std::cos(alpha),  std::sin(theta)*std::sin(alpha), a*std::cos(theta),
             std::sin(theta),  std::cos(theta)*std::cos(alpha), -std::cos(theta)*std::sin(alpha), a*std::sin(theta),
             0.0,             std::sin(alpha),                 std::cos(alpha),                 d,
             0.0,             0.0,                             0.0,                             1.0;
        return T;
    }

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

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Eigen::Quaterniond quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        Eigen::Matrix3d R_current = quat.toRotationMatrix();

        // İLK VERİYİ YAKALA VE SİSTEMİ SIFIRLA
        if (!initial_imu_captured_) {
            R_imu_initial_ = R_current;
            initial_imu_captured_ = true;
            RCLCPP_INFO(this->get_logger(), "IMU Baslangic Referansi (Sifir Noktasi) Kaydedildi!");
        }
        R_world_base_ = R_current;
        imu_received_ = true;
    }

    void robot_dynamics(const Vector6d& q, const Vector6d& dq, Matrix6d& D, Matrix6d& C, Vector6d& g)
    {
        // (Senin gönderdiğin D, C, g matris hesaplama denklemleri burada aynı şekilde kalacak)
        // ... BURAYA O UZUN DİNAMİK DENKLEMLERİNİ YAPIŞTIRACAKSIN ...
        double th1 = q(0), th2 = q(1), th3 = q(2), th4 = q(3), th5 = q(4), th6 = q(5);
        double th1_dot = dq(0), th2_dot = dq(1), th3_dot = dq(2), th4_dot = dq(3), th5_dot = dq(4), th6_dot = dq(5);

        for(int i=0; i<6; i++) {
            g(i) = 0.0;
            for(int j=0; j<6; j++) { D(i, j) = 0.0; C(i, j) = 0.0; }
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

        // C(q, dq) Matrisi (Coriolis)
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

        // g(q) Vektörü (Yerçekimi)
        g(1) = std::sin(th5)*(0.2375*std::sin(th2 + th3)*std::sin(th4) - 0.2375*std::cos(th2 + th3)*std::cos(th4)) - 11.9*std::cos(th2) - 6.354*std::cos(th2 + th3) + 0.978*std::cos(th2 + th3)*std::sin(th4) + 0.978*std::sin(th2 + th3)*std::cos(th4);
        g(2) = 0.978*std::sin(th2 + th3 + th4) - 6.354*std::cos(th2 + th3) - 0.2375*std::cos(th2 + th3 + th4)*std::sin(th5);
        g(3) = 0.978*std::sin(th2 + th3 + th4) - 0.2375*std::cos(th2 + th3 + th4)*std::sin(th5);
        g(4) = -0.2375*std::sin(th2 + th3 + th4)*std::cos(th5);
    }

    void control_loop()
    {
        // Eğer IMU sıfırlanmadıysa veya veri gelmediyse bekle
        if (!data_received_ || !initial_imu_captured_) return;

        // --- AKTİF ORYANTASYON TELAFİSİ ALGORİTMASI ---
        if (imu_received_) {
            // ADIM 1: Dünya eksenindeki değişimi hesapla ve tam tersini (inverse) al.
            // Bu sıralama (R_init * R_current^T) IMU'nun montaj açısından bağımsız olarak
            // saf dünya rotasyonunu verir!
            Eigen::Matrix3d R_base_EE_target = R_world_base_.transpose() * R_imu_initial_ * R_base_EE_init_;

            // ADIM 2: İlk 3 eklemin anlık durumu (q_) ile ayrıştırma işlemi
            Eigen::Matrix4d T1 = get_dh_matrix(q_(0), a_[0], d_[0], alpha_[0]);
            Eigen::Matrix4d T2 = get_dh_matrix(q_(1), a_[1], d_[1], alpha_[1]);
            Eigen::Matrix4d T3 = get_dh_matrix(q_(2), a_[2], d_[2], alpha_[2]);
        
            Eigen::Matrix3d R_base_3 = (T1 * T2 * T3).block<3,3>(0,0);
            Eigen::Matrix3d R_3_6_target = R_base_3.transpose() * R_base_EE_target;

            // ADIM 3: Ters Kinematik (IK) Çözümü
            double r13 = R_3_6_target(0,2);
            double r23 = R_3_6_target(1,2);
            double r31 = R_3_6_target(2,0);
            double r32 = R_3_6_target(2,1);
            double r33 = R_3_6_target(2,2);

            double c5 = r33;
            // Float hassasiyeti kayıplarında karekök için negatif koruması: max(0.0, ...)
            // HATA DÜZELTMESİ: İki farklı IK çözümü (Bilek Yukarı ve Aşağı) hesaplanıyor
            double s5_pos = std::sqrt(std::max(0.0, 1.0 - c5*c5));
            double s5_neg = -s5_pos;

            double th5_1 = std::atan2(s5_pos, c5);
            double th5_2 = std::atan2(s5_neg, c5);

            // Mevcut açıya en yakın olan çözümü seç (180 derece atlamaları engeller!)
            double dist1 = std::abs(shortest_angular_distance(q_(4), th5_1) - q_(4));
            double dist2 = std::abs(shortest_angular_distance(q_(4), th5_2) - q_(4));

            double s5_chosen, theta5_new;
            if (dist1 <= dist2) {
                s5_chosen = s5_pos;
                theta5_new = th5_1;
            } else {
                s5_chosen = s5_neg;
                theta5_new = th5_2;
            }

            double theta4_new, theta6_new;
            if (std::abs(s5_chosen) > 1e-6) {
                // s5_chosen işaretini oranlara katarak bölgeyi (quadrant) doğru buluyoruz
                theta6_new = std::atan2(-r32 / s5_chosen, r31 / s5_chosen);
                theta4_new = std::atan2(-r23 / s5_chosen, -r13 / s5_chosen);
            } else {
                // Tekillik (Gimbal Lock) Durumu
                theta6_new = q_(5); // J6'yı sabit tut
                theta4_new = std::atan2(R_3_6_target(1,0), R_3_6_target(0,0)) - theta6_new;
            }

            // Açıları sarmal hatasından (Wrap-around) koruyarak hedefe (q_d_) atama
            q_d_(3) = shortest_angular_distance(q_(3), theta4_new);
            q_d_(4) = shortest_angular_distance(q_(4), theta5_new);
            q_d_(5) = shortest_angular_distance(q_(5), theta6_new);

            // --- EKLENEN DEBUG BÖLÜMÜ ---
            static int log_counter = 0;
            if (log_counter++ % 100 == 0) { // Saniyede 1 kere yazdır
                Eigen::Vector3d euler = R_world_base_.eulerAngles(2, 1, 0); // Yaw, Pitch, Roll
                RCLCPP_INFO(this->get_logger(), "IMU (Roll: %.1f, Pitch: %.1f, Yaw: %.1f) | HEDEF BİLEKLER -> J4: %.1f, J5: %.1f, J6: %.1f",
                            euler[2]*180.0/M_PI, euler[1]*180.0/M_PI, euler[0]*180.0/M_PI,
                            q_d_(3)*180.0/M_PI, q_d_(4)*180.0/M_PI, q_d_(5)*180.0/M_PI);
            }
        }
        // ----------------------------------------------

        // 1. Hataları Hesapla
        Vector6d e = q_d_ - q_;
        Vector6d de = -dq_; // İstenen hız 0 olduğu için direkt -dq
        
        // 2. Dış Döngü Komutu (u = Kp*e + Kv*de)
        Vector6d u = Kp_ * e + Kv_ * de;

        // 3. Dinamik Matrisleri Çek
        Matrix6d D, C;
        Vector6d g;
        robot_dynamics(q_, dq_, D, C, g);

        // 4. Tork Kuralı: tau = D*u + C*dq + g
        Vector6d tau = D * u + C * dq_ + g;

        // --- EKLENEN KRİTİK KISIM: TORK SINIRLAMA (SATURATION) ---
        // UR3 Gerçek Motor Limitleri: Omuzlar 150 Nm, Bilekler 28 Nm
        double max_shoulder_torque = 150.0;
        double max_wrist_torque = 28.0;
        
        for(int i = 0; i < 3; i++) {
            tau(i) = std::clamp(tau(i), -max_shoulder_torque, max_shoulder_torque);
        }
        for(int i = 3; i < 6; i++) {
            tau(i) = std::clamp(tau(i), -max_wrist_torque, max_wrist_torque);
        }
        // ---------------------------------------------------------

        // 5. Yayına Gönder
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(tau.data(), tau.data() + tau.size());
        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComputedTorqueController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
