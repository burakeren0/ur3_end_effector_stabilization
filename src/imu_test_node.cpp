#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include <string>

using std::placeholders::_1;

class ImuToEulerTestNode : public rclcpp::Node
{
public:
    ImuToEulerTestNode() : Node("imu_to_euler_test_node")
    {
        // CSV Hazırlığı
        csv_file_.open("imu_angles_data.csv");
        if (csv_file_.is_open()) {
            csv_file_ << "Time_sec,Roll_deg,Pitch_deg,Yaw_deg\n";
        }

        // TOPIC GÜNCELLEMESİ: /imu_data dinleniyor
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_data", 10, std::bind(&ImuToEulerTestNode::imu_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "IMU Test Dugumu baslatildi. '/imu_data' dinleniyor...");
    }

    ~ImuToEulerTestNode() {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV dosyasi basariyla kaydedildi.");
        }
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        static int count = 0;
        if (count++ % 50 == 0) RCLCPP_INFO(this->get_logger(), "Veri aliniyor ve CSV'ye yaziliyor...");

        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double r_deg = roll * 180.0 / M_PI;
        double p_deg = pitch * 180.0 / M_PI;
        double y_deg = yaw * 180.0 / M_PI;
        
        // Zamanı güvenli şekilde saniyeye çevirme
        double time_sec = msg->header.stamp.sec + (msg->header.stamp.nanosec * 1e-9);

        if (csv_file_.is_open()) {
            csv_file_ << time_sec << "," << r_deg << "," << p_deg << "," << y_deg << "\n";
            csv_file_.flush(); // Veriyi anında kaydet
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    std::ofstream csv_file_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToEulerTestNode>());
    rclcpp::shutdown();
    return 0;
}