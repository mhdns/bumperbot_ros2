#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>


class ImuRepublisher : public rclcpp::Node
{
public:
    ImuRepublisher(const std::string & name) : Node(name)
    {
        rclcpp::sleep_for(std::chrono::seconds (1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/out", 10, std::bind(&ImuRepublisher::imuCallback, this, std::placeholders::_1));
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu_ekf", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    void imuCallback(const sensor_msgs::msg::Imu & imu)
    {
        sensor_msgs::msg::Imu new_imu;
        new_imu = imu;
        new_imu.header.frame_id = "base_footprint_ekf";
        imu_pub_->publish(imu);
    }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuRepublisher>("imu_republisher"));
    rclcpp::shutdown();
    return 0;
}