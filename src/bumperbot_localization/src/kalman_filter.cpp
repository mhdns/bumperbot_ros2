#include "../include/bumperbot_localization/kalman_filter.hpp"

KalmanFilter::KalmanFilter(const std::string & name)
    : Node(name),
      mean_(0.0),
      variance_(1000.0),
      imu_angular_z_(0.0),
      is_first_odom_(true),
      last_angular_z_(0.0),
      motion_(0.0),
      motion_variance_(4.0),
      measurement_variance_(0.5)
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/bumperbot_controller/odom_noisy", 10, std::bind(&KalmanFilter::odomCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&KalmanFilter::imuCallback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom_kalman", 10);
}

void KalmanFilter::odomCallback(const nav_msgs::msg::Odometry::SharedPtr &odom)
{
    kalman_odom_ = *odom;

    if (is_first_odom_) {
        is_first_odom_ = false;
        last_angular_z_ = odom->twist.twist.angular.z;
        mean_ = odom->twist.twist.angular.z;
        return;
    }

    statePrediction();
    measurementUpdate();
}

void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu::SharedPtr &imu)
{
    imu_angular_z_ = imu->angular_velocity.z;
}

void KalmanFilter::statePrediction()
{
    motion_ = imu_angular_z_ - last_angular_z_;
    last_angular_z_ = imu_angular_z_;
    variance_ = variance_ + motion_variance_;
    mean_ = mean_ + motion_ * variance_;
}

void KalmanFilter::measurementUpdate()
{
    mean_ = (measurement_variance_ * mean_ + variance_ * imu_angular_z_) / (variance_ + measurement_variance_);
    variance_ = (variance_ * measurement_variance_) / (variance_ + measurement_variance_);
}

int main(int argc, char **argv)
{
    return 0;
}