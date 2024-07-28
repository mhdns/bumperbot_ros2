#ifndef BUILD_SIMPLE_TF_KINEMATICS_HPP
#define BUILD_SIMPLE_TF_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class SimpleTfKinematics : public rclcpp::Node
{
public:
    SimpleTfKinematics(const std::string &name);

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
};

#endif //BUILD_SIMPLE_TF_KINEMATICS_HPP
