#include "psub2-2/sub.hpp"
void Sub::subscribe_msg(const geometry_msgs::msg::Vector3::SharedPtr msg) const
{
RCLCPP_INFO(this->get_logger(), "Received message: '%.2lf', '%.2lf', '%.2lf'", msg->x, msg->y, msg->z);
}
Sub::Sub() : Node("mysub")
{
auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("mytopic", qos_profile,
std::bind(&Sub::subscribe_msg, this, _1));
}