#include "psub2-2/pub.hpp"
Pub::Pub() : Node("mypub"), count_(0)
{
auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("mytopic", qos_profile);
timer_ = this->create_wall_timer(1s, std::bind(&Pub::publish_msg, this));
}
int cnt = 0;
void Pub::publish_msg()
{
auto msg = geometry_msgs::msg::Vector3();
msg.x = cnt;
msg.y = cnt;
msg.z = cnt++;
RCLCPP_INFO(this->get_logger(), "Published message: '%.2lf', '%.2lf', '%.2lf' ", msg.x, msg.y, msg.z);
pub_->publish(msg);
}