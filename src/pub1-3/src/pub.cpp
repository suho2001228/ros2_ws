#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <chrono>
#include <iostream>
int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
auto node = std::make_shared<rclcpp::Node>("node_pub1_3");
auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
auto mypub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",
qos_profile);
geometry_msgs::msg::Twist message;
rclcpp::WallRate loop_rate(1.0);
char c;
while(rclcpp::ok())
{
std::cout << "전진f 후진b 좌회전l 우회전r : ";
std::cin >> c;
if(c=='f'){
    message.linear.x = 2.0;
}
else if(c=='b'){
    message.linear.x = -2.0;
}
else if(c=='l'){
    message.angular.z = 2.0;
}
else if(c=='r'){
    message.angular.z = -2.0;
}
RCLCPP_INFO(node->get_logger(),"%c",c);
mypub->publish(message);
//rclcpp::spin_some(node);
loop_rate.sleep(); //반복주파수에서 남은 시간 만큼 sleep
}
rclcpp::shutdown();
return 0;
}