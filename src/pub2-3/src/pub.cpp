#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <chrono>
#include <functional>
#include <iostream>
using namespace std::chrono_literals;

void callback(rclcpp::Node::SharedPtr node,
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
{
	auto message = geometry_msgs::msg::Twist();

    std::cout << "선속도 (linear) x y 값 입력: ";
    std::cin >> message.linear.x >> message.linear.y;

    std::cout << "각속도 (angular) z 값 입력: ";
    std::cin >> message.angular.z;

    RCLCPP_INFO(node->get_logger(),
                "선속도: x=%.2f, y=%.2f\n 각속도도: z=%.2f",
                message.linear.x, message.linear.y, message.angular.z);

    pub->publish(message);
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("mynode");
	auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
	// /turtle1/cmd_vel로 메세지 전송
	auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile);
	std::function<void()> fn = std::bind(callback, node, pub);
	auto timer = node->create_wall_timer(500ms, fn);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

