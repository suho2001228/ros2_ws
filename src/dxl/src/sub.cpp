#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "dxl/dxl.hpp"
#include <memory>
#include <functional>

using namespace std::placeholders;

// 콜백 함수 정의
void mysub_callback(rclcpp::Node::SharedPtr node, Dxl &dxl, const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Received message: %lf, %lf", msg->x, msg->y);
    dxl.setVelocity(static_cast<int>(msg->x), static_cast<int>(msg->y));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    Dxl dxl;

    auto node = std::make_shared<rclcpp::Node>("node_dxlsub");

    if (!dxl.open())
    {
        RCLCPP_ERROR(node->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        return -1;
    }

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // 콜백 함수 바인딩
    std::function<void(const geometry_msgs::msg::Vector3::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, std::ref(dxl), _1);

    // 구독자 생성
    auto mysub = node->create_subscription<geometry_msgs::msg::Vector3>(
        "topic_dxlpub", qos_profile, fn);

    rclcpp::spin(node);

    dxl.close();
    rclcpp::shutdown();

    return 0;
}
