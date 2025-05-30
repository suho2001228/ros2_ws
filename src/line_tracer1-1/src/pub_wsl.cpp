#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp" // error 토픽 받고
#include "geometry_msgs/msg/vector3.hpp" // dxl 작동 토픽 보내야됨 
#include <memory>
#include <functional>
// #include "line_tracer1-1/dxl.hpp"
using std::placeholders::_1;

geometry_msgs::msg::Vector3 vel;

void error_callback(rclcpp::Node::SharedPtr node, 
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mypub,
    const std_msgs::msg::Int32::SharedPtr msg){

    int error = msg->data;

    rclcpp::WallRate loop_rate(20.0); // 50ms에 한번씩 실행

    int vel1, vel2;
    int baseSpeed = 100; // 기본 직진 속도 
    float pGain = 0.15; // P 제어 계수 , 반응이 느리면 p값 증가 / 불안정하면 p값 감소 
    int maxSpeed = 200; // 최대 바퀴 속도 

    /* 
    ex) error > 0 인 경우
    에러가 양수이므로 -> 라인이 왼쪽에 있고 -> 오른쪽 바퀴 속도를 올려서 -> 왼쪽으로 회전해야함.
    그렇기 때문에 baseSpeed에서 왼쪽 바퀴는 속도를 줄여아함. vel1 = baseSpeed - pGain * error
    오른쪽 바퀴는 속도를 올려야함 . vel2 = - (baseSpeed + pGain * error)

    ex) error < 0 인 경우
    에러가 음수이므로 -> 라인이 오른쪽에 있고 -> 왼쪽 바퀴 속도를 올려서 -> 오른쪽으로 회전해야함.
    그렇기 떄문에 baseSpeed에서 왼쪽 바퀴는 속도를 올려야함. vel1 = baseSpeed - pGain * error 
    (에러값이 음수이므로 vel1 값은 커짐)
    오른쪽 바퀴는 속도를 줄여야함. vel2 = - (baseSpeed + pGain * error)
    (에러값이 음수이므로 vel2의 절대값은 줄어들음)
    */
    vel1 = baseSpeed - pGain * error;
    vel2 = -(baseSpeed + pGain * error);

    vel1 = std::max(-maxSpeed, std::min(maxSpeed, vel1)); // 최대 바퀴 속도 이상으로 못돌게 설정.
    vel2 = std::max(-maxSpeed, std::min(maxSpeed, vel2));

    vel.x = vel1, vel.y = vel2;
    RCLCPP_INFO(node->get_logger(), "에러 : %d 왼쪽속도 : %d, 오른쪽속도 : %d", error, vel1, vel2);
    mypub->publish(vel);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;

    auto node = std::make_shared<rclcpp::Node>("node_dxlpub");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);

    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(error_callback, node, mypub, _1);
    auto sub = node->create_subscription<std_msgs::msg::Int32>("line_tracer/error", qos_profile, fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}