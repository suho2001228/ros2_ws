#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <memory>
#include <chrono>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("video_file_pub");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto mypub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);

    // 동영상 경로 (WSL 내부 경로로 수정하세요)
    std::string video_path = "/home/linux/ros2_ws/src/line_tracer1-1/7_lt_ccw_100rpm_in.mp4";

    // 동영상 열기
    cv::VideoCapture cap(video_path, cv::CAP_FFMPEG);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "비디오 파일을 열 수 없습니다: %s", video_path.c_str());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::WallRate loop_rate(30.0);  // 대략 30fps
    cv::Mat frame;

    while (rclcpp::ok()) {
        cap >> frame;
        if (frame.empty()) {
            RCLCPP_INFO(node->get_logger(), "영상 끝. 처음으로 재시작합니다.");
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            continue;
        }

        cv::imshow("Video Publisher", frame);
        if (cv::waitKey(1) == 27) break;  // ESC 누르면 종료

        std_msgs::msg::Header hdr;
        hdr.stamp = node->now();
        hdr.frame_id = "camera";

        auto msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        mypub->publish(*msg);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
