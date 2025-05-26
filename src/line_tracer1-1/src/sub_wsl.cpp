#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class LineTracer : public rclcpp::Node
{
public:
    LineTracer() : Node("sub_wsl")
    {
    using std::placeholders::_1;
    // 퍼블리셔와 같은 QoS로 설정 (Best Effort)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos,
        std::bind(&LineTracer::image_callback, this, _1));

    error_pub_ = this->create_publisher<std_msgs::msg::Float32>("/error", 10);

    RCLCPP_INFO(this->get_logger(), "라인 트레이서 노드 시작됨");
    }


private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try {
            RCLCPP_INFO(this->get_logger(), "콜백 함수 시작");
            // 압축 이미지 디코딩
            cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "디코딩된 이미지가 비어 있습니다");
                return;
            }
            
            // 하단 영역만 추출 (ROI)
            int height = frame.rows;
            int width = frame.cols;
            cv::Mat roi = frame(cv::Range(height * 3 / 4, height), cv::Range(0, width));

            // 흑백 변환 + 이진화
            cv::Mat gray, binary;
            cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
            cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV);  // 검은 선 기준

            // 중심 계산
            cv::Moments m = cv::moments(binary, true);
            if (m.m00 > 0) {
                int cx = int(m.m10 / m.m00);
                float error = cx - (width / 2.0f);

                std_msgs::msg::Float32 err_msg;
                err_msg.data = error;
                error_pub_->publish(err_msg);

                RCLCPP_INFO(this->get_logger(), "에러: %.2f", error);

                // 디버깅용 시각화
                cv::line(roi, cv::Point(cx, 0), cv::Point(cx, roi.rows), cv::Scalar(0, 255, 0), 2);
                cv::line(roi, cv::Point(width / 2, 0), cv::Point(width / 2, roi.rows), cv::Scalar(0, 0, 255), 2);
                cv::imshow("ROI", roi);
                cv::waitKey(1);
            } else {
                RCLCPP_WARN(this->get_logger(), "라인을 찾지 못했습니다");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "이미지 처리 중 오류: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineTracer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
