#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
using std::placeholders::_1;

void image_callback(cv::Point& main_point, 
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_pub,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
    // Compressed 이미지 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) return;

    int width = frame.cols;
    int height = frame.rows;
    int roi_height = 90;

    // 관심영역 (하단 90픽셀)
    cv::Rect roi(0, std::max(0, height - roi_height), width, roi_height);
    cv::Mat cropped = frame(roi).clone();

    // Grayscale + 밝기 보정 + Threshold
    cv::Mat gray, binary;
    cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);
    gray += cv::Scalar(100) - cv::mean(gray); // 밝기 보정
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);

    // Connected Components 분석
    cv::Mat labels, stats, centroids, output;
    int num = cv::connectedComponentsWithStats(binary, labels, stats, centroids);

    std::vector<cv::Rect> objects;
    std::vector<cv::Point> centers;

    int closest_idx = -1;
    int min_dist = binary.cols;

    // 중심 기준점 (첫 프레임 or 이전 중심점)
    if (main_point.x < 0) {
        main_point = cv::Point(binary.cols / 2, binary.rows - 1);
        // RCLCPP_INFO(rclcpp::get_logger("line_tracer"), "if문 실행");
    }

    for (int i = 1; i < num; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < 100) continue;

        cv::Point center(
            cvRound(centroids.at<double>(i, 0)),
            cvRound(centroids.at<double>(i, 1))
        );

        int dist = cv::norm(center - main_point);
        if (dist <= 140 && dist < min_dist) {
            min_dist = dist;
            closest_idx = static_cast<int>(centers.size());
        }

        cv::Rect obj(
            stats.at<int>(i, cv::CC_STAT_LEFT), 
            stats.at<int>(i, cv::CC_STAT_TOP),
            stats.at<int>(i, cv::CC_STAT_WIDTH),
            stats.at<int>(i, cv::CC_STAT_HEIGHT)
        );

        objects.push_back(obj);
        centers.push_back(center);
    }

    cv::cvtColor(binary, output, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < objects.size(); ++i) {
        cv::Scalar color = (static_cast<int>(i) == closest_idx) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::rectangle(output, objects[i], color, 2);
        cv::circle(output, centers[i], 5, color, -1);
    }

    int error = 0;
    if (closest_idx >= 0) {
        main_point = centers[closest_idx];
        error = (output.cols / 2) - main_point.x;

        std_msgs::msg::Int32 err_msg;
        err_msg.data = error;
        error_pub->publish(err_msg);

        RCLCPP_INFO(rclcpp::get_logger("line_tracer"), "Error: %d", error);
    }
    RCLCPP_INFO(rclcpp::get_logger("line_tracer"), "Point.x : %d, Point.y : %d", main_point.x, main_point.y);

    cv::imshow("Detection ROI", output);
    cv::waitKey(1);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    cv::Point main_point(-1, -1); // 이전 중심점
    auto node = rclcpp::Node::make_shared("line_tracer");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_pub;
    error_pub = node->create_publisher<std_msgs::msg::Int32>("line_tracer/error", qos);

    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(image_callback, main_point, error_pub, _1);
    auto sub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos, fn);

    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
