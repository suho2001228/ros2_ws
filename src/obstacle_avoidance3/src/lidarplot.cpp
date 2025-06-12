#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <math.h>
#include <opencv2/opencv.hpp>
#define RAD2DEG(x) ((x)*180. / M_PI) // 라디안 -> 도
using std::placeholders::_1;
// 가장 가까운 영역 바운딩 박스 그리고 각도 구해주는 함수
double drawAndGetAngle(cv::Mat& frame, const cv::Rect& rect,
                       const cv::Point& mid_point, const cv::Scalar& color,
                       bool is_left, double& rect_y) {
    // 사각형 아래 y 좌표
    rect_y = rect.y + rect.height;

    // 중심점 y와 같으면 충돌 방지용 보정
    if (rect_y == mid_point.y) {
        rect_y -= rect.height;
    }

    // 기준점 계산 (왼쪽은 오른쪽 하단, 오른쪽은 왼쪽 하단 사용)
    cv::Point target_point;
    if (is_left) {
        target_point = cv::Point(rect.x + rect.width, rect_y);
    } else {
        target_point = cv::Point(rect.x, rect_y);
    }

    // 방향 벡터 계산 및 각도
    double angle_rad = std::atan2(target_point.x - mid_point.x, mid_point.y - target_point.y);

    // 시각화
    cv::rectangle(frame, rect, color, 2);
    cv::arrowedLine(frame, mid_point, target_point, color, 2);

    return angle_rad;
}

// 두 벡터 합친 각도 구하는 함수 / 매개변수에 길이를 넣으면 길이값이 벡터에 반영됨.
double midAngle(double left_angle_rad, double right_angle_rad,
                double left_len = 1, double right_len = 1) {
    double vx1 = std::cos(left_angle_rad);
    double vy1 = std::sin(left_angle_rad);
    double vx2 = std::cos(right_angle_rad);
    double vy2 = std::sin(right_angle_rad);

    double w1 = left_len;
    double w2 = right_len;

    // 가중 벡터 평균
    double vx = (vx1 * w1 + vx2 * w2) / (w1 + w2);
    double vy = (vy1 * w1 + vy2 * w2) / (w1 + w2);

    return std::atan2(vy, vx); 
}

// 각도로 계산해서 바퀴 속도 구하는 함수
geometry_msgs::msg::Vector3 speeds(double mid_angle_rad,
                                   double base_speed = 50.0, double p = 50.0) {
    double angle_deg = mid_angle_rad / 1.57;
    double left_speed = base_speed + angle_deg * p;
    double right_speed = -base_speed + angle_deg * p;

    geometry_msgs::msg::Vector3 vel;
    vel.x = std::round(left_speed);
    vel.y = std::round(right_speed);
    vel.z = 0.0;
    return vel;
}

static void scanCb(rclcpp::Node::SharedPtr node,
                   rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mypub,
                   geometry_msgs::msg::Vector3 vel,
                   sensor_msgs::msg::LaserScan::SharedPtr scan) {

    static bool speed_stop = false;

    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    int count = scan->scan_time / scan->time_increment;
    for (int i = 0; i < count; i++) {
        float r = scan->angle_min + scan->angle_increment * i;

        int x = cvRound(250 + 250.0 * scan->ranges[i] * sin(r));
        int y = cvRound(250 + 250.0 * scan->ranges[i] * cos(r));
        if (x > 500 || y > 500 || x < 0 || y < 0) continue;
        // 그 위치에 반지름이 1인 원 그리기
        cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0,0,255), -1);
    }

    cv::Rect roi(0, 0, img.cols, img.rows / 2);
    cv::Point mid_point(img.cols / 2, img.rows / 2);

    cv::Mat gray, binary, closed;
    cv::cvtColor(img(roi), gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV);

    const int kernel_size = 5;
    const cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(binary, closed, cv::MORPH_CLOSE, element);

    cv::Mat labels, stats, centroids;
    int num = cv::connectedComponentsWithStats(closed, labels, stats, centroids);

    double left_len = 500, right_len = 500;
    cv::Rect left_rect(0, mid_point.y, 0, 0), right_rect(img.cols, mid_point.y, 0, 0);
    for (int i = 1; i < num; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);

        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

        cv::Rect box(x, y, width, height);
        cv::rectangle(img, box, cv::Scalar(0, 255, 0), 2);  // 녹색 바운딩 박스

        if (std::abs(250 - x) < std::abs(250 - (x + width))) {  // 오른쪽
            double len = cv::norm(mid_point - cv::Point(x, y + height));
            if (len < right_len) {
                right_len = len;
                right_rect = box;
            }
        } else { // 왼쪽 
            double len = cv::norm(cv::Point(x + width, y + height) - mid_point);
            if (len < left_len) {
                left_len = len;
                left_rect = box;
            }
        }
    }

    double right_angle_rad, left_angle_rad;
    double right_rect_y, left_rect_y;

    // 오른쪽
    right_angle_rad = drawAndGetAngle(img, right_rect, mid_point, cv::Scalar(255,0,255), false, right_rect_y);

    // 왼쪽
    left_angle_rad = drawAndGetAngle(img, left_rect, mid_point, cv::Scalar(255,0,0), true, left_rect_y);

    double dist = cv::norm(cv::Point(right_rect.x, right_rect_y) - cv::Point(left_rect.x + left_rect.width, left_rect_y));
    cv::line(img, cv::Point(right_rect.x, right_rect_y), cv::Point(left_rect.x + left_rect.width, left_rect_y), cv::Scalar(30,60,200));
    RCLCPP_INFO(node->get_logger(),"두 점 사이의 거리 : %.1lf", dist);

    double mid_angle = midAngle(left_angle_rad, right_angle_rad);

    cv::line(img, cv::Point(mid_point.x, 0), mid_point, cv::Scalar(0,0,0));
    cv::arrowedLine(img, mid_point,
        cv::Point(mid_point.x + cvRound(mid_point.x * std::sin(mid_angle)),
                  mid_point.y + cvRound(-1 * mid_point.y * std::cos(mid_angle))),
        cv::Scalar(0,140,255));

    if (!speed_stop && dist > 60) {
        vel = speeds(mid_angle);
    } else {
        vel.x = 0;
        vel.y = 0;
    }

    mypub->publish(vel);

    // 중심 좌표
    cv::circle(img, cv::Point(250, 250), 5, cv::Scalar(255,255,0), 1);
    imshow("lidar", img);
    int key = cv::waitKey(30) & 0xFF;
    if (key == 's') speed_stop = !speed_stop;
    img = cv::Scalar(255, 255, 255);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    geometry_msgs::msg::Vector3 vel;
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;
    auto node = rclcpp::Node::make_shared("sllidar_client");

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);

    std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr scan)> fn;
    fn = std::bind(scanCb, node, mypub, vel, _1);

    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), fn);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}