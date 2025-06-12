#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include"geometry_msgs/msg/vector3.hpp"
#include <iostream>
#include <cmath>

//  가장 가까운 영역 바운딩 박스 그리고 각도 구해주는 함수
double drawAndGetAngle(cv::Mat& frame, const cv::Rect& rect,  
    const cv::Point& mid_point, const cv::Scalar& color, bool is_left) {
    // 사각형 아래 y 좌표
    int rect_y = rect.y + rect.height;

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

    return std::atan2(vy, vx);  // 라디안 단위로 반환
}
// 각도로 계산해서 바퀴 속도 구하는 함수수
geometry_msgs::msg::Vector3 speeds(double mid_angle_rad,
    double base_speed = 50.0, double p = 25.0) {

    double angle_deg = mid_angle_rad / 1.57;
    double left_speed = base_speed + angle_deg * p;
    double right_speed = -base_speed + angle_deg * p;

    geometry_msgs::msg::Vector3 vel;
    vel.x = std::round(left_speed);
    vel.y = std::round(right_speed);
    vel.z = 0.0; 
    return vel;
}
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("video_play_node");

    auto qos_profile=rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);
    geometry_msgs::msg::Vector3 vel;
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;
    std::string video_path = "/home/linux/ros2_ws/src/obstacle_avoidance2/test1.mp4";
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "비디오 파일을 열 수 없습니다");
        rclcpp::shutdown();
        return -1;
    }

    std::string dst = "output.mp4";
    cv::VideoWriter writer;
    writer.open(dst, cv::VideoWriter::fourcc('m','p','4','v'), (double)10, cv::Size(500, 500), true);
    if(!writer.isOpened()) { RCLCPP_ERROR(node->get_logger(), "Writer open failed!"); rclcpp::shutdown(); return -1; }

    cv::Mat frame;
    rclcpp::Rate loop_rate(10); // 10hz 기준

    
    cv::Mat gray, binary, closed;
    int kernel_size = 5;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));

    bool paused = false;
    bool speed_stop = false;
    while (rclcpp::ok()) {
        if(!paused){
            cap >> frame;
            if (frame.empty()) {
                RCLCPP_INFO(node->get_logger(), "영상 끝, 종료.");
                rclcpp::shutdown(); return -1;
                // RCLCPP_INFO(node->get_logger(), "영상 끝, 재실행.");
                // cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                // continue;
            }
            cv::Rect roi(0, 0, frame.cols, frame.rows / 2);
            cv::Point mid_point(frame.cols / 2, frame.rows / 2);
            // RCLCPP_INFO(node->get_logger(), "mid x : %d, y : %d", mid_point.x, mid_point.y);

            cv::cvtColor(frame(roi), gray, cv::COLOR_BGR2GRAY);
            cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV);
            cv::morphologyEx(binary, closed, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 3);

            cv::Mat labels, stats, centroids;
            int num = cv::connectedComponentsWithStats(closed, labels, stats, centroids);
            // imshow("binary",binary);
            imshow("closed",closed);

            double left_len = 500, right_len = 500;
            cv::Rect left_rect(0, mid_point.y, 0, 0), right_rect(500, mid_point.y, 0, 0);
            for (int i = 1; i < num; i++) {
                // int area = stats.at<int>(i, cv::CC_STAT_AREA);
                // if (area < 10) continue; // 너무 작은 영역 무시

                int x = stats.at<int>(i, cv::CC_STAT_LEFT);
                int y = stats.at<int>(i, cv::CC_STAT_TOP);
                int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
                int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

                cv::Rect box(x, y, width, height);
                cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);  // 녹색 바운딩 박스

                if(std::abs(250 - x) < std::abs(250 - (x + width))){  // 오른쪽
                    // cv::arrowedLine(frame,cv::Point(250,250),cv::Point(x + width,y+height), cv::Scalar(0,0,0));
                    double len = cv::norm(mid_point - cv::Point(x,y + height));
                    if(len < right_len){
                        right_len = len;
                        right_rect = box;
                    }
                    // RCLCPP_INFO(node->get_logger(), "오른쪽 %d번째 길이 : %lf,", i, len);
                    
                }
                else{ // 왼쪽에 있을때
                    // cv::arrowedLine(frame,cv::Point(250,250),cv::Point(x,y+height), cv::Scalar(0,0,0));
                    double len = cv::norm(cv::Point(x + width,y + height) - mid_point);
                    if(len < left_len){
                        left_len = len;
                        left_rect = box;
                    }
                    // RCLCPP_INFO(node->get_logger(), "왼쪽 %d번째 길이 : %lf,", i, len);
                }
            }   

            
            // 오른쪽
            double right_angle_rad = drawAndGetAngle(frame, right_rect, mid_point, cv::Scalar(255,0,255), false);
            // RCLCPP_INFO(node->get_logger(), "오른쪽 x : %d, y : %d, w : %d, h : %d", right_rect.x, right_rect.y,right_rect.width,right_rect.height);
            // RCLCPP_INFO(node->get_logger(), "오른쪽 x : %d, y : %d / rad : %lf", right_rect.x, right_rect.y + right_rect.height, right_angle_rad);
            // RCLCPP_INFO(node->get_logger(), "오른쪽 가장 짧은 길이 : %lf,", right_len);
                
            // 왼쪽
            // RCLCPP_INFO(node->get_logger(), "왼쪽 x : %d, y : %d, w : %d, h : %d", left_rect.x, left_rect.y,left_rect.width,left_rect.height);
            double left_angle_rad = drawAndGetAngle(frame, left_rect, mid_point, cv::Scalar(255,0,0), true);
            // RCLCPP_INFO(node->get_logger(), "왼쪽 가장 짧은 길이 : %lf,", left_len);
            

            // double dist = cv::norm(cv::Point(right_rect.x, right_rect_y) - cv::Point(left_rect.x + left_rect.width, left_rect_y));
            // cv::line(frame, cv::Point(right_rect.x, right_rect_y), cv::Point(left_rect.x + left_rect.width, left_rect_y), cv::Scalar(30,60,200));
            // RCLCPP_INFO(node->get_logger(),"두 점 사이의 거리 : %.1lf",dist);
            // RCLCPP_INFO(node->get_logger(), "왼쪽 x : %d, y : %d / rad : %lf", left_rect.x + left_rect.width, left_rect.y + left_rect.height, left_angle_rad);

            // 중간 
            // cv::arrowedLine(frame,cv::Point(250,250),
            //     cv::Point((right_rect.x + left_rect.x + left_rect.width) * 0.5 , 
            //         (right_rect.y + right_rect.height + left_rect.y + left_rect.height) * 0.5),
            //     cv::Scalar(0,0,0));

            
            // float x = (right_rect.x + left_rect.x + left_rect.width) * 0.5;
            // float y = (right_rect.y + right_rect.height + left_rect.y + left_rect.height) * 0.5;
            // cv::circle(frame, cv::Point(x,y),4,cv::Scalar(0,0,0));
            
            
            // RCLCPP_INFO(node->get_logger(),"x : %f, y : %f",x,y);
            // double angle_rad = std::atan((x - 250) / (250 - y));
            // double angle_rad = std::atan2(y - 250, x - 250);

            // 각도만 합친거 : 크기 반영 x
            double mid_angle = midAngle(left_angle_rad,right_angle_rad);
            
            // 각도 + 크기 모두 반영영
            double size_mid_angle = midAngle(left_angle_rad,right_angle_rad,left_len,right_len);
            RCLCPP_INFO(node->get_logger(),"각도 : %lf",mid_angle);

            // double angle_rad = right_angle_rad + left_angle_rad;
            // double angle_deg = mid_angle * 180.0 / CV_PI;
            
            cv::line(frame, cv::Point(mid_point.x,0), mid_point,cv::Scalar(0,0,0));
            cv::arrowedLine(frame, mid_point,
                cv::Point(mid_point.x + cvRound(mid_point.x * std::sin(mid_angle)),
                    mid_point.y + cvRound(-1 *mid_point.y * std::cos(mid_angle))),
                cv::Scalar(0,140,255));
            // RCLCPP_INFO(node->get_logger(),"라디안 : %lf/ 도 : %lf",mid_angle, angle_deg);

            // // 크기 + 각도 
            // cv::arrowedLine(frame, mid_point,
            //     cv::Point(mid_point.x + cvRound(mid_point.x * std::sin(size_mid_angle)),
            //         mid_point.y + cvRound(-1 *mid_point.y * std::cos(size_mid_angle))),
            //     cv::Scalar(255, 90, 0));

            // // 각도 + 크기 + 각도 / 2
            // cv::arrowedLine(frame, mid_point,
            //     cv::Point(mid_point.x + cvRound(mid_point.x * std::sin((mid_angle + size_mid_angle)/2)),
            //         mid_point.y + cvRound(-1 *mid_point.y * std::cos((mid_angle + size_mid_angle)/2))),
            //     cv::Scalar(0, 0, 255));
            
            if(!speed_stop){
                vel = speeds(mid_angle);
            }
            else{
                vel.x = 0;
                vel.y = 0;
            }
            
            mypub->publish(vel);
        }
        cv::imshow("frame", frame);
        writer.write(frame);
        // cv::imshow("gray", gray);
        // cv::imshow("closed",closed);
        int key = cv::waitKey(30) & 0xFF;
        if (key == 27) break;
        else if (key == 'p') paused = !paused;
        else if (key == 's') speed_stop = !speed_stop;
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    cap.release();
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
