#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("video_play_node");

    std::string video_path = "/home/linux/ros2_ws/src/obstacle_avoidance2/test1.mp4";
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "비디오 파일을 열 수 없습니다");
        rclcpp::shutdown();
        return -1;
    }

    cv::Mat frame;
    rclcpp::Rate loop_rate(10); // 10fps 기준

    
    cv::Mat gray, binary, closed;
    int kernel_size = 5;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    while (rclcpp::ok()) {
        cap >> frame;
        if (frame.empty()) {
            RCLCPP_INFO(node->get_logger(), "영상 끝, 재실행.");
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            continue;
        }
        cv::Rect roi(0, 0, frame.cols, frame.rows / 2);

        cv::cvtColor(frame(roi), gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV);
        cv::morphologyEx(binary, closed, cv::MORPH_CLOSE, element);

        cv::Mat labels, stats, centroids;
        int num = cv::connectedComponentsWithStats(closed, labels, stats, centroids);

        double left_len = 0, right_len = 0;
        cv::Rect left_rect(0, 250, 0, 0), right_rect(500, 250, 0, 0);
        for (int i = 1; i < num; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 10) continue; // 너무 작은 영역 무시

            int x = stats.at<int>(i, cv::CC_STAT_LEFT);
            int y = stats.at<int>(i, cv::CC_STAT_TOP);
            int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

            cv::Rect box(x, y, width, height);
            cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);  // 녹색 바운딩 박스

            if(std::abs(250 - x) < std::abs(250 - (x + width))){  // 왼쪽
                // cv::arrowedLine(frame,cv::Point(250,250),cv::Point(x + width,y+height), cv::Scalar(0,0,0));
                double len = cv::norm(cv::Point(x + width,y + height) - cv::Point(250,250));
                if(len > right_len){
                    right_len = len;
                    right_rect = box;
                }
            }
            else{ // 오른쪽에 있을때
                // cv::arrowedLine(frame,cv::Point(250,250),cv::Point(x,y+height), cv::Scalar(0,0,0));
                double len = cv::norm(cv::Point(x ,y + height) - cv::Point(250,250));
                if(len < left_len){
                    left_len = len;
                    left_rect = box;
                }
            }
        }   
        // 오른쪽
        cv::rectangle(frame, right_rect, cv::Scalar(255, 0, 255), 2);
        cv::arrowedLine(frame,cv::Point(250,250),cv::Point(right_rect.x,
            right_rect.y + right_rect.height), cv::Scalar(255, 0, 255));

        // 왼쪽
        cv::rectangle(frame, left_rect, cv::Scalar(255, 0, 0), 2);
        cv::arrowedLine(frame,cv::Point(250,250),cv::Point(left_rect.x + left_rect.width,
            left_rect.y + left_rect.height), cv::Scalar(255, 0, 0));


        cv::imshow("frame", frame);
        cv::imshow("gray", gray);
        // cv::imshow("closed",closed);
        if (cv::waitKey(1) == 27) break;
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    cap.release();
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}