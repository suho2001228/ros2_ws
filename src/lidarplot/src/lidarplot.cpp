/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <opencv2/opencv.hpp>
#define RAD2DEG(x) ((x)*180. / M_PI) // 라디안 -> 도 

cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;

  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n",
         scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n",
         RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
  for (int i = 0; i < count; i++) {
    float r = scan->angle_min + scan->angle_increment * i;
    float degree = RAD2DEG(r);
    printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);

    int x = cvRound(250 + .500 * scan->ranges[i] * sin(r));
    int y = cvRound(250 + 50.0 * scan->ranges[i] * cos(r));
    if(x > 500 || y > 500 || x < 0 || y < 0) continue;

    cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0,0,255), -1);
    // printf("x, y : [%d, %d]\n", x, y);
  }

  cv::circle(img, cv::Point(250, 250), 5, cv::Scalar(255,255,0), 1);
  imshow("lidar", img);
  cv::waitKey(1);
 img = cv::Scalar(255, 255, 255);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
