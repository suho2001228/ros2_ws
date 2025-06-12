#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <opencv2/opencv.hpp>
#define RAD2DEG(x) ((x)*180. / M_PI) // 라디안 -> 도 
std::string dst = "output.mp4";
cv::VideoWriter writer;

cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  
  for (int i = 0; i < count; i++) {
    float r = scan->angle_min + scan->angle_increment * i;
    int x = cvRound(250 + 250.0 * scan->ranges[i] * sin(r));
    int y = cvRound(250 + 250.0 * scan->ranges[i] * cos(r));
    if(x > 500 || y > 500 || x < 0 || y < 0) continue;

    cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0,0,255), -1);
  }

//   cv::circle(img, cv::Point(250, 250), 5, cv::Scalar(255,255,0), 1);
//   cv::line(img,cv::Point(250,0),cv::Point(250,499),cv::Scalar(0,0,0));
//   cv::line(img,cv::Point(0,250),cv::Point(499,250),cv::Scalar(0,0,0));
  imshow("lidar", img);
  writer.write(img);
  cv::waitKey(1);
  img = cv::Scalar(255, 255, 255);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");
  writer.open(dst, cv::VideoWriter::fourcc('m','p','4','v'), (double)10, cv::Size(500, 500), true);
  if(!writer.isOpened()) { RCLCPP_ERROR(node->get_logger(), "Writer open failed!"); rclcpp::shutdown(); return -1; }
  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
