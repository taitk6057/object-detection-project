#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

class CameraServer : public rclcpp::Node
{
public:
  CameraServer(std::string topic, std::string cameraNum, int fps, std::string name)
  : Node(name), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic, rmw_qos_profile_sensor_data);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&CameraServer::timer_callback, this));
    cap = cv::VideoCapture(-1);
  }

private:
  void timer_callback()
  {

    cap >> frame;
    std::cout << frame.rows << " " << frame.cols << std::endl;

    cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC4;
    out_msg.image    = frame;

    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", this->count_);
    publisher_->publish(out_msg.toImageMsg());  // Publish.

    this->count_++;

  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::VideoCapture cap;
  cv::Mat frame;
  size_t count_;
};

int main(int argc, char * argv[])
{

  std::string topic = "camserver/rgb";
  std::string cameraNum = "/dev/video0";
  int fps = 30;
  std::string name = "Camera_Server";

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraServer>(topic, cameraNum, fps, name));
  rclcpp::shutdown();
  return 0;
}