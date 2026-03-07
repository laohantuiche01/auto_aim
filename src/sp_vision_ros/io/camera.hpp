#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

class Camera
{
public:
  Camera(const std::string & config_path);
  Camera(rclcpp::Node * node);  // ROS2 参数版本
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

  // 更新参数（热重载）
  void update_param(const std::string & name, const rclcpp::Parameter & param);

private:
  std::unique_ptr<CameraBase> camera_;
  rclcpp::Node * node_ = nullptr;  // 保存节点指针用于热重载
  std::string camera_name_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP