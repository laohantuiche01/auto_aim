#include "camera.hpp"

#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"
#include "tools/logger.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");

  if (camera_name == "mindvision") {
    auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else {
    throw std::runtime_error("Unknown camera_name: " + camera_name + "!");
  }
}

Camera::Camera(rclcpp::Node * node) : node_(node)
{
  auto camera_name = node->get_parameter("camera_name").as_string();
  camera_name_ = camera_name;

  if (camera_name == "mindvision") {
    auto exposure_ms = node->get_parameter("exposure_ms").as_double();
    auto gamma = node->get_parameter("gamma").as_double();
    auto vid_pid = node->get_parameter("vid_pid").as_string();
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto exposure_ms = node->get_parameter("exposure_ms").as_double();
    auto gain = node->get_parameter("gain").as_double();
    auto vid_pid = node->get_parameter("vid_pid").as_string();
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else {
    throw std::runtime_error("Unknown camera_name: " + camera_name + "!");
  }
}

void Camera::update_param(const std::string & name, const rclcpp::Parameter & param)
{
  // 相机参数一般不支持热重载（需要重新初始化硬件）
  tools::logger()->warn("[Camera] 参数 {} 不支持热重载，需重启节点", name);
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
