//
// Created by baiye on 2026/2/28.
//

#ifndef SP_VISION_ROS_CAMERA_NODE_H
#define SP_VISION_ROS_CAMERA_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "io/camera.hpp"

class CameraNode : public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions & options);
    ~CameraNode();

private:
    void publish_loop();
    void declare_all_parameters();

    // 动态参数回调
    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params);

    std::unique_ptr<io::Camera> camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    std::thread pub_thread_;
    std::atomic<bool> quit_{false};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif //SP_VISION_ROS_CAMERA_NODE_H