//
// Created by baiye on 2026/2/28.
//

#include "../include/sp_vision_ros/camera_node.h"
#include "tools/logger.hpp"

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("camera_node", options) {
    // 声明所有参数
    declare_all_parameters();

    // 初始化相机（传递节点指针）
    camera_ = std::make_unique<io::Camera>(this);
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS());

    // 添加动态参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&CameraNode::param_callback, this, std::placeholders::_1));

    pub_thread_ = std::thread(&CameraNode::publish_loop, this);
    tools::logger()->info("CameraNode 已启动");
}

void CameraNode::declare_all_parameters() {
    // 相机类型
    this->declare_parameter("camera_name", "mindvision");

    // MindVision 参数
    this->declare_parameter("exposure_ms", 5000);
    this->declare_parameter("gamma", 1.0);
    this->declare_parameter("vid_pid", "0483:7525");

    // HikRobot 参数
    this->declare_parameter("gain", 1.0);
}

rcl_interfaces::msg::SetParametersResult CameraNode::param_callback(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "参数已更新";

    for (const auto &param : params) {
        const auto &name = param.get_name();

        try {
            camera_->update_param(name, param);
        } catch (const std::exception & e) {
            result.successful = false;
            result.reason = std::string("参数更新失败: ") + e.what();
            tools::logger()->error("[HotReload] 更新 {} 失败: {}", name, e.what());
            return result;
        }
    }

    return result;
}

void CameraNode::publish_loop() {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;  // 未使用，保留参数

    while (!quit_ && rclcpp::ok()) {
        camera_->read(img, timestamp);

        if (!img.empty()) {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();

            // 使用 ROS 时间（与整个系统时间一致，用于 TF 查找等）
            msg->header.stamp = this->now();
            msg->header.frame_id = "camera_link";
            msg->height = img.rows;
            msg->width = img.cols;
            msg->encoding = "bgr8";
            msg->is_bigendian = false;
            msg->step = img.step;

            // 移动数据以避免拷贝
            size_t data_size = img.total() * img.elemSize();
            msg->data.resize(data_size);
            std::memcpy(msg->data.data(), img.data, data_size);

            // 使用 unique_ptr 发布，减少一次拷贝
            img_pub_->publish(std::move(msg));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));  // ~30fps
    }
}

CameraNode::~CameraNode() {
    quit_ = true;
    if (pub_thread_.joinable()) {
        pub_thread_.join();
    }
    tools::logger()->info("CameraNode 已关闭");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}