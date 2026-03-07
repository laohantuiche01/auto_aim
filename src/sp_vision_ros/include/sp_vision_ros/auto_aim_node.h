//
// Created by baiye on 2026/2/27.
//

#ifndef SP_VISION_ROS_AUTO_AIM_NODE_H
#define SP_VISION_ROS_AUTO_AIM_NODE_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sp_vision_msgs/msg/armors.hpp"

#include "auto_aim/yolo.hpp"
#include "tools/logger.hpp"

class AutoAimNode : public rclcpp::Node {
public:
    AutoAimNode(const rclcpp::NodeOptions & options);
    ~AutoAimNode() = default;

private:
    void image_callback(const sensor_msgs::msg::Image::UniquePtr & img_msg);
    void declare_all_parameters();

    // 将 Armor 转换为 ROS 消息
    sp_vision_msgs::msg::Armor armor_to_msg(const auto_aim::Armor & armor);

    // 动态参数回调
    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params);

    // 算法模块 - 只有检测
    std::unique_ptr<auto_aim::YOLO> yolo_;

    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<sp_vision_msgs::msg::Armors>::SharedPtr armors_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif //SP_VISION_ROS_AUTO_AIM_NODE_H