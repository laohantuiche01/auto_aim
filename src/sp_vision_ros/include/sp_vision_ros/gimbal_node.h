//
// Created by baiye on 2026/2/28.
//

#ifndef SP_VISION_ROS_GIMBAL_NODE_H
#define SP_VISION_ROS_GIMBAL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "sp_vision_msgs/msg/gimbal_state.hpp"
#include "sp_vision_msgs/msg/gimbal_cmd.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <memory>
#include <string>

#include "io/gimbal/gimbal.hpp"

class GimbalNode : public rclcpp::Node {
public:
    explicit GimbalNode(const rclcpp::NodeOptions & options);
    ~GimbalNode() override = default;

private:
    void cmd_callback(const sp_vision_msgs::msg::GimbalCmd::SharedPtr msg);
    void publish_timer_callback();
    void declare_all_parameters();

    // 动态参数回调
    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params);

    std::unique_ptr<io::Gimbal> gimbal_;
    rclcpp::Publisher<sp_vision_msgs::msg::GimbalState>::SharedPtr state_pub_;
    rclcpp::Subscription<sp_vision_msgs::msg::GimbalCmd>::SharedPtr cmd_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif //SP_VISION_ROS_GIMBAL_NODE_H