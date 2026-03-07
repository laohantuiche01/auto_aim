//
// Created by baiye on 2026/2/28.
//

#include "../include/sp_vision_ros/gimbal_node.h"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

GimbalNode::GimbalNode(const rclcpp::NodeOptions & options)
: Node("gimbal_node", options) {
    // 声明所有参数
    declare_all_parameters();

    // 初始化云台（传递节点指针）
    gimbal_ = std::make_unique<io::Gimbal>(this);

    state_pub_ = this->create_publisher<sp_vision_msgs::msg::GimbalState>(
        "/gimbal/state", rclcpp::SensorDataQoS());
    cmd_sub_ = this->create_subscription<sp_vision_msgs::msg::GimbalCmd>(
        "/cmd_gimbal", rclcpp::SensorDataQoS(),
        std::bind(&GimbalNode::cmd_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    publish_timer_ = this->create_wall_timer(
        10ms,  // 100Hz
        [this]() { this->publish_timer_callback(); }
    );

    // 添加动态参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&GimbalNode::param_callback, this, std::placeholders::_1));

    tools::logger()->info("GimbalNode 已启动");
}

void GimbalNode::declare_all_parameters() {
    this->declare_parameter("com_port", "/dev/ttyACM0");
}

rcl_interfaces::msg::SetParametersResult GimbalNode::param_callback(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "参数已更新";

    for (const auto &param : params) {
        const auto &name = param.get_name();

        try {
            gimbal_->update_param(name, param);
        } catch (const std::exception & e) {
            result.successful = false;
            result.reason = std::string("参数更新失败: ") + e.what();
            tools::logger()->error("[HotReload] 更新 {} 失败: {}", name, e.what());
            return result;
        }
    }

    return result;
}

void GimbalNode::cmd_callback(const sp_vision_msgs::msg::GimbalCmd::SharedPtr msg) {
    // mode: 0=不控制, 1=仅控制, 2=控制+开火
    bool control = (msg->mode != 0);
    // 消息中没有加速度字段，默认设为 0
    gimbal_->send(
        control,
        msg->fire,
        static_cast<float>(msg->yaw),
        static_cast<float>(msg->yaw_vel),
        0.0f,
        static_cast<float>(msg->pitch),
        static_cast<float>(msg->pitch_vel),
        0.0f
    );
}

void GimbalNode::publish_timer_callback() {
    auto gs = gimbal_->state();
    auto mode = gimbal_->mode();
    auto now = std::chrono::steady_clock::now();
    auto q = gimbal_->q(now);

    sp_vision_msgs::msg::GimbalState state_msg;
    state_msg.header.stamp = this->now();
    state_msg.header.frame_id = "gimbal_link";
    state_msg.quaternion = {q.w(), q.x(), q.y(), q.z()};  // wxyz
    state_msg.mode = static_cast<uint8_t>(mode);
    state_msg.yaw = gs.yaw;
    state_msg.pitch = gs.pitch;
    state_msg.yaw_vel = gs.yaw_vel;
    state_msg.pitch_vel = gs.pitch_vel;
    state_msg.bullet_speed = gs.bullet_speed;
    state_msg.bullet_count = gs.bullet_count;
    state_pub_->publish(state_msg);

    // 发布 TF (odom → gimbal_link)
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = "odom";
    tf.child_frame_id = "gimbal_link";

    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();

    tf_broadcaster_->sendTransform(tf);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}