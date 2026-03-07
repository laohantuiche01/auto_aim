//
// Created by baiye on 2026/3/2.
//

#ifndef SP_VISION_ROS_ARMOR_SOLVER_NODE_H
#define SP_VISION_ROS_ARMOR_SOLVER_NODE_H

#include <string>
#include <thread>
#include <atomic>
#include <optional>


#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "sp_vision_msgs/msg/gimbal_state.hpp"
#include "sp_vision_msgs/msg/gimbal_cmd.hpp"
#include "sp_vision_msgs/msg/armors.hpp"

#include "auto_aim/solver.hpp"
#include "auto_aim/tracker.hpp"
#include "auto_aim/planner/planner.hpp"
#include "auto_aim/armor.hpp"
#include "auto_aim/target.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/logger.hpp"

class ArmorSolverNode : public rclcpp::Node {
public:
    ArmorSolverNode(const rclcpp::NodeOptions & options);
    ~ArmorSolverNode();

private:
    void armors_callback(const sp_vision_msgs::msg::Armors::SharedPtr armors_msg);
    void gimbal_state_callback(const sp_vision_msgs::msg::GimbalState::SharedPtr msg);
    void planner_loop();

    // 声明所有参数
    void declare_all_parameters();

    // 动态参数回调
    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params);

    // 将 ROS 消息转换为 Armor
    std::list<auto_aim::Armor> msg_to_armors(const sp_vision_msgs::msg::Armors & msg);

    // 从 TF 获取云台姿态
    Eigen::Quaterniond get_gimbal_orientation(const rclcpp::Time & stamp);

    // 算法模块 - Solver + Tracker + Planner
    std::unique_ptr<auto_aim::Solver> solver_;
    std::unique_ptr<auto_aim::Tracker> tracker_;
    std::unique_ptr<auto_aim::Planner> planner_;

    double default_bullet_speed_;

    // 线程控制
    tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue_;
    std::atomic<bool> quit_;
    std::thread planner_thread_;

    // 云台状态缓存
    struct GimbalStateData {
        double yaw;
        double pitch;
        double yaw_vel;
        double pitch_vel;
        double bullet_speed;
    };
    std::optional<GimbalStateData> gimbal_state_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS 接口
    rclcpp::Subscription<sp_vision_msgs::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Subscription<sp_vision_msgs::msg::GimbalState>::SharedPtr gimbal_state_sub_;
    rclcpp::Publisher<sp_vision_msgs::msg::GimbalCmd>::SharedPtr cmd_pub_;

    // 动态参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif //SP_VISION_ROS_ARMOR_SOLVER_NODE_H