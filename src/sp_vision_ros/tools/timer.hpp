#ifndef SP_VISION_ROS_TIMER_H
#define SP_VISION_ROS_TIMER_H

// Ros2 include
#include <rclcpp/rclcpp.hpp>

// base include
#include <chrono>
#include <thread>
#include <stdexcept>
#include <memory>
#include <opencv2/core/hal/interface.h>


class RosTimer
{
public:
    /**
     * @brief 初始化工具类（需在节点创建后调用）
     * @param node 节点指针（用于获取时钟和设置 use_sim_time）
     */
    static void init(const rclcpp::Node* node);

    /**
     * @brief 获取当前 ROS 时间（来自 /clock）并转换为 steady_clock::time_point
     * @return std::chrono::steady_clock::time_point 类型的当前时间（纳秒）
     */
    static std::chrono::steady_clock::time_point get_steady_time_from_clock();

    /**
     * @brief 获取原始的 ROS 时间（rclcpp::Time 类型）
     * @return 来自 /clock 的 ROS 时间
     */
    static rclcpp::Time get_ros_time();

    static rclcpp::Clock::SharedPtr get_ros_clock();

    static double steady_time_to_double_seconds(const std::chrono::steady_clock::time_point& tp);
    static int64_t steady_time_to_int64_nanoseconds(const std::chrono::steady_clock::time_point& tp);

    static double ros_time_to_double_seconds(rclcpp::Time ros_time);
    static int64_t ros_time_to_int64_nanoseconds(rclcpp::Time ros_time);

private:
    // 节点指针（仅持有，不管理生命周期）
    static const rclcpp::Node* node_ptr_;
    // 节点时钟
    static rclcpp::Clock::SharedPtr ros_clock_;
    // 标记是否已初始化
    static bool is_initialized_;
};


#endif
