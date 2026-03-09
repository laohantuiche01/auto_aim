#include <timer.hpp>

const rclcpp::Node* RosTimer::node_ptr_ = nullptr;
rclcpp::Clock::SharedPtr RosTimer::ros_clock_ = nullptr;
bool RosTimer::is_initialized_ = false;

void RosTimer::init(const rclcpp::Node* node)
{
    if (node == nullptr)
    {
        throw std::invalid_argument("RosTimer::init: node pointer cannot be null!");
    }
    if (is_initialized_)
    {
        RCLCPP_WARN(rclcpp::get_logger("RosTimer"), "RosTimer has already been initialized, skipping...");
        return;
    }
    node_ptr_ = node;
    auto non_const_node = const_cast<rclcpp::Node*>(node_ptr_);
    non_const_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    rclcpp::Parameter use_sim_time_param;
    if (non_const_node->get_parameter("use_sim_time", use_sim_time_param) &&
        use_sim_time_param.as_bool())
    {
        RCLCPP_INFO(rclcpp::get_logger("RosTimer"), "Successfully set use_sim_time=true (bind to /clock)");
    }
    else
    {
        throw std::runtime_error("RosTimer::init: Failed to set use_sim_time parameter!");
    }

    ros_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

    is_initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("RosTimer"), "RosTimer initialized successfully!");
}

std::chrono::steady_clock::time_point RosTimer::get_steady_time_from_clock()
{
    if (!is_initialized_)
    {
        throw std::runtime_error("RosTimeUtils not initialized! Call init() first.");
    }

    rclcpp::Time ros_time = ros_clock_->now();

    std::chrono::nanoseconds ns(ros_time.nanoseconds());
    return std::chrono::steady_clock::time_point(ns);
}

rclcpp::Time RosTimer::get_ros_time()
{
    if (!is_initialized_)
    {
        throw std::runtime_error("RosTimeUtils not initialized! Call init() first.");
    }
    return ros_clock_->now();
}

rclcpp::Clock::SharedPtr RosTimer::get_ros_clock()
{
    if (!is_initialized_)
    {
        throw std::runtime_error("RosTimer not initialized! Call init() first.");
    }
    return ros_clock_;
}

double RosTimer::steady_time_to_double_seconds(const std::chrono::steady_clock::time_point& tp)
{
    int64_t total_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        tp.time_since_epoch()
    ).count();
    return static_cast<double>(total_nanoseconds) / 1e9;
}

int64_t RosTimer::steady_time_to_int64_nanoseconds(const std::chrono::steady_clock::time_point& tp)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        tp.time_since_epoch()
    ).count();
}

double RosTimer::ros_time_to_double_seconds(rclcpp::Time ros_time)
{
    return static_cast<double>(ros_time.nanoseconds()) / 1e9;
}

int64_t RosTimer::ros_time_to_int64_nanoseconds(rclcpp::Time ros_time)
{
    int64_t seconds = ros_time.nanoseconds();
    return seconds;
}
