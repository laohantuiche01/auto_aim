#include "kalman_debug.hpp"

KalmanDebug::KalmanDebug() : node_ptr_(nullptr), is_inited_(false)
{
}

KalmanDebug& KalmanDebug::get_instance()
{
    static KalmanDebug instance;
    return instance;
}

void KalmanDebug::init(rclcpp::Node* node, const std::string& root_topic)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (node != nullptr)
    {
        node_ptr_ = node;
        root_topic_ = root_topic;
        is_inited_ = true;
        RCLCPP_INFO(node->get_logger(), "KalmanDebug init success (root topic: %s)", root_topic_.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("KalmanDebug"), "Init failed: node weak ptr is invalid!");
        is_inited_ = false;
    }
}

void KalmanDebug::publish(const std::string& text, const std::string& sub_topic)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (!is_inited_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("KalmanDebug"), "Not initialized! Call init() first.");
        return;
    }

    if (node_ptr_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("KalmanDebug"), "Node pointer is null!");
        is_inited_ = false;
        return;
    }

    const std::string full_topic = root_topic_ + "/" + sub_topic;
    if (string_pub_map_.find(full_topic) == string_pub_map_.end())
    {
        string_pub_map_[full_topic] = node_ptr_->create_publisher<std_msgs::msg::String>(full_topic, 10);
    }

    std_msgs::msg::String msg;
    msg.data = text;
    string_pub_map_[full_topic]->publish(msg);
}

void KalmanDebug::clear()
{
    std::lock_guard<std::mutex> lock(mtx_);

    pub_map_.clear();
    string_pub_map_.clear();

    is_inited_ = false;

    RCLCPP_INFO(rclcpp::get_logger("KalmanDebug"), "KalmanDebug cleared successfully!");
}
