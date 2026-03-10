#ifndef SP_VISION_ROS_KALMAN_DEBUG_H
#define SP_VISION_ROS_KALMAN_DEBUG_H

// ros2相关内容
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

//基础库引用
#include <Eigen/Dense>
#include <unordered_map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

template <typename T>
struct is_std_vector : std::false_type {};

template <typename T>
struct is_std_vector<std::vector<T>> : std::true_type {};

template <typename T>
constexpr bool is_std_vector_v = is_std_vector<T>::value;

class KalmanDebug
{
public:
    /**
     * 单例模式
     * @return 返回单例引用
     */
    static KalmanDebug& get_instance();

    /**
     * 初始化函数，弱持有对象
     * @param node_weak 传入节点指针
     * @param root_topic 话题的前缀
     */
    void init(rclcpp::Node* node, const std::string& root_topic = "auto_aim/debug");

    /**
     * 创建并发布相关的调试信息
     * @tparam T 可以传入矩阵
     * @param data 矩阵数据
     * @param sub_topic 定制的话题名称
     */
    template <typename T>
    void publish(const T& data, const std::string& sub_topic)
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
        if (pub_map_.find(full_topic) == pub_map_.end())
        {
            pub_map_[full_topic] = node_ptr_->create_publisher<std_msgs::msg::Float64MultiArray>(full_topic, 10);
        }

        std_msgs::msg::Float64MultiArray msg;
        using DecayedT = std::decay_t<T>;

        if constexpr (std::is_base_of_v<Eigen::EigenBase<DecayedT>, DecayedT>)
        {
            fill_msg(data, msg); // Eigen类型
        }
        else if constexpr (is_std_vector_v<DecayedT>)
        {
            fill_msg(data, msg); // std::vector类型
        }
        else if constexpr (std::is_arithmetic_v<DecayedT>)
        {
            fill_msg(data, msg); // 单个数值
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("KalmanDebug"), "Unsupported data type!");
            return;
        }
        auto pub = pub_map_[full_topic];
        if (pub)
        {
            pub->publish(msg);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("KalmanDebug"), "Publisher for topic %s is null!", full_topic.c_str());
        }
    }

    /**
     *
     * @param text 发布文本调试信息
     * @param sub_topic
     */
    void publish(const std::string& text, const std::string& sub_topic = "log");

    /**
     * 重置函数
     */
    void clear();

    KalmanDebug(const KalmanDebug&) = delete;
    KalmanDebug& operator=(const KalmanDebug&) = delete;

private:
    KalmanDebug();

    rclcpp::Node* node_ptr_;
    std::string root_topic_;
    bool is_inited_; //是否初始化
    std::mutex mtx_;

    //发布数据信息的缓存
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> pub_map_;
    //发布文字信息的缓存
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> string_pub_map_;

    template <typename Derived>
    void fill_msg(const Eigen::MatrixBase<Derived>& data, std_msgs::msg::Float64MultiArray& msg)
    {
        msg.data.reserve(data.size());
        for (int i = 0; i < data.size(); ++i)
        {
            msg.data.push_back(data(i));
        }
    }

    template <typename T>
    void fill_msg(const std::vector<T>& data, std_msgs::msg::Float64MultiArray& msg)
    {
        static_assert(std::is_arithmetic_v<T>, "Vector must contain arithmetic types (int/double/float etc.)!");
        msg.data.reserve(data.size());
        for (const auto& val : data)
        {
            msg.data.push_back(static_cast<double>(val));
        }
    }

    template <typename T>
    std::enable_if_t<std::is_arithmetic_v<T>>
    fill_msg(const T& single_val, std_msgs::msg::Float64MultiArray& msg)
    {
        msg.data.clear();
        msg.data.push_back(static_cast<double>(single_val));
    }
};


#endif //SP_VISION_ROS_KALMAN_DEBUG_H
