#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <rclcpp/rclcpp.hpp>

namespace io
{
    Gimbal::Gimbal(const std::string& config_path)
    {
        auto yaml = tools::load(config_path);
        auto com_port = tools::read<std::string>(yaml, "com_port");

        std::function<void(const GimbalToVision&)> GimbalToVisionCallbackLamda = [this](auto&& PH1)
        {
            GimbalToVisionCallback(std::forward<decltype(PH1)>(PH1));
        };

        try
        {
            serial_ = std::move(robot::RobotSerial(config_path, 115200));
            serial_.registerCallback(0x01, GimbalToVisionCallbackLamda);
        }
        catch (const std::exception& e)
        {
            tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
            exit(1);
        }

        thread_ = std::thread(&Gimbal::read_thread, this);

        queue_.pop();
        tools::logger()->info("[Gimbal] First q received.");
    }

    Gimbal::Gimbal(rclcpp::Node* node)
    {
        auto com_port = node->get_parameter("com_port").as_string();

        try
        {
            serial_ = std::move(robot::RobotSerial(com_port, 115200));
        }
        catch (const std::exception& e)
        {
            tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
            exit(1);
        }

        thread_ = std::thread(&Gimbal::read_thread, this);

        queue_.pop();
        tools::logger()->info("[Gimbal] First q received.");
    }

    void Gimbal::update_param(const std::string& name, const rclcpp::Parameter& param)
    {
        // 串口参数不支持热重载（需要重新连接）
        tools::logger()->warn("[Gimbal] 参数 {} 不支持热重载，需重启节点", name);
    }

    Gimbal::~Gimbal()
    {
        quit_ = true;
        if (thread_.joinable()) thread_.join();
        serial_.close();
    }

    GimbalMode Gimbal::mode() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_;
    }

    GimbalState Gimbal::state() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    std::string Gimbal::str(GimbalMode mode) const
    {
        switch (mode)
        {
        case GimbalMode::IDLE:
            return "IDLE";
        case GimbalMode::AUTO_AIM:
            return "AUTO_AIM";
        case GimbalMode::SMALL_BUFF:
            return "SMALL_BUFF";
        case GimbalMode::BIG_BUFF:
            return "BIG_BUFF";
        default:
            return "INVALID";
        }
    }

    Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
    {
        while (true)
        {
            auto [q_a, t_a] = queue_.pop();
            auto [q_b, t_b] = queue_.front();
            auto t_ab = tools::delta_time(t_a, t_b);
            auto t_ac = tools::delta_time(t_a, t);
            auto k = t_ac / t_ab;
            Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
            if (t < t_a) return q_c;
            if (!(t_a < t && t <= t_b)) continue;

            return q_c;
        }
    }

    void Gimbal::send(io::VisionToGimbal VisionToGimbal)
    {
        tx_data_.mode = VisionToGimbal.mode;
        tx_data_.yaw = VisionToGimbal.yaw;
        tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
        tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
        tx_data_.pitch = VisionToGimbal.pitch;
        tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
        tx_data_.pitch_acc = VisionToGimbal.pitch_acc;

        try
        {
            serial_.write(0x12, tx_data_);
        }
        catch (const std::exception& e)
        {
            tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
        }
    }

    void Gimbal::send(
        bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
        float pitch_acc)
    {
        tx_data_.mode = control ? (fire ? 2 : 1) : 0;
        tx_data_.yaw = yaw;
        tx_data_.yaw_vel = yaw_vel;
        tx_data_.yaw_acc = yaw_acc;
        tx_data_.pitch = pitch;
        tx_data_.pitch_vel = pitch_vel;
        tx_data_.pitch_acc = pitch_acc;

        try
        {
            serial_.write(0x15, tx_data_);
        }
        catch (const std::exception& e)
        {
            tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
        }
    }

    void Gimbal::read_thread()
    {
        tools::logger()->info("[Gimbal] read_thread started.");
        while (true)
        {
            serial_.spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void Gimbal::GimbalToVisionCallback(const GimbalToVision& data)
    {
        auto t = std::chrono::steady_clock::now();
        Eigen::Quaterniond q(data.q[0], data.q[1], data.q[2], data.q[3]);
        queue_.push({q, t});
        std::lock_guard<std::mutex> lock(mutex_);

        state_.yaw = rx_data_.yaw;
        state_.yaw_vel = rx_data_.yaw_vel;
        state_.pitch = rx_data_.pitch;
        state_.pitch_vel = rx_data_.pitch_vel;
        state_.bullet_speed = rx_data_.bullet_speed;
        state_.bullet_count = rx_data_.bullet_count;

        switch (rx_data_.mode)
        {
        case 0:
            mode_ = GimbalMode::IDLE;
            break;
        case 1:
            mode_ = GimbalMode::AUTO_AIM;
            break;
        case 2:
            mode_ = GimbalMode::SMALL_BUFF;
            break;
        case 3:
            mode_ = GimbalMode::BIG_BUFF;
            break;
        default:
            mode_ = GimbalMode::IDLE;
            tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
            break;
        }
    }

    void Gimbal::reconnect()
    {
        int max_retry_count = 10;
        for (int i = 0; i < max_retry_count && !quit_; ++i)
        {
            tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
            try
            {
                serial_.close();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            catch (...)
            {
            }

            try
            {
                serial_.reopen(); // 尝试重新打开
                queue_.clear();
                tools::logger()->info("[Gimbal] Reconnected serial successfully.");
                break;
            }
            catch (const std::exception& e)
            {
                tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
} // namespace io
