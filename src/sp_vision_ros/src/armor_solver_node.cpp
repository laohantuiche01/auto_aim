//
// Created by baiye on 2026/3/2.
//

#include "../include/sp_vision_ros/armor_solver_node.h"
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions& options)
    : Node("armor_solver", options),
      target_queue_(1),
      quit_(false)
{
    tools::logger()->info("ArmorSolverNode (后端处理) 已启动");

    // 声明所有参数
    declare_all_parameters();

    // 初始化算法模块（传递节点指针）
    solver_ = std::make_unique<auto_aim::Solver>(this);
    tracker_ = std::make_unique<auto_aim::Tracker>(this, *solver_);
    planner_ = std::make_unique<auto_aim::Planner>(this);

    target_queue_.push(std::nullopt);

    // 初始化 TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // 订阅前端检测结果
    armors_sub_ = this->create_subscription<sp_vision_msgs::msg::Armors>(
        "/detected_armors", rclcpp::SensorDataQoS(),
        std::bind(&ArmorSolverNode::armors_callback, this, std::placeholders::_1));

    // 订阅云台状态
    gimbal_state_sub_ = this->create_subscription<sp_vision_msgs::msg::GimbalState>(
        "/gimbal/state", rclcpp::SensorDataQoS(),
        std::bind(&ArmorSolverNode::gimbal_state_callback, this, std::placeholders::_1));

    // 发布控制命令
    cmd_pub_ = this->create_publisher<sp_vision_msgs::msg::GimbalCmd>(
        "/auto_aim/cmd_gimbal", rclcpp::SensorDataQoS());

    // 添加动态参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ArmorSolverNode::param_callback, this, std::placeholders::_1));

    // 启动规划线程
    planner_thread_ = std::thread(&ArmorSolverNode::planner_loop, this);
}

ArmorSolverNode::~ArmorSolverNode()
{
    quit_ = true;
    if (planner_thread_.joinable())
    {
        planner_thread_.join();
    }
    tools::logger()->info("ArmorSolverNode 已关闭");
}

void ArmorSolverNode::gimbal_state_callback(const sp_vision_msgs::msg::GimbalState::SharedPtr msg)
{
    tools::logger()->debug("[ArmorSolver] 收到云台状态: yaw={:.2f}, pitch={:.2f}", msg->yaw, msg->pitch);

    // 缓存云台状态数据（使用 C++17 兼容语法）
    GimbalStateData gs;
    gs.yaw = msg->yaw;
    gs.pitch = msg->pitch;
    gs.yaw_vel = msg->yaw_vel;
    gs.pitch_vel = msg->pitch_vel;
    gs.bullet_speed = msg->bullet_speed;
    gimbal_state_ = gs;
}

void ArmorSolverNode::armors_callback(const sp_vision_msgs::msg::Armors::SharedPtr armors_msg)
{
    tools::logger()->debug("[ArmorSolver] 收到 {} 个装甲板", armors_msg->armors.size());

    if (armors_msg->armors.empty())
    {
        target_queue_.push(std::nullopt);
        return;
    }

    // 获取云台姿态
    auto q = get_gimbal_orientation(armors_msg->header.stamp);
    solver_->set_R_gimbal2world(q);

    // 转换消息为 Armor
    auto armors = msg_to_armors(*armors_msg);

    // 将 ROS 时间直接转换成 steady_clock
    // 注意：偏移量会在 dt 计算中抵消，所以直接转换即可
    auto ros_time = rclcpp::Time(armors_msg->header.stamp);
    auto t = std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(ros_time.nanoseconds()));

    // Tracker 处理
    auto targets = tracker_->track(armors, t);

    // publishTransform(armors.front().ypr_in_gimbal(0),
    //                  armors.front().ypr_in_gimbal(1),
    //                  armors.front().ypr_in_gimbal(2),
    //                  armors.front().xyz_in_gimbal,
    //                  "gimbal_link",
    //                  "target_link");

    if (!targets.empty())
    {
        target_queue_.push(targets.front());
        tools::logger()->debug("[ArmorSolver] 跟踪到目标");
    }
    else
    {
        target_queue_.push(std::nullopt);
        tools::logger()->debug("[ArmorSolver] 无有效目标");
    }
}

std::list<auto_aim::Armor> ArmorSolverNode::msg_to_armors(const sp_vision_msgs::msg::Armors& msg)
{
    std::list<auto_aim::Armor> armors;

    for (const auto& armor_msg : msg.armors)
    {
        // 提取 keypoints
        std::vector<cv::Point2f> keypoints;
        if (armor_msg.keypoints.size() == 4)
        {
            for (size_t i = 0; i < 4; ++i)
            {
                keypoints.push_back(cv::Point2f(
                    armor_msg.keypoints[i].x,
                    armor_msg.keypoints[i].y
                ));
            }
        }

        // 使用 Armor 构造函数创建对象
        // 构造函数会根据 class_id 自动设置 color, name, type
        auto_aim::Armor armor(armor_msg.class_id, 0.0f, cv::Rect(), keypoints);

        armors.push_back(armor);
    }

    return armors;
}

Eigen::Quaterniond ArmorSolverNode::get_gimbal_orientation(const rclcpp::Time& stamp)
{
    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "world", "gimbal_link", stamp, 50ms);

        Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform);
        // 显式转换为 Quaterniond
        return Eigen::Quaterniond(transform_eigen.rotation());
    }
    catch (tf2::TransformException& ex)
    {
        tools::logger()->warn("TF 查找失败: {}", ex.what());
        return Eigen::Quaterniond::Identity();
    }
}

void ArmorSolverNode::publishTransform(
    const Eigen::Quaterniond& q,
    const Eigen::Vector3d& t,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const rclcpp::Time& timestamp)
{
    // 构建TF变换消息
    geometry_msgs::msg::TransformStamped transform_stamped;

    // 设置时间戳
    transform_stamped.header.stamp = this->get_clock()->now();
    // 设置坐标系
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;

    // 设置平移
    transform_stamped.transform.translation.x = t.x();
    transform_stamped.transform.translation.y = t.y();
    transform_stamped.transform.translation.z = t.z();

    // 设置旋转（四元数）
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    // 发布TF变换
    tf_broadcaster_->sendTransform(transform_stamped);
    RCLCPP_DEBUG(
        this->get_logger(),
        "发布TF变换: %s -> %s | 平移(x=%.3f, y=%.3f, z=%.3f) | 旋转(x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
        frame_id.c_str(), child_frame_id.c_str(),
        t.x(), t.y(), t.z(),
        q.x(), q.y(), q.z(), q.w()
    );
}

void ArmorSolverNode::publishTransform(
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const rclcpp::Time& timestamp)
{
    // 将旋转矩阵转换为四元数（Eigen内置转换，保证wxyz顺序）
    Eigen::Quaterniond q(R);
    // 调用四元数版本的发布函数
    publishTransform(q, t, frame_id, child_frame_id, timestamp);
}

void ArmorSolverNode::publishTransform(double roll, double pitch, double yaw, const Eigen::Vector3d& t,
                                       const std::string& frame_id, const std::string& child_frame_id,
                                       const rclcpp::Time& timestamp)
{
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) // 先绕Z轴转yaw
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) // 再绕Y轴转pitch
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()); // 最后绕X轴转roll

    publishTransform(q, t, frame_id, child_frame_id, timestamp);

    RCLCPP_DEBUG(
        this->get_logger(),
        "发布TF变换(欧拉角): %s -> %s | 平移(x=%.3f, y=%.3f, z=%.3f) | 欧拉角(roll=%.3f°(%.3frad), pitch=%.3f°(%.3frad), yaw=%.3f°(%.3frad))",
        frame_id.c_str(), child_frame_id.c_str(),
        t.x(), t.y(), t.z(),
        roll * 180 / M_PI, roll,
        pitch * 180 / M_PI, pitch,
        yaw * 180 / M_PI, yaw
    );
}

void ArmorSolverNode::planner_loop()
{
    while (!quit_ && rclcpp::ok())
    {
        if (!target_queue_.empty())
        {
            auto target_opt = target_queue_.front();

            if (target_opt.has_value())
            {
                auto target = target_opt.value();

                // 获取弹速（如果没有收到 gimbal_state，使用默认值）
                double bullet_speed = gimbal_state_.has_value()
                                          ? gimbal_state_.value().bullet_speed
                                          : default_bullet_speed_;

                // Planner 规划
                auto plan = planner_->plan(target, bullet_speed);

                // 发布控制命令
                sp_vision_msgs::msg::GimbalCmd gimbal_cmd;
                gimbal_cmd.header.frame_id = "";
                gimbal_cmd.header.stamp = this->now();

                gimbal_cmd.mode = plan.control;
                gimbal_cmd.fire = plan.fire;
                gimbal_cmd.yaw = plan.yaw * CV_PI / 180;
                gimbal_cmd.pitch = plan.pitch * CV_PI / 180;
                gimbal_cmd.yaw_vel = plan.yaw_vel;
                gimbal_cmd.pitch_vel = plan.pitch_vel;

                cmd_pub_->publish(gimbal_cmd);
                //tools::logger()->debug("[ArmorSolver] 发布控制命令: yaw={:.2f}, pitch={:.2f}, fire={}",
               //                        plan.yaw, plan.pitch, plan.fire);
            }

            std::this_thread::sleep_for(10ms);
        }
        else
        {
            std::this_thread::sleep_for(200ms);
        }
    }
}

void ArmorSolverNode::declare_all_parameters()
{
    // Tracker 参数
    this->declare_parameter("enemy_color", "blue");
    this->declare_parameter("min_detect_count", 5);
    this->declare_parameter("max_temp_lost_count", 15);
    this->declare_parameter("outpost_max_temp_lost_count", 75);

    // Aimer 参数
    this->declare_parameter("yaw_offset", 0.0);
    this->declare_parameter("pitch_offset", 4.0);
    this->declare_parameter("comming_angle", 55.0);
    this->declare_parameter("leaving_angle", 20.0);
    this->declare_parameter("decision_speed", 7.0);
    this->declare_parameter("high_speed_delay_time", 0.04);
    this->declare_parameter("low_speed_delay_time", 0.04);
    this->declare_parameter("left_yaw_offset", -1.0);
    this->declare_parameter("right_yaw_offset", 1.0);

    // Shooter 参数
    this->declare_parameter("first_tolerance", 3.0);
    this->declare_parameter("second_tolerance", 2.0);
    this->declare_parameter("judge_distance", 2.0);
    this->declare_parameter("auto_fire", true);

    // Planner 参数
    this->declare_parameter("fire_thresh", 0.0035);
    this->declare_parameter("max_yaw_acc", 50.0);
    this->declare_parameter("Q_yaw", std::vector<double>{9000000.0, 0.0});
    this->declare_parameter("R_yaw", std::vector<double>{1.0});
    this->declare_parameter("max_pitch_acc", 100.0);
    this->declare_parameter("Q_pitch", std::vector<double>{9000000.0, 0.0});
    this->declare_parameter("R_pitch", std::vector<double>{1.0});

    // Solver 相机标定参数
    this->declare_parameter("R_gimbal2imubody", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    this->declare_parameter("camera_matrix", std::vector<double>{
                                580.095913, 0.0, 360.323345, 0.0, 580.141271, 269.174007, 0.0, 0.0, 1.0
                            });
    this->declare_parameter("distort_coeffs", std::vector<double>{
                                -0.081190039023144409, 0.19231523391750371, 0.00096216291015417384,
                                -0.00049720732747006446, 0.0
                            });
    //this->declare_parameter("R_camera2gimbal", std::vector<double>{-0.027182119030230909, -0.12616154330853446, 0.99163723074269183, -0.99949106557517331, 0.019998323121329122, -0.024853106601381177, -0.016695575474690555, -0.99180811252093692, -0.12664093215554434});
    this->declare_parameter("R_camera2gimbal", std::vector<double>{0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0});
    this->declare_parameter("t_camera2gimbal",
                            std::vector<double>{-0.18301755922179547, -0.0070579365371973915, 0.031873960507270753});
}

rcl_interfaces::msg::SetParametersResult ArmorSolverNode::param_callback(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "参数已更新";

    for (const auto& param : params)
    {
        const auto& name = param.get_name();

        try
        {
            // 相机参数不支持热更新
            if (name == "camera_matrix" || name == "distort_coeffs" ||
                name == "R_camera2gimbal" || name == "t_camera2gimbal" ||
                name == "R_gimbal2imubody")
            {
                result.successful = false;
                result.reason = "相机参数不支持热更新，请重启节点";
                tools::logger()->warn("[HotReload] {} 不支持热更新", name);
                return result;
            }

            // Tracker 参数
            tracker_->update_param(name, param);

            // Planner 参数
            planner_->update_param(name, param, this);
        }
        catch (const std::exception& e)
        {
            result.successful = false;
            result.reason = std::string("参数更新失败: ") + e.what();
            tools::logger()->error("[HotReload] 更新 {} 失败: {}", name, e.what());
            return result;
        }
    }

    return result;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorSolverNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
