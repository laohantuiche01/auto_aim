//
// Created by baiye on 2026/2/27.
//

#include "../include/sp_vision_ros/auto_aim_node.h"

using namespace std::chrono_literals;

AutoAimNode::AutoAimNode(const rclcpp::NodeOptions &options)
: Node("auto_aim", options) {
    tools::logger()->info("[AutoAimNode] 前端检测节点已启动");

    // 声明所有参数
    declare_all_parameters();

    // 初始化 YOLO（传递节点指针）
    yolo_ = std::make_unique<auto_aim::YOLO>(this);

    // 订阅图像
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&AutoAimNode::image_callback, this, std::placeholders::_1));

    // 发布检测结果
    armors_pub_ = this->create_publisher<sp_vision_msgs::msg::Armors>(
        "/detected_armors", rclcpp::SensorDataQoS());

    // 添加动态参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&AutoAimNode::param_callback, this, std::placeholders::_1));

}

void AutoAimNode::declare_all_parameters() {
    // YOLO 类型
    this->declare_parameter("yolo_name", "yolov8");

    // 模型路径
    this->declare_parameter("yolov5_model_path", "/home/laohantuiche/Radar/auto_aim/src/sp_vision_ros/assets/yolov5.xml");
    this->declare_parameter("yolov8_model_path", "/home/laohantuiche/Radar/auto_aim/src/sp_vision_ros/assets/yolov8.xml");
    this->declare_parameter("yolo11_model_path", "/home/laohantuiche/Radar/auto_aim/src/sp_vision_ros/assets/yolo11.xml");

    // 设备和阈值
    this->declare_parameter("device", "GPU");
    this->declare_parameter("threshold", 0.6);
    this->declare_parameter("min_confidence", 0.5);

    // ROI 参数
    this->declare_parameter("use_roi", false);
    this->declare_parameter("roi_x", 0);
    this->declare_parameter("roi_y", 0);
    this->declare_parameter("roi_width", 720);
    this->declare_parameter("roi_height", 540);
}

rcl_interfaces::msg::SetParametersResult AutoAimNode::param_callback(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "参数已更新";

    for (const auto &param : params) {
        const auto &name = param.get_name();

        try {
            // YOLO 参数不支持热重载（需要重新加载模型）
            tools::logger()->warn("[AutoAim] 参数 {} 不支持热重载，需重启节点", name);
        } catch (const std::exception & e) {
            result.successful = false;
            result.reason = std::string("参数更新失败: ") + e.what();
            tools::logger()->error("[HotReload] 更新 {} 失败: {}", name, e.what());
            return result;
        }
    }

    return result;
}

void AutoAimNode::image_callback(const sensor_msgs::msg::Image::UniquePtr &img_msg) {
    tools::logger()->debug("[AutoAimNode] 收到图像: {}x{}", img_msg->width, img_msg->height);

    //cv::Mat img = cv_bridge::toCvCopy(*img_msg, img_msg->encoding)->image;
    cv::Mat img = cv_bridge::toCvCopy(*img_msg, "rgb8")->image;

    // YOLO 检测
    auto armors = yolo_->detect(img);

    tools::logger()->debug("[AutoAimNode] 检测到 {} 个装甲板", armors.size());

    // 转换为 ROS 消息并发布
    sp_vision_msgs::msg::Armors armors_msg;
    armors_msg.header.stamp = img_msg->header.stamp;
    armors_msg.header.frame_id = "camera_link";

    for (const auto & armor : armors) {
        armors_msg.armors.push_back(armor_to_msg(armor));
    }

    // if (armors_msg.armors.size() == 1)
    // {
    //     cv::circle(img,armors.front().center_norm,
    //         10,{0,0,255},-1);
    // }
    // cv::imshow("111",img);
    // cv::waitKey(1);


    armors_pub_->publish(armors_msg);
    tools::logger()->debug("[AutoAimNode] 发布 /detected_armors: {} 个装甲板", armors_msg.armors.size());
}

sp_vision_msgs::msg::Armor AutoAimNode::armor_to_msg(const auto_aim::Armor & armor) {
    sp_vision_msgs::msg::Armor msg;

    // 计算 class_id: color * 18 + name * 2 + type
    msg.class_id = static_cast<int32_t>(
        armor.color * 18 + armor.name * 2 + armor.type
    );

    // Keypoints - 4个角点，用于后端 PnP 求解
    if (armor.points.size() == 4) {
        for (size_t i = 0; i < 4; ++i) {
            msg.keypoints[i].x = armor.points[i].x;
            msg.keypoints[i].y = armor.points[i].y;
            msg.keypoints[i].z = 0.0f;
        }
    }

    // 调试/可视化字段
    msg.number = auto_aim::ARMOR_NAMES[armor.name];
    msg.type = auto_aim::ARMOR_TYPES[armor.type];
    msg.color = static_cast<uint8_t>(armor.color);

    // 3D 信息 - 前端不做 PnP，后端填充
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;

    return msg;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoAimNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}