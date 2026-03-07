#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
// #include "tasks/omniperception/perceptron.hpp"  // TODO: omniperception module not found
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(rclcpp::Node * node, Solver & solver);

  std::string state() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  // TODO: omniperception module not found
  // std::tuple<omniperception::DetectionResult, std::list<Target>> track(
  //   const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  //   std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

  // 更新参数（热重载）
  void update_param(const std::string & name, const rclcpp::Parameter & param);

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  bool first_frame_ = true;  // 标记是否为第一帧

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP