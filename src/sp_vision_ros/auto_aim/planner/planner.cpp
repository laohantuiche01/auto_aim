#include "planner.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
Planner::Planner(rclcpp::Node * node)
{
  // 从节点读取参数
  yaw_offset_ = node->get_parameter("yaw_offset").as_double() / 57.3;
  pitch_offset_ = node->get_parameter("pitch_offset").as_double() / 57.3;
  fire_thresh_ = node->get_parameter("fire_thresh").as_double();
  decision_speed_ = node->get_parameter("decision_speed").as_double();
  high_speed_delay_time_ = node->get_parameter("high_speed_delay_time").as_double();
  low_speed_delay_time_ = node->get_parameter("low_speed_delay_time").as_double();

  setup_yaw_solver(node);
  setup_pitch_solver(node);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  target.predict(bullet_traj.fly_time);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);
  plan.target_pitch *= 180/M_PI;

  plan.yaw = yaw_solver_->work->x(0, HALF_HORIZON) + yaw0;
  plan.yaw *= 180/M_PI;

  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch *= 180/M_PI;
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  auto shoot_offset_ = 2;
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) return {false};

  double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  // 直接使用时间差，避免混用不同时钟源
  target->predict(delay_time);

  return plan(*target, bullet_speed);
}

void Planner::setup_yaw_solver(rclcpp::Node * node)
{
  auto max_yaw_acc = node->get_parameter("max_yaw_acc").as_double();
  auto Q_yaw = node->get_parameter("Q_yaw").as_double_array();
  auto R_yaw = node->get_parameter("R_yaw").as_double_array();

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver(rclcpp::Node * node)
{
  auto max_pitch_acc = node->get_parameter("max_pitch_acc").as_double();
  auto Q_pitch = node->get_parameter("Q_pitch").as_double_array();
  auto R_pitch = node->get_parameter("R_pitch").as_double_array();

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), bullet_traj.pitch + pitch_offset_};
}

Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

void Planner::update_param(const std::string & name, const rclcpp::Parameter & param, rclcpp::Node * node)
{
  if (name == "yaw_offset") {
    yaw_offset_ = param.as_double() / 57.3;
    tools::logger()->info("[Planner] yaw_offset updated to: {:.3f} deg", param.as_double());
  }
  else if (name == "pitch_offset") {
    pitch_offset_ = param.as_double() / 57.3;
    tools::logger()->info("[Planner] pitch_offset updated to: {:.3f} deg", param.as_double());
  }
  else if (name == "fire_thresh") {
    fire_thresh_ = param.as_double();
    tools::logger()->info("[Planner] fire_thresh updated to: {:.4f}", fire_thresh_);
  }
  else if (name == "decision_speed") {
    decision_speed_ = param.as_double();
    tools::logger()->info("[Planner] decision_speed updated to: {:.2f}", decision_speed_);
  }
  else if (name == "high_speed_delay_time") {
    high_speed_delay_time_ = param.as_double();
    tools::logger()->info("[Planner] high_speed_delay_time updated to: {:.3f}", high_speed_delay_time_);
  }
  else if (name == "low_speed_delay_time") {
    low_speed_delay_time_ = param.as_double();
    tools::logger()->info("[Planner] low_speed_delay_time updated to: {:.3f}", low_speed_delay_time_);
  }
  else if (name == "max_yaw_acc" || name == "Q_yaw" || name == "R_yaw") {
    setup_yaw_solver(node);
    tools::logger()->info("[Planner] yaw_solver parameters updated");
  }
  else if (name == "max_pitch_acc" || name == "Q_pitch" || name == "R_pitch") {
    setup_pitch_solver(node);
    tools::logger()->info("[Planner] pitch_solver parameters updated");
  }
}

}  // namespace auto_aim