#include "cpp/planner.h"

#include "cpp/geometry.h"
#include "cpp/trajectory_tracker.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace hyw_sim {
namespace {

using PlannerCreator =
    std::function<std::unique_ptr<Planner>(const proto::PlannerInputs &)>;

std::unordered_map<std::string, PlannerCreator> &Registry() {
  static auto *registry = new std::unordered_map<std::string, PlannerCreator>();
  return *registry;
}

bool RegisterPlanner(const std::string &name, PlannerCreator creator) {
  Registry()[name] = std::move(creator);
  return true;
}

proto::PlannerConfig
DefaultTrajectoryConfig(const proto::PlannerInputs &inputs) {
  if (inputs.has_trajectory_config()) {
    return inputs.trajectory_config();
  }
  proto::PlannerConfig cfg;
  cfg.set_horizon_s(3.0);
  cfg.set_point_dt_s(0.1);
  return cfg;
}

int ClosestValidRefIndex(const std::vector<proto::ReferencePoint> &ref,
                         double x, double y) {
  int best = -1;
  double best_d = 1e300;
  for (int i = 0; i < static_cast<int>(ref.size()); ++i) {
    if (!ref[i].valid())
      continue;
    const double d = std::hypot(ref[i].x() - x, ref[i].y() - y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

bool RefTangent(const std::vector<proto::ReferencePoint> &ref, int i,
                double &tx, double &ty) {
  if (i < 0 || i >= static_cast<int>(ref.size()))
    return false;
  if (!ref[i].valid())
    return false;
  for (int j = i + 1; j < static_cast<int>(ref.size()); ++j) {
    if (!ref[j].valid())
      continue;
    const double dx = ref[j].x() - ref[i].x();
    const double dy = ref[j].y() - ref[i].y();
    const double len = std::hypot(dx, dy);
    if (len > 1e-6) {
      tx = dx / len;
      ty = dy / len;
      return true;
    }
  }
  const double h = ref[i].heading();
  tx = std::cos(h);
  ty = std::sin(h);
  return true;
}

int ClosestValidRefIndexFrom(const std::vector<proto::ReferencePoint> &ref,
                             double x, double y, int min_index) {
  int best = -1;
  double best_d = 1e300;
  const int start = std::max(0, min_index);
  for (int i = start; i < static_cast<int>(ref.size()); ++i) {
    if (!ref[i].valid())
      continue;
    const double d = std::hypot(ref[i].x() - x, ref[i].y() - y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

double RefPathLateralError(const std::vector<proto::ReferencePoint> &ref,
                           double x, double y, int center_idx) {
  if (ref.empty())
    return 0.0;
  const int n = static_cast<int>(ref.size());
  const int i0 = std::max(0, center_idx - 8);
  const int i1 = std::min(n - 1, center_idx + 24);
  double best = 1e300;
  for (int i = i0; i <= i1; ++i) {
    if (!ref[i].valid())
      continue;
    best = std::min(best, std::hypot(ref[i].x() - x, ref[i].y() - y));
    if (i + 1 <= i1 && ref[i + 1].valid()) {
      const double ax = ref[i].x();
      const double ay = ref[i].y();
      const double bx = ref[i + 1].x();
      const double by = ref[i + 1].y();
      const double dx = bx - ax;
      const double dy = by - ay;
      const double len2 = dx * dx + dy * dy;
      if (len2 > 1e-6) {
        const double t = std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0,
                                    1.0);
        best = std::min(best, std::hypot(x - (ax + t * dx), y - (ay + t * dy)));
      }
    }
  }
  return std::isfinite(best) ? best : 0.0;
}

double LaneOffsetPenalty(double lateral_offset_m) {
  const double a = std::fabs(lateral_offset_m);
  if (a < 0.25)
    return 0.0;
  const double excess = a - 0.25;
  return excess * excess * 4.0;
}

double LaneBoundaryPenalty(double dist_to_left_boundary_m,
                         double dist_to_right_boundary_m) {
  double penalty = 0.0;
  if (dist_to_left_boundary_m < 0.6) {
    const double e = 0.6 - dist_to_left_boundary_m;
    penalty += e * e * 8.0;
  }
  if (dist_to_right_boundary_m < 0.6) {
    const double e = 0.6 - dist_to_right_boundary_m;
    penalty += e * e * 8.0;
  }
  if (dist_to_left_boundary_m < 0.0)
    penalty += 50.0;
  if (dist_to_right_boundary_m < 0.0)
    penalty += 50.0;
  return penalty;
}

void SimulateOneStep(proto::VehicleState &ego, double cmd_accel,
                     double cmd_steer, double step_dt,
                     const proto::VehicleParams &p) {
  proto::PlanCommand cmd;
  cmd.set_target_acceleration(cmd_accel);
  cmd.set_steering_angle(cmd_steer);
  StepVehicle(&ego, cmd, step_dt, p);
}

OBB MakeEgoObb(const proto::VehicleState &ego, const proto::VehicleParams &p,
               double inflate = 0.0) {
  OBB ego_box;
  const double d = p.length() / 2.0 - p.rear_overhang();
  ego_box.cx = ego.x() + d * std::cos(ego.heading());
  ego_box.cy = ego.y() + d * std::sin(ego.heading());
  ego_box.cz = 0.5 * 1.6;
  ego_box.heading = ego.heading();
  ego_box.half_length = std::max(0.5, p.length() * 0.5) + inflate;
  ego_box.half_width = std::max(0.3, p.width() * 0.5) + inflate;
  ego_box.half_height = 0.5 * 1.6;
  return ego_box;
}

OBB MakeNpcObbAt(const proto::NpcSnapshot &n, double t_ahead, double inflate) {
  OBB npc_box;
  npc_box.cx = n.x() + n.vx() * t_ahead;
  npc_box.cy = n.y() + n.vy() * t_ahead;
  npc_box.cz = n.z();
  npc_box.heading = n.heading();
  npc_box.half_length = std::max(0.5, n.length() * 0.5) + inflate;
  npc_box.half_width = std::max(0.3, n.width() * 0.5) + inflate;
  npc_box.half_height = std::max(0.2, n.height() * 0.5);
  return npc_box;
}

double LeaderLimitedSpeedDwa(const proto::VehicleState &ego,
                             const std::vector<proto::NpcSnapshot> &npcs,
                             double desired_speed_mps,
                             const proto::VehicleParams &p) {
  double safe_speed = desired_speed_mps;
  constexpr double kTimeHeadway = 1.2;
  constexpr double kMinGap = 2.5;
  const double ego_half = p.length() * 0.5 + 0.35;
  for (const auto &n : npcs) {
    const double dx = n.x() - ego.x();
    const double dy = n.y() - ego.y();
    const double c = std::cos(-ego.heading());
    const double s = std::sin(-ego.heading());
    const double fx = c * dx - s * dy;
    const double fy = s * dx + c * dy;
    if (fx < 0.0 || fx > 60.0)
      continue;
    if (std::fabs(fy) > (1.8 + 0.5 * n.width()))
      continue;
    const double gap = std::max(0.1, fx - 0.5 * n.length() - ego_half);
    const double npc_speed = std::hypot(n.vx(), n.vy());
    if (gap < 6.0) {
      const double comfortable =
          std::max(0.0, (gap - kMinGap) / std::max(0.5, kTimeHeadway));
      safe_speed =
          std::min(safe_speed, std::max(0.0, std::min(npc_speed, comfortable)));
    }
  }
  return safe_speed;
}
// 【终极修复 1】根据距离动态计算安全接近速度 (基于 v^2 = 2ad)
double ComputeApproachSpeed(double dist, double deadzone) {
  constexpr double kComfortableDecel = 1.5; // 期望的舒适减速度 1.5 m/s^2
  // 如果进入死区，返回0；否则返回平滑递减的速度曲线
  return std::sqrt(2.0 * kComfortableDecel * std::max(0.0, dist - deadzone));
}
// =====================================================================
// ReferenceTrajectoryPlanner
// =====================================================================
class ReferenceTrajectoryPlanner final : public Planner {
public:
  explicit ReferenceTrajectoryPlanner(const proto::PlannerInputs &inputs)
      : goal_(inputs.goal()), desired_speed_mps_(inputs.desired_speed_mps()),
        reference_points_(inputs.reference_points().begin(),
                          inputs.reference_points().end()),
        p_(inputs.ego_vehicle()), traj_cfg_(DefaultTrajectoryConfig(inputs)) {}

  std::string Name() const override { return "reference_tracker"; }

  proto::PlannerTrajectory
  Plan(const proto::PlannerObservation &obs) const override {
    const auto cmd = PlanStep(obs.ego(), {obs.npcs().begin(), obs.npcs().end()},
                              obs.frame_id());
    return BuildTrajectoryFromCommand(cmd, obs.ego(), p_, traj_cfg_);
  }

private:
  proto::PlanCommand PlanStep(const proto::VehicleState &ego,
                              const std::vector<proto::NpcSnapshot> &npcs,
                              int frame_id) const;
  double ComputeLeaderLimitedSpeed(const proto::VehicleState &ego,
                                   const std::vector<proto::NpcSnapshot> &npcs,
                                   double desired_speed_mps) const;

  proto::Pose2D goal_;
  double desired_speed_mps_ = 13.9;
  std::vector<proto::ReferencePoint> reference_points_;
  proto::VehicleParams p_;
  proto::PlannerConfig traj_cfg_;
};

proto::PlanCommand ReferenceTrajectoryPlanner::PlanStep(
    const proto::VehicleState &ego, const std::vector<proto::NpcSnapshot> &npcs,
    int frame_id) const {
  proto::PlanCommand cmd;
  constexpr double kGoalDeadzone = 1.5;

  int ref_idx = ClosestValidRefIndex(reference_points_, ego.x(), ego.y());
  if (ref_idx < 0 && !reference_points_.empty()) {
    const int idx = std::max(
        0, std::min(frame_id, static_cast<int>(reference_points_.size() - 1)));
    if (reference_points_[idx].valid())
      ref_idx = idx;
  }

  double target_x = goal_.x();
  double target_y = goal_.y();
  double target_speed = desired_speed_mps_;

  if (ref_idx >= 0) {
    target_speed = std::min(desired_speed_mps_,
                            std::max(0.0, reference_points_[ref_idx].speed()));
    const int look =
        std::min(ref_idx + 15, static_cast<int>(reference_points_.size() - 1));
    for (int j = look; j > ref_idx; --j) {
      if (reference_points_[j].valid()) {
        target_x = reference_points_[j].x();
        target_y = reference_points_[j].y();
        break;
      }
    }
    if (target_x == goal_.x() && target_y == goal_.y()) {
      target_x = reference_points_[ref_idx].x();
      target_y = reference_points_[ref_idx].y();
    }
  }

  target_speed = ComputeLeaderLimitedSpeed(ego, npcs, target_speed);

  const double dx = target_x - ego.x();
  const double dy = target_y - ego.y();
  const double dist = std::hypot(goal_.x() - ego.x(), goal_.y() - ego.y());
  const double dot_goal = (goal_.x() - ego.x()) * std::cos(ego.heading()) +
                          (goal_.y() - ego.y()) * std::sin(ego.heading());

  if (dist < kGoalDeadzone || (dist < 10.0 && dot_goal < 0.0)) {
    proto::PlanCommand stop_cmd;
    stop_cmd.set_desired_speed_mps(0.0);
    stop_cmd.set_target_acceleration(
        std::max(-p_.max_decel(), 2.0 * (0.0 - ego.speed())));
    stop_cmd.set_steering_angle(0.0);
    return stop_cmd;
  }

  // 【终极修复 3】同步运动学曲线
  double safe_speed = ComputeApproachSpeed(dist, kGoalDeadzone);
  double near_goal_speed = std::min(target_speed, safe_speed);

  cmd.set_desired_speed_mps(near_goal_speed);
  cmd.set_target_acceleration(1.2 * (near_goal_speed - ego.speed()));

  const double desired_heading = std::atan2(dy, dx);
  const double heading_error =
      std::atan2(std::sin(desired_heading - ego.heading()),
                 std::cos(desired_heading - ego.heading()));
  const double k_steer = 0.75;
  cmd.set_steering_angle(k_steer * heading_error);

  return cmd;
}

double ReferenceTrajectoryPlanner::ComputeLeaderLimitedSpeed(
    const proto::VehicleState &ego, const std::vector<proto::NpcSnapshot> &npcs,
    double desired_speed_mps) const {
  // [代码与原版相同，略去修改]
  double safe_speed = desired_speed_mps;
  constexpr double kTimeHeadway = 1.2;
  constexpr double kMinGap = 2.5;
  for (const auto &n : npcs) {
    const double dx = n.x() - ego.x();
    const double dy = n.y() - ego.y();
    const double c = std::cos(-ego.heading());
    const double s = std::sin(-ego.heading());
    const double fx = c * dx - s * dy;
    const double fy = s * dx + c * dy;
    if (fx < 0.0 || fx > 60.0)
      continue;
    if (std::fabs(fy) > (1.8 + 0.5 * n.width()))
      continue;
    const double gap = std::max(0.1, fx - 0.5 * n.length() - 2.25);
    const double npc_speed = std::hypot(n.vx(), n.vy());
    if (gap < 6.0) {
      const double comfortable =
          std::max(0.0, (gap - kMinGap) / std::max(0.5, kTimeHeadway));
      safe_speed =
          std::min(safe_speed, std::max(0.0, std::min(npc_speed, comfortable)));
    }
  }
  return safe_speed;
}

// =====================================================================
// GoalSeekPlanner
// =====================================================================
class GoalSeekPlanner final : public Planner {
public:
  explicit GoalSeekPlanner(const proto::PlannerInputs &inputs)
      : goal_(inputs.goal()), desired_speed_mps_(inputs.desired_speed_mps()),
        p_(inputs.ego_vehicle()), traj_cfg_(DefaultTrajectoryConfig(inputs)) {}

  std::string Name() const override { return "goal_seek"; }

  proto::PlannerTrajectory
  Plan(const proto::PlannerObservation &obs) const override {
    proto::PlanCommand cmd;
    const double dx = goal_.x() - obs.ego().x();
    const double dy = goal_.y() - obs.ego().y();
    const double dist = std::hypot(dx, dy);
    const double dot_goal =
        dx * std::cos(obs.ego().heading()) + dy * std::sin(obs.ego().heading());

    constexpr double kGoalDeadzone = 1.5;

    // 终点死区与过冲拦截
    if (dist < kGoalDeadzone || (dist < 10.0 && dot_goal < 0.0)) {
      proto::PlanCommand stop_cmd;
      stop_cmd.set_desired_speed_mps(0.0);
      stop_cmd.set_target_acceleration(
          std::max(-p_.max_decel(), 2.0 * (0.0 - obs.ego().speed())));
      stop_cmd.set_steering_angle(0.0);
      return BuildTrajectoryFromCommand(stop_cmd, obs.ego(), p_, traj_cfg_);
    }

    // 【终极修复 2】利用运动学曲线限制目标速度，远距离就自然降速，杜绝冲线绕圈
    double safe_speed = ComputeApproachSpeed(dist, kGoalDeadzone);
    double v_tgt = std::min(desired_speed_mps_, safe_speed);
    cmd.set_desired_speed_mps(v_tgt);
    cmd.set_target_acceleration(1.2 * (v_tgt - obs.ego().speed()));

    const double desired_heading = std::atan2(dy, dx);
    const double heading_error =
        std::atan2(std::sin(desired_heading - obs.ego().heading()),
                   std::cos(desired_heading - obs.ego().heading()));
    cmd.set_steering_angle(0.9 * heading_error);

    return BuildTrajectoryFromCommand(cmd, obs.ego(), p_, traj_cfg_);
  }

private:
  proto::Pose2D goal_;
  double desired_speed_mps_ = 13.9;
  proto::VehicleParams p_;
  proto::PlannerConfig traj_cfg_;
};

// =====================================================================
// LocalDwaPlanner
// =====================================================================
class LocalDwaPlanner final : public Planner {
public:
  explicit LocalDwaPlanner(const proto::PlannerInputs &inputs)
      : goal_(inputs.goal()), desired_speed_mps_(inputs.desired_speed_mps()),
        reference_points_(inputs.reference_points().begin(),
                          inputs.reference_points().end()),
        p_(inputs.ego_vehicle()), traj_cfg_(DefaultTrajectoryConfig(inputs)) {}

  std::string Name() const override { return "local_dwa"; }

  proto::PlannerTrajectory
  Plan(const proto::PlannerObservation &obs) const override {
    const auto cmd =
        PlanStep(obs.ego(), {obs.npcs().begin(), obs.npcs().end()},
                 obs.frame_id(), obs.road());
    return BuildTrajectoryFromCommand(cmd, obs.ego(), p_, traj_cfg_);
  }

private:
  static constexpr double kRollDt = 0.05;
  static constexpr int kHorizonSteps = 12;
  static constexpr double kNpcInflate = 0.35;

  proto::PlanCommand PlanStep(const proto::VehicleState &ego,
                              const std::vector<proto::NpcSnapshot> &npcs,
                              int frame_id,
                              const proto::RoadContext &road) const;
  proto::PlanCommand FallbackPurePursuit(const proto::VehicleState &ego,
                                         double target_x, double target_y,
                                         double target_speed,
                                         const proto::RoadContext &road) const;

  proto::Pose2D goal_;
  double desired_speed_mps_ = 13.9;
  std::vector<proto::ReferencePoint> reference_points_;
  proto::VehicleParams p_;
  proto::PlannerConfig traj_cfg_;
};

proto::PlanCommand
LocalDwaPlanner::FallbackPurePursuit(const proto::VehicleState &ego,
                                     double target_x, double target_y,
                                     double target_speed,
                                     const proto::RoadContext &road) const {
  proto::PlanCommand cmd;
  
  // 1. 计算自车到【最终终点】(goal_) 的相对状态，用于判断是否该彻底停车
  const double dx_goal = goal_.x() - ego.x();
  const double dy_goal = goal_.y() - ego.y();
  const double dist_goal = std::hypot(dx_goal, dy_goal);
  
  // 使用点积判断终点是在车头前方 (正数) 还是车尾后方 (负数)
  const double dot_goal = dx_goal * std::cos(ego.heading()) + 
                          dy_goal * std::sin(ego.heading());

  constexpr double kGoalDeadzone = 1.5;

  // 2. 【终点拦截状态机】防止过冲、防止原地打转和倒车
  // 条件：进入 1.5m 死区，或者（距离 10m 以内且终点已经跑到了车后方）
  if (dist_goal < kGoalDeadzone || (dist_goal < 10.0 && dot_goal < 0.0)) {
    proto::PlanCommand stop_cmd;
    stop_cmd.set_desired_speed_mps(0.0);
    // 阻尼式平稳刹车：加速度与当前车速成反比，车速归零时加速度也归零，绝不倒车
    stop_cmd.set_target_acceleration(std::max(-p_.max_decel(), 2.0 * (0.0 - ego.speed())));
    stop_cmd.set_steering_angle(0.0); // 停车时方向盘回正
    return stop_cmd;
  }

  // 3. 【纵向控制】结合运动学曲线进行平滑降速
  // 调用定义在文件顶部的 ComputeApproachSpeed 函数，获取当前距离下的物理安全速度上限
  double safe_speed = ComputeApproachSpeed(dist_goal, kGoalDeadzone);
  
  // 实际的目标速度，不能超过安全曲线，也不能超过传入的 target_speed (参考线限速)
  double v_tgt = std::min(target_speed, safe_speed);

  cmd.set_desired_speed_mps(v_tgt);
  // P 控制器输出加速度
  cmd.set_target_acceleration(1.2 * (v_tgt - ego.speed()));

  // 4. 【横向控制】纯跟踪 (Pure Pursuit) 转向逻辑
  // 注意：转向追踪的是传入的预瞄点 (target_x, target_y)，而不是最终的 goal_
  const double dx_target = target_x - ego.x();
  const double dy_target = target_y - ego.y();
  const double desired_heading = std::atan2(dy_target, dx_target);
  
  // 将角度误差限制在 [-pi, pi] 之间
  const double heading_error =
      std::atan2(std::sin(desired_heading - ego.heading()),
                 std::cos(desired_heading - ego.heading()));
                 
  double steer = 0.85 * heading_error;
  if (road.closest_lane_id() != 0) {
    steer -= 0.4 * road.lateral_offset_m();
    steer = std::clamp(steer, -p_.max_steer(), p_.max_steer());
  }
  cmd.set_steering_angle(steer);

  return cmd;
}

proto::PlanCommand
LocalDwaPlanner::PlanStep(const proto::VehicleState &ego,
                          const std::vector<proto::NpcSnapshot> &npcs,
                          int frame_id, const proto::RoadContext &road) const {
  constexpr int kSteerSamples = 13;
  constexpr int kAccelSamples = 7;
  constexpr double kGoalDeadzone = 1.5;

  const double dx_goal = goal_.x() - ego.x();
  const double dy_goal = goal_.y() - ego.y();
  const double dist_goal = std::hypot(dx_goal, dy_goal);
  const double dot_goal =
      dx_goal * std::cos(ego.heading()) + dy_goal * std::sin(ego.heading());

  // 【终极修复】阻尼式彻底停车，且扩大捕捉圈到 10m 防过冲
  if (dist_goal < kGoalDeadzone || (dist_goal < 10.0 && dot_goal < 0.0)) {
    proto::PlanCommand stop_cmd;
    stop_cmd.set_desired_speed_mps(0.0);
    stop_cmd.set_target_acceleration(
        std::max(-p_.max_decel(), 2.0 * (0.0 - ego.speed())));
    stop_cmd.set_steering_angle(0.0);
    return stop_cmd;
  }

  const int min_ref_idx = std::max(0, frame_id - 3);
  int ref_idx =
      ClosestValidRefIndexFrom(reference_points_, ego.x(), ego.y(), min_ref_idx);
  if (ref_idx < 0) {
    ref_idx = ClosestValidRefIndex(reference_points_, ego.x(), ego.y());
  }
  if (ref_idx < 0 && !reference_points_.empty()) {
    const int idx = std::max(
        0, std::min(frame_id, static_cast<int>(reference_points_.size() - 1)));
    if (reference_points_[idx].valid())
      ref_idx = idx;
  }
  if (ref_idx >= 0 && ref_idx < min_ref_idx) {
    ref_idx = min_ref_idx;
  }

  double target_speed = desired_speed_mps_;
  if (ref_idx >= 0) {
    target_speed = std::min(desired_speed_mps_,
                            std::max(0.0, reference_points_[ref_idx].speed()));
  }
  if (dist_goal > 10.0) {
    target_speed = std::max(target_speed, 2.0);
  }
  target_speed = LeaderLimitedSpeedDwa(ego, npcs, target_speed, p_);

  double tx = std::cos(ego.heading());
  double ty = std::sin(ego.heading());

  if (ref_idx >= 0 && RefTangent(reference_points_, ref_idx, tx, ty)) {
  } else if (dist_goal > 1e-3) {
    tx = dx_goal / dist_goal;
    ty = dy_goal / dist_goal;
  }

  double target_x = ego.x() + tx * 12.0;
  double target_y = ego.y() + ty * 12.0;
  if (ref_idx >= 0) {
    const int look =
        std::min(ref_idx + 15, static_cast<int>(reference_points_.size() - 1));
    for (int j = look; j > ref_idx; --j) {
      if (reference_points_[j].valid()) {
        target_x = reference_points_[j].x();
        target_y = reference_points_[j].y();
        break;
      }
    }
  } else if (dist_goal > 1e-3) {
    target_x = goal_.x();
    target_y = goal_.y();
  }

  double best_cost = 1e300;
  double best_a = 0.0;
  double best_steer = 0.0;
  int best_first_hit = -1;
  bool found_free = false;

  for (int ai = 0; ai < kAccelSamples; ++ai) {
    const double t_a =
        static_cast<double>(ai) / static_cast<double>(kAccelSamples - 1);
    const double accel =
        -p_.max_decel() + t_a * (p_.max_accel() + p_.max_decel());

    for (int si = 0; si < kSteerSamples; ++si) {
      const double t_s =
          static_cast<double>(si) / static_cast<double>(kSteerSamples - 1);
      const double steer_cmd = -p_.max_steer() + t_s * (2.0 * p_.max_steer());

      proto::VehicleState roll;
      roll.CopyFrom(ego);
      int first_hit = kHorizonSteps + 1;
      bool hit = false;
      for (int h = 0; h < kHorizonSteps; ++h) {
        SimulateOneStep(roll, accel, steer_cmd, kRollDt, p_);
        const double t_npc = static_cast<double>(h + 1) * kRollDt;
        const OBB ego_b = MakeEgoObb(roll, p_, 0.08);
        for (const auto &n : npcs) {
          const OBB nb = MakeNpcObbAt(n, t_npc, kNpcInflate);
          if (Overlap(ego_b, nb)) {
            first_hit = h;
            hit = true;
            break;
          }
        }
        if (hit)
          break;
      }

      const double prog = (roll.x() - ego.x()) * tx + (roll.y() - ego.y()) * ty;
      const double lat_err =
          std::fabs(-ty * (roll.x() - ego.x()) + tx * (roll.y() - ego.y()));
      const double ref_lat =
          RefPathLateralError(reference_points_, roll.x(), roll.y(), ref_idx);
      double lane_cost = 0.55 * lat_err + 1.1 * ref_lat;
      if (road.closest_lane_id() != 0) {
        const double dlat_path =
            -ty * (roll.x() - ego.x()) + tx * (roll.y() - ego.y());
        const double pred_lat = road.lateral_offset_m() + dlat_path;
        lane_cost += 2.0 * LaneOffsetPenalty(pred_lat);
        const double pred_left_b = road.dist_to_left_boundary_m() - dlat_path;
        const double pred_right_b = road.dist_to_right_boundary_m() + dlat_path;
        lane_cost +=
            LaneBoundaryPenalty(pred_left_b, pred_right_b);
      }
      const double cost_track = lane_cost - 0.85 * prog +
                                0.35 * std::fabs(steer_cmd) +
                                0.04 * std::fabs(accel);

      if (!hit) {
        if (!found_free || cost_track < best_cost - 1e-9 ||
            (std::fabs(cost_track - best_cost) < 1e-9 &&
             std::fabs(steer_cmd) < std::fabs(best_steer))) {
          found_free = true;
          best_cost = cost_track;
          best_a = accel;
          best_steer = steer_cmd;
          best_first_hit = kHorizonSteps + 1;
        }
      } else if (!found_free) {
        if (best_first_hit < 0 || first_hit > best_first_hit ||
            (first_hit == best_first_hit && cost_track < best_cost)) {
          best_cost = cost_track;
          best_a = accel;
          best_steer = steer_cmd;
          best_first_hit = first_hit;
        }
      }
    }
  }

  // ... (保留 DWA 采样循环逻辑) ...

  if (!found_free) {
    return FallbackPurePursuit(ego, target_x, target_y, target_speed, road);
  }

  proto::PlanCommand cmd;

  // 【终极修复 4】强制钳制 DWA 的纵向输出
  // DWA 的代价函数是不考虑终点减速的，我们必须用运动学曲线强行接管
  double safe_speed = ComputeApproachSpeed(dist_goal, kGoalDeadzone);

  // 如果车速过快，或者进入了 10 米准备停车区，强行切断 DWA 油门，转为 P
  // 控制器刹车
  if (ego.speed() > safe_speed || dist_goal < 10.0) {
    double v_tgt = std::min(target_speed, safe_speed);
    cmd.set_target_acceleration(1.5 * (v_tgt - ego.speed()));
    cmd.set_desired_speed_mps(v_tgt);
  } else {
    // 正常巡航听从 DWA
    cmd.set_target_acceleration(best_a);
    if (ego.speed() < 0.5 && dist_goal > 5.0 &&
        cmd.target_acceleration() < 0.3) {
      cmd.set_target_acceleration(0.5);
    }
    cmd.set_desired_speed_mps(target_speed);
  }

  double steer = best_steer;
  if (road.closest_lane_id() != 0 &&
      std::fabs(road.lateral_offset_m()) > 0.35) {
    steer -= 0.25 * road.lateral_offset_m();
    steer = std::clamp(steer, -p_.max_steer(), p_.max_steer());
  }
  cmd.set_steering_angle(steer);
  return cmd;
}

const bool kRegisteredReferenceTracker =
    RegisterPlanner("reference_tracker", [](const proto::PlannerInputs &in) {
      return std::make_unique<ReferenceTrajectoryPlanner>(in);
    });
const bool kRegisteredGoalSeek =
    RegisterPlanner("goal_seek", [](const proto::PlannerInputs &in) {
      return std::make_unique<GoalSeekPlanner>(in);
    });
const bool kRegisteredLocalDwa =
    RegisterPlanner("local_dwa", [](const proto::PlannerInputs &in) {
      return std::make_unique<LocalDwaPlanner>(in);
    });

} // namespace

std::unique_ptr<Planner> CreatePlanner(const std::string &planner_name,
                                       const proto::PlannerInputs &inputs,
                                       std::string *error) {
  (void)kRegisteredReferenceTracker;
  (void)kRegisteredGoalSeek;
  (void)kRegisteredLocalDwa;
  auto it = Registry().find(planner_name);
  if (it == Registry().end()) {
    if (error) {
      *error = "unknown planner: " + planner_name;
    }
    return nullptr;
  }
  return it->second(inputs);
}

std::vector<std::string> AvailablePlannerNames() {
  std::vector<std::string> names;
  names.reserve(Registry().size());
  for (const auto &kv : Registry()) {
    names.push_back(kv.first);
  }
  std::sort(names.begin(), names.end());
  return names;
}

} // namespace hyw_sim