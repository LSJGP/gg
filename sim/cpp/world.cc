#include "cpp/world.h"

#include <algorithm>
#include <cmath>

#include "cpp/geometry.h"

namespace hyw_sim {
namespace {

constexpr double kExemptRearEndEgoSpeedMax = 5.0;
constexpr double kExemptRearEndRelSpeedMin = 2.0;
constexpr double kExemptCutInLatVelMin = 0.5;
constexpr double kExemptHeadOnAngleDeg = 135.0;

double WrapAngle(double rad) {
  return std::atan2(std::sin(rad), std::cos(rad));
}

NPCSnapshot ToSnapshot(const Track& tr, const TrackState& st) {
  NPCSnapshot out;
  out.id = tr.id;
  out.object_type = tr.object_type;
  out.x = st.x;
  out.y = st.y;
  out.z = st.z;
  out.heading = st.yaw;
  out.vx = st.vx;
  out.vy = st.vy;
  out.length = st.length;
  out.width = st.width;
  out.height = st.height;
  return out;
}

CollisionInfo ClassifyCollision(const VehicleState& ego, const NPCSnapshot& n) {
  const double dx = n.x - ego.x;
  const double dy = n.y - ego.y;
  const double c = std::cos(-ego.heading);
  const double s = std::sin(-ego.heading);
  const double fx = c * dx - s * dy;
  const double fy = s * dx + c * dy;
  const double bearing = std::atan2(fy, fx);

  const double ego_speed = ego.speed;
  const double npc_speed = std::hypot(n.vx, n.vy);
  const double rvx = n.vx - ego.speed * std::cos(ego.heading);
  const double rvy = n.vy - ego.speed * std::sin(ego.heading);
  const double rel_speed = std::hypot(rvx, rvy);

  double approach_angle = 0.0;
  if (npc_speed > 0.5) {
    const double ndx = n.vx / npc_speed;
    const double ndy = n.vy / npc_speed;
    const double edx = std::cos(ego.heading);
    const double edy = std::sin(ego.heading);
    const double cosang = std::clamp(edx * ndx + edy * ndy, -1.0, 1.0);
    approach_angle = std::acos(cosang) * 180.0 / M_PI;
  }

  std::string zone;
  std::string kind;
  const double abs_b = std::fabs(bearing);
  if (abs_b < M_PI / 4.0) {
    zone = "front";
    kind = "ego_front_into_npc";
  } else if (abs_b > 3.0 * M_PI / 4.0) {
    zone = "rear";
    kind = "npc_rear_into_ego";
  } else {
    zone = "side";
    kind = "side_collision";
  }

  bool exempt = false;
  bool ego_at_fault = true;
  std::string exempt_reason;
  if (zone == "rear") {
    if (ego_speed < kExemptRearEndEgoSpeedMax &&
        (npc_speed - ego_speed) > kExemptRearEndRelSpeedMin) {
      exempt = true;
      ego_at_fault = false;
      exempt_reason = "rear_end_on_slow_ego";
    }
  } else if (zone == "side") {
    const double sx = -std::sin(ego.heading);
    const double sy = std::cos(ego.heading);
    const double lat_v_npc = sx * n.vx + sy * n.vy;
    if ((bearing > 0 && lat_v_npc < -kExemptCutInLatVelMin) ||
        (bearing < 0 && lat_v_npc > kExemptCutInLatVelMin)) {
      exempt = true;
      ego_at_fault = false;
      exempt_reason = "forced_cut_in";
    }
  } else {
    if (approach_angle > kExemptHeadOnAngleDeg) {
      exempt = true;
      ego_at_fault = false;
      exempt_reason = "wrong_way_head_on";
    }
  }

  CollisionInfo out;
  out.collided = true;
  out.other_id = n.id;
  out.kind = kind;
  out.ego_at_fault = ego_at_fault;
  out.exempt = exempt;
  out.exempt_reason = exempt_reason;
  out.relative_speed_mps = rel_speed;
  out.ego_speed_mps = ego_speed;
  out.approach_angle_deg = approach_angle;
  return out;
}

}  // namespace

WorldSimulator::WorldSimulator(Scenario scenario, VehicleParams params)
    : scenario_(std::move(scenario)), params_(params) {}

std::vector<FrameRecord> WorldSimulator::Run(
    const Planner& planner, const WorldConfig& cfg,
    const std::function<void(const FrameRecord&)>* on_frame) {
  std::vector<FrameRecord> out;
  if (scenario_.timestamps_seconds.empty()) return out;

  const double t0 = scenario_.timestamps_seconds.front();
  double total_seconds = cfg.max_seconds;
  if (total_seconds <= 0.0 && scenario_.timestamps_seconds.size() > 1) {
    total_seconds =
        scenario_.timestamps_seconds.back() - scenario_.timestamps_seconds.front();
  }
  if (total_seconds <= 0.0) total_seconds = 5.0;

  VehicleState ego;
  ego.x = scenario_.init_pose.x;
  ego.y = scenario_.init_pose.y;
  ego.heading = scenario_.init_pose.yaw;

  const int n_steps = std::max(1, static_cast<int>(std::round(total_seconds / cfg.dt)));
  out.reserve(n_steps);
  for (int step = 0; step < n_steps; ++step) {
    const double t = static_cast<double>(step) * cfg.dt;
    const double scenario_time = t0 + t;
    const auto npcs = NPCsAtTime(scenario_time);
    const PlanCommand cmd = planner.Plan(ego, npcs, cfg.dt, step);
    StepVehicle(&ego, cmd, cfg.dt);
    const CollisionInfo collision = DetectCollision(ego, npcs);

    FrameRecord r;
    r.frame_id = step;
    r.timestamp_us = static_cast<int64_t>(std::llround(scenario_time * 1e6));
    r.ego = ego;
    r.command = cmd;
    r.collision = collision;
    r.num_npcs = static_cast<int>(npcs.size());
    out.push_back(r);
    if (on_frame && *on_frame) {
      (*on_frame)(r);
    }
    if (cfg.stop_on_collision && collision.collided && !collision.exempt) {
      break;
    }
  }
  return out;
}

std::vector<NPCSnapshot> WorldSimulator::NPCsAtTime(double t) const {
  const auto& ts = scenario_.timestamps_seconds;
  if (t <= ts.front()) return NPCsAtIndex(0);
  if (t >= ts.back()) return NPCsAtIndex(static_cast<int>(ts.size() - 1));
  int lo = 0;
  int hi = static_cast<int>(ts.size() - 1);
  while (lo + 1 < hi) {
    const int mid = (lo + hi) / 2;
    if (ts[mid] <= t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  const double seg_len = ts[hi] - ts[lo];
  const double a = (seg_len < 1e-9) ? 0.0 : (t - ts[lo]) / seg_len;
  return InterpNPCs(lo, hi, a);
}

std::vector<NPCSnapshot> WorldSimulator::NPCsAtIndex(int idx) const {
  std::vector<NPCSnapshot> out;
  for (const auto& tr : scenario_.tracks) {
    if (tr.is_sdc || idx < 0 || idx >= static_cast<int>(tr.states.size())) continue;
    const auto& st = tr.states[idx];
    if (!st.valid) continue;
    out.push_back(ToSnapshot(tr, st));
  }
  return out;
}

std::vector<NPCSnapshot> WorldSimulator::InterpNPCs(int lo, int hi, double a) const {
  std::vector<NPCSnapshot> out;
  for (const auto& tr : scenario_.tracks) {
    if (tr.is_sdc) continue;
    if (lo >= static_cast<int>(tr.states.size()) || hi >= static_cast<int>(tr.states.size())) {
      continue;
    }
    const auto& s0 = tr.states[lo];
    const auto& s1 = tr.states[hi];
    if (s0.valid && s1.valid) {
      NPCSnapshot n;
      n.id = tr.id;
      n.object_type = tr.object_type;
      n.x = s0.x + a * (s1.x - s0.x);
      n.y = s0.y + a * (s1.y - s0.y);
      n.z = s0.z + a * (s1.z - s0.z);
      n.heading = WrapAngle(s0.yaw + a * WrapAngle(s1.yaw - s0.yaw));
      n.vx = s0.vx + a * (s1.vx - s0.vx);
      n.vy = s0.vy + a * (s1.vy - s0.vy);
      n.length = s1.length;
      n.width = s1.width;
      n.height = s1.height;
      out.push_back(n);
    } else if (s0.valid) {
      out.push_back(ToSnapshot(tr, s0));
    } else if (s1.valid) {
      out.push_back(ToSnapshot(tr, s1));
    }
  }
  return out;
}

CollisionInfo WorldSimulator::DetectCollision(const VehicleState& ego,
                                              const std::vector<NPCSnapshot>& npcs) const {
  OBB ego_box;
  const double d = params_.length / 2.0 - params_.rear_overhang;
  ego_box.cx = ego.x + d * std::cos(ego.heading);
  ego_box.cy = ego.y + d * std::sin(ego.heading);
  ego_box.cz = 0.5 * 1.6;  // Ego height defaults to sedan-like 1.6m.
  ego_box.heading = ego.heading;
  ego_box.half_length = params_.length / 2.0;
  ego_box.half_width = params_.width / 2.0;
  ego_box.half_height = 0.5 * 1.6;

  CollisionInfo best;
  bool has_best = false;
  for (const auto& n : npcs) {
    OBB npc_box;
    npc_box.cx = n.x;
    npc_box.cy = n.y;
    npc_box.cz = n.z;
    npc_box.heading = n.heading;
    npc_box.half_length = std::max(0.5, n.length * 0.5);
    npc_box.half_width = std::max(0.3, n.width * 0.5);
    npc_box.half_height = std::max(0.2, n.height * 0.5);
    if (!Overlap(ego_box, npc_box)) continue;
    const CollisionInfo info = ClassifyCollision(ego, n);
    if (!has_best || ((!info.exempt && best.exempt) ||
                      (info.exempt == best.exempt &&
                       info.relative_speed_mps > best.relative_speed_mps))) {
      best = info;
      has_best = true;
    }
  }
  if (!has_best) return CollisionInfo{};
  return best;
}

void WorldSimulator::StepVehicle(VehicleState* ego, const PlanCommand& cmd,
                                 double dt) const {
  const double accel =
      std::clamp(cmd.target_acceleration, -params_.max_decel, params_.max_accel);
  const double target_steer =
      std::clamp(cmd.steering_angle, -params_.max_steer, params_.max_steer);
  const double ds_max = params_.max_steer_rate * dt;
  const double steer =
      std::clamp(target_steer, ego->steer - ds_max, ego->steer + ds_max);

  const double new_speed = std::clamp(ego->speed + accel * dt, 0.0, params_.max_speed);
  const double wb = std::max(0.5, params_.wheelbase);
  const double new_heading = ego->heading + (new_speed / wb) * std::tan(steer) * dt;
  ego->x += new_speed * std::cos(new_heading) * dt;
  ego->y += new_speed * std::sin(new_heading) * dt;
  ego->heading = new_heading;
  ego->speed = new_speed;
  ego->acceleration = accel;
  ego->steer = steer;
}

}  // namespace hyw_sim
