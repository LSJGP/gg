#include "cpp/world.h"

#include <cmath>

#include "cpp/map_query.h"
#include "cpp/trajectory_tracker.h"

namespace hyw_sim {
namespace {

double WrapAngle(double rad) {
  return std::atan2(std::sin(rad), std::cos(rad));
}

proto::NpcSnapshot ToSnapshot(const proto::Track& tr, const proto::TrackState& st) {
  proto::NpcSnapshot out;
  out.set_id(tr.id());
  out.set_object_type(tr.object_type());
  out.set_x(st.x());
  out.set_y(st.y());
  out.set_z(st.z());
  out.set_heading(st.yaw());
  out.set_vx(st.vx());
  out.set_vy(st.vy());
  out.set_length(st.length());
  out.set_width(st.width());
  out.set_height(st.height());
  return out;
}

}  // namespace

WorldSimulator::WorldSimulator(const proto::ScenarioMeta& meta,
                               const proto::DynamicObjects& dynamic,
                               const LaneGraph& lane_graph,
                               proto::VehicleParams params)
    : meta_(meta),
      dynamic_(dynamic),
      lane_graph_(lane_graph),
      params_(std::move(params)) {}

std::vector<proto::FrameRecord> WorldSimulator::Run(const Planner& planner,
                                                    const proto::WorldConfig& cfg,
                                                    const WorldStepHooks* hooks) {
  std::vector<proto::FrameRecord> out;
  if (dynamic_.timestamps_seconds_size() == 0) return out;

  const double t0 = dynamic_.timestamps_seconds(0);
  double total_seconds = cfg.max_seconds();
  if (total_seconds <= 0.0 && dynamic_.timestamps_seconds_size() > 1) {
    total_seconds = dynamic_.timestamps_seconds(
                        dynamic_.timestamps_seconds_size() - 1) -
                    t0;
  }
  if (total_seconds <= 0.0) total_seconds = 5.0;

  proto::VehicleState ego;
  ego.set_x(meta_.init_pose().x());
  ego.set_y(meta_.init_pose().y());
  ego.set_heading(meta_.init_pose().yaw());
  ego.set_speed(std::max(0.0, cfg.initial_ego_speed_mps()));

  const int n_steps = std::max(1, static_cast<int>(std::round(total_seconds / cfg.dt())));
  out.reserve(static_cast<size_t>(n_steps));
  for (int step = 0; step < n_steps; ++step) {
    const double t = static_cast<double>(step) * cfg.dt();
    const double scenario_time = t0 + t;
    const auto npcs = NPCsAtTime(scenario_time);
    const auto road = BuildRoadContext(lane_graph_, ego);

    proto::PlannerObservation obs;
    obs.set_frame_id(step);
    obs.set_timestamp_us(static_cast<int64_t>(std::llround(scenario_time * 1e6)));
    obs.set_dt(cfg.dt());
    *obs.mutable_ego() = ego;
    for (const auto& n : npcs) {
      *obs.add_npcs() = n;
    }
    *obs.mutable_road() = road;

    if (hooks && hooks->on_observation) {
      hooks->on_observation(obs);
    }

    const proto::PlannerTrajectory trajectory = planner.Plan(obs);
    const proto::PlanCommand cmd = TrajectoryToCommand(trajectory, cfg.dt());
    if (hooks && hooks->on_plan) {
      hooks->on_plan(cmd, trajectory);
    }
    StepVehicle(&ego, cmd, cfg.dt(), params_);

    proto::FrameRecord r;
    r.set_frame_id(step);
    r.set_timestamp_us(obs.timestamp_us());
    *r.mutable_ego() = ego;
    *r.mutable_command() = cmd;
    r.set_num_npcs(static_cast<int32_t>(npcs.size()));
    for (const auto& n : npcs) {
      *r.add_npcs() = n;
    }
    *r.mutable_road() = road;
    *r.mutable_planned_trajectory() = trajectory;
    out.push_back(r);
    if (hooks && hooks->on_frame) {
      hooks->on_frame(r);
    }
  }
  return out;
}

std::vector<proto::NpcSnapshot> WorldSimulator::NPCsAtTime(double t) const {
  const auto& ts = dynamic_.timestamps_seconds();
  if (t <= ts.Get(0)) return NPCsAtIndex(0);
  if (t >= ts.Get(ts.size() - 1)) {
    return NPCsAtIndex(static_cast<int>(ts.size() - 1));
  }
  int lo = 0;
  int hi = static_cast<int>(ts.size() - 1);
  while (lo + 1 < hi) {
    const int mid = (lo + hi) / 2;
    if (ts.Get(mid) <= t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  const double seg_len = ts.Get(hi) - ts.Get(lo);
  const double a = (seg_len < 1e-9) ? 0.0 : (t - ts.Get(lo)) / seg_len;
  return InterpNPCs(lo, hi, a);
}

std::vector<proto::NpcSnapshot> WorldSimulator::NPCsAtIndex(int idx) const {
  std::vector<proto::NpcSnapshot> out;
  for (const auto& tr : dynamic_.tracks()) {
    if (tr.is_sdc() || idx < 0 || idx >= tr.states_size()) continue;
    const auto& st = tr.states(idx);
    if (!st.valid()) continue;
    out.push_back(ToSnapshot(tr, st));
  }
  return out;
}

std::vector<proto::NpcSnapshot> WorldSimulator::InterpNPCs(int lo, int hi,
                                                            double a) const {
  std::vector<proto::NpcSnapshot> out;
  for (const auto& tr : dynamic_.tracks()) {
    if (tr.is_sdc()) continue;
    if (lo >= tr.states_size() || hi >= tr.states_size()) {
      continue;
    }
    const auto& s0 = tr.states(lo);
    const auto& s1 = tr.states(hi);
    if (s0.valid() && s1.valid()) {
      proto::NpcSnapshot n;
      n.set_id(tr.id());
      n.set_object_type(tr.object_type());
      n.set_x(s0.x() + a * (s1.x() - s0.x()));
      n.set_y(s0.y() + a * (s1.y() - s0.y()));
      n.set_z(s0.z() + a * (s1.z() - s0.z()));
      n.set_heading(WrapAngle(s0.yaw() + a * WrapAngle(s1.yaw() - s0.yaw())));
      n.set_vx(s0.vx() + a * (s1.vx() - s0.vx()));
      n.set_vy(s0.vy() + a * (s1.vy() - s0.vy()));
      n.set_length(s1.length());
      n.set_width(s1.width());
      n.set_height(s1.height());
      out.push_back(n);
    } else if (s0.valid()) {
      out.push_back(ToSnapshot(tr, s0));
    } else if (s1.valid()) {
      out.push_back(ToSnapshot(tr, s1));
    }
  }
  return out;
}

}  // namespace hyw_sim
