#include "cpp/world.h"

#include <cmath>

#include "cpp/map_query.h"
#include "cpp/trajectory_tracker.h"

namespace hyw_sim {

WorldSimulator::WorldSimulator(const proto::ScenarioMeta& meta,
                               std::unique_ptr<DynamicNpcSource> dynamic,
                               const LaneGraph& lane_graph,
                               proto::VehicleParams params)
    : meta_(meta),
      dynamic_(std::move(dynamic)),
      lane_graph_(lane_graph),
      params_(std::move(params)) {}

std::vector<proto::FrameRecord> WorldSimulator::Run(const Planner& planner,
                                                    const proto::WorldConfig& cfg,
                                                    const WorldStepHooks* hooks) {
  std::vector<proto::FrameRecord> out;
  const auto& ts = dynamic_->timestamps();
  if (ts.empty()) return out;

  const double t0 = ts.front();
  double total_seconds = cfg.max_seconds();
  if (total_seconds <= 0.0 && ts.size() > 1) {
    total_seconds = ts.back() - t0;
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
  const auto& ts = dynamic_->timestamps();
  if (t <= ts.front()) return NPCsAtIndex(0);
  if (t >= ts.back()) {
    return NPCsAtIndex(static_cast<int>(ts.size() - 1));
  }
  int lo = 0;
  int hi = static_cast<int>(ts.size() - 1);
  while (lo + 1 < hi) {
    const int mid = (lo + hi) / 2;
    if (ts[static_cast<size_t>(mid)] <= t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  const double seg_len = ts[static_cast<size_t>(hi)] - ts[static_cast<size_t>(lo)];
  const double a = (seg_len < 1e-9) ? 0.0 : (t - ts[static_cast<size_t>(lo)]) / seg_len;
  return dynamic_->InterpNPCs(lo, hi, a);
}

std::vector<proto::NpcSnapshot> WorldSimulator::NPCsAtIndex(int idx) const {
  return dynamic_->NpcsAtIndex(idx);
}

int64_t WorldSimulator::stream_io_us() const {
  return dynamic_ ? dynamic_->stream_io_us() : 0;
}

}  // namespace hyw_sim
