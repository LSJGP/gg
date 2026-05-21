#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "cpp/dynamic_source.h"
#include "cpp/lane_graph.h"
#include "cpp/planner.h"
#include "proto/sim/runtime.pb.h"
#include "proto/sim/scenario.pb.h"

namespace hyw_sim {

struct WorldStepHooks {
  std::function<void(const proto::PlannerObservation&)> on_observation;
  std::function<void(const proto::PlanCommand&, const proto::PlannerTrajectory&)>
      on_plan;
  std::function<void(const proto::FrameRecord&)> on_frame;
};

class WorldSimulator {
 public:
  WorldSimulator(const proto::ScenarioMeta& meta,
                 std::unique_ptr<DynamicNpcSource> dynamic,
                 const LaneGraph& lane_graph, proto::VehicleParams params);

  std::vector<proto::FrameRecord> Run(const Planner& planner,
                                      const proto::WorldConfig& cfg,
                                      const WorldStepHooks* hooks = nullptr);

  int64_t stream_io_us() const;

 private:
  std::vector<proto::NpcSnapshot> NPCsAtTime(double t) const;
  std::vector<proto::NpcSnapshot> NPCsAtIndex(int idx) const;

  proto::ScenarioMeta meta_;
  std::unique_ptr<DynamicNpcSource> dynamic_;
  const LaneGraph& lane_graph_;
  proto::VehicleParams params_;
};

}  // namespace hyw_sim
