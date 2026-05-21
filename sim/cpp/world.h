#pragma once

#include <functional>
#include <vector>

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
  WorldSimulator(const proto::ScenarioMeta& meta, const proto::DynamicObjects& dynamic,
                 const LaneGraph& lane_graph, proto::VehicleParams params);

  std::vector<proto::FrameRecord> Run(const Planner& planner,
                                      const proto::WorldConfig& cfg,
                                      const WorldStepHooks* hooks = nullptr);

 private:
  std::vector<proto::NpcSnapshot> NPCsAtTime(double t) const;
  std::vector<proto::NpcSnapshot> NPCsAtIndex(int idx) const;
  std::vector<proto::NpcSnapshot> InterpNPCs(int lo, int hi, double a) const;

  proto::ScenarioMeta meta_;
  proto::DynamicObjects dynamic_;
  const LaneGraph& lane_graph_;
  proto::VehicleParams params_;
};

}  // namespace hyw_sim
