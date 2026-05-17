#pragma once

#include <functional>
#include <vector>

#include "cpp/planner.h"
#include "cpp/types.h"

namespace hyw_sim {

struct WorldConfig {
  double dt = 0.1;
  double max_seconds = 0.0;
  bool stop_on_collision = false;
  double initial_ego_speed_mps = 0.0;
};

class WorldSimulator {
 public:
  explicit WorldSimulator(Scenario scenario, VehicleParams params);

  /// Optional `on_frame` invoked each timestep after `FrameRecord` is built (same thread
  /// as `Run`). Use for async grading enqueue; keep work minimal to avoid slowing sim.
  std::vector<FrameRecord> Run(const Planner& planner, const WorldConfig& cfg,
                              const std::function<void(const FrameRecord&)>* on_frame =
                                  nullptr);

 private:
  std::vector<NPCSnapshot> NPCsAtTime(double t) const;
  CollisionInfo DetectCollision(const VehicleState& ego,
                                const std::vector<NPCSnapshot>& npcs) const;
  void StepVehicle(VehicleState* ego, const PlanCommand& cmd, double dt) const;
  std::vector<NPCSnapshot> NPCsAtIndex(int idx) const;
  std::vector<NPCSnapshot> InterpNPCs(int lo, int hi, double a) const;

  Scenario scenario_;
  VehicleParams params_;
};

}  // namespace hyw_sim
