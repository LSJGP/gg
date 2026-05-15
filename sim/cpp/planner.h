#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cpp/types.h"

namespace hyw_sim {

struct ReferencePoint {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  bool valid = false;
};

struct PlannerInputs {
  Pose2D goal;
  double desired_speed_mps = 13.9;
  std::vector<ReferencePoint> reference_points;
  /// Kinematics / footprint for planners that roll out the ego (e.g. local_dwa).
  VehicleParams ego_vehicle{};
};

class Planner {
 public:
  virtual ~Planner() = default;
  virtual std::string Name() const = 0;
  virtual PlanCommand Plan(const VehicleState& ego,
                           const std::vector<NPCSnapshot>& npcs, double dt,
                           int frame_id) const = 0;
};

std::unique_ptr<Planner> CreatePlanner(const std::string& planner_name,
                                       const PlannerInputs& inputs,
                                       std::string* error);
std::vector<std::string> AvailablePlannerNames();

}  // namespace hyw_sim
