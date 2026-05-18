#pragma once

#include <memory>
#include <string>
#include <vector>

#include "proto/sim/runtime.pb.h"

namespace hyw_sim {

class Planner {
 public:
  virtual ~Planner() = default;
  virtual std::string Name() const = 0;
  virtual proto::PlannerTrajectory Plan(const proto::PlannerObservation& obs) const = 0;
};

std::unique_ptr<Planner> CreatePlanner(const std::string& planner_name,
                                       const proto::PlannerInputs& inputs,
                                       std::string* error);
std::vector<std::string> AvailablePlannerNames();

}  // namespace hyw_sim
