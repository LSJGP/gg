#include "src/planning/simple_planner.h"

#include <algorithm>

namespace grading_mini {

SimplePlanner::SimplePlanner(double max_speed_mps)
    : max_speed_mps_(max_speed_mps) {}

void SimplePlanner::Plan(proto::MetricFrameInput* input) const {
  if (!input) return;
  if (input->has_planning_command()) return;
  const double v = input->vehicle_state().speed();
  const double desired = std::min(v, max_speed_mps_);
  input->mutable_planning_command()->set_desired_speed_mps(desired);
}

}  // namespace grading_mini
