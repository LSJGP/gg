#pragma once

#include "src/grading/metric_base.h"
#include "src/grading/metric_register.h"

namespace grading_mini {

// Detects NPC–ego OBB overlap from MetricFrameInput and classifies regulatory
// exemption. The metric PASSES only if there is no non-exempt collision over
// the whole run. Exempt examples:
//   - rear_end_on_slow_ego : NPC rear-ends a stationary / slow ego
//   - forced_cut_in        : NPC laterally enters ego's lane
//   - wrong_way_head_on    : NPC approaches head-on in the wrong direction
class RegulatoryCollisionChecker : public MetricBase {
 public:
  absl::Status Init(const google::protobuf::Message* config) override;

  absl::Status CalculateOneFrame(
      const MetricFrameInput& input,
      const std::deque<MetricFrameOutput>& history,
      MetricFrameOutput* output) override;

  absl::StatusOr<MetricSummary> SummarizeResult(
      const std::deque<MetricFrameOutput>& history) override;

 private:
  int total_frames_ = 0;
  int collision_frames_ = 0;
  int exempt_frames_ = 0;
  int non_exempt_frames_ = 0;
};

}  // namespace grading_mini
