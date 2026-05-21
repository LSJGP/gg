#pragma once

#include <limits>

#include "proto/grading/scene.pb.h"
#include "src/grading/metric_base.h"
#include "src/grading/metric_register.h"

namespace grading_mini {

class DrivableAreaChecker : public MetricBase {
 public:
  absl::Status Init(const google::protobuf::Message* config) override;

  absl::Status CalculateOneFrame(
      const MetricFrameInput& input,
      const std::deque<MetricFrameOutput>& history,
      MetricFrameOutput* output) override;

  absl::StatusOr<MetricSummary> SummarizeResult(
      const std::deque<MetricFrameOutput>& history) override;

 private:
  double min_clearance_m_ = 0.35;
  bool check_center_only_ = false;
  bool has_scene_map_ = false;
  bool warned_no_map_ = false;
  proto::SceneMap scene_map_;
  int total_frames_ = 0;
  int violation_frames_ = 0;
  int skipped_frames_ = 0;
  double last_min_dist_m_ = std::numeric_limits<double>::infinity();
};

}  // namespace grading_mini
