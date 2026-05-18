#pragma once

#include "proto/grading/scene.pb.h"
#include "src/grading/metric_base.h"
#include "src/grading/metric_register.h"

namespace grading_mini {

class LaneDepartureChecker : public MetricBase {
 public:
  absl::Status Init(const google::protobuf::Message* config) override;

  absl::Status CalculateOneFrame(
      const MetricFrameInput& input,
      const std::deque<MetricFrameOutput>& history,
      MetricFrameOutput* output) override;

  absl::StatusOr<MetricSummary> SummarizeResult(
      const std::deque<MetricFrameOutput>& history) override;

 private:
  double MinRoadEdgeDistance(const proto::SceneMap& map, double x, double y) const;
  double MinCornerRoadEdgeDistance(const MetricFrameInput& input) const;

  double min_road_edge_clearance_m_ = 0.35;
  double min_lane_boundary_clearance_m_ = 0.0;
  bool has_scene_map_ = false;
  proto::SceneMap scene_map_;
  int total_frames_ = 0;
  int violation_frames_ = 0;
};

}  // namespace grading_mini
