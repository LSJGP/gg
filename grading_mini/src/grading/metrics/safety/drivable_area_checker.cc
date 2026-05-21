#include "src/grading/metrics/safety/drivable_area_checker.h"

#include <cmath>
#include <limits>

#include "proto/grading/metrics/safety_metric.pb.h"
#include "spdlog/spdlog.h"
#include "src/grading/metrics/safety/geometry.h"

namespace grading_mini {

REGISTER_METRIC(DrivableAreaChecker, "drivable_area_checker");

absl::Status DrivableAreaChecker::Init(const google::protobuf::Message* config) {
  if (config) {
    if (const auto* typed =
            dynamic_cast<const proto::DrivableAreaCheckerConfig*>(config)) {
      if (typed->min_clearance_m() > 0.0) {
        min_clearance_m_ = typed->min_clearance_m();
      }
      check_center_only_ = typed->check_center_only();
    }
  }
  SPDLOG_INFO(
      "DrivableAreaChecker init: min_clearance={:.2f}m check_center_only={}",
      min_clearance_m_, check_center_only_);
  return absl::OkStatus();
}

absl::Status DrivableAreaChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  total_frames_++;

  if (input.has_scene_map()) {
    scene_map_ = input.scene_map();
    has_scene_map_ = true;
  }

  bool ok = true;

  if (!has_scene_map_ || scene_map_.road_edges_size() == 0) {
    if (!warned_no_map_) {
      SPDLOG_WARN(
          "DrivableAreaChecker: missing scene_map or road_edges; skipping frames");
      warned_no_map_ = true;
    }
    skipped_frames_++;
    output->set_bool_value(true);
    return absl::OkStatus();
  }

  double min_dist = std::numeric_limits<double>::infinity();

  if (check_center_only_) {
    const auto& vs = input.vehicle_state();
    min_dist = MinDistToRoadEdges(scene_map_, vs.x(), vs.y());
  } else {
    const Obb2D box = MakeEgoObb(input);
    for (const auto& corner : ObbCorners(box)) {
      min_dist = std::min(min_dist,
                          MinDistToRoadEdges(scene_map_, corner[0], corner[1]));
    }
  }

  last_min_dist_m_ = min_dist;

  if (std::isfinite(min_dist) && min_dist < min_clearance_m_) {
    ok = false;
    violation_frames_++;
    SPDLOG_WARN("Frame {}: left drivable area min_dist={:.2f}m threshold={:.2f}m",
                input.frame_id(), min_dist, min_clearance_m_);
  }

  output->set_bool_value(ok);
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> DrivableAreaChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  MetricSummary summary;
  summary.set_metric_name(name_);
  summary.set_passed(violation_frames_ == 0);
  std::string detail = "violation_frames=" + std::to_string(violation_frames_) +
                       "/" + std::to_string(total_frames_) +
                       " skipped_frames=" + std::to_string(skipped_frames_);
  if (std::isfinite(last_min_dist_m_)) {
    detail += " last_min_dist_m=" + std::to_string(last_min_dist_m_);
  }
  summary.set_detail(detail);
  return summary;
}

}  // namespace grading_mini
