#include "src/grading/metrics/safety/regulatory_collision_checker.h"

#include "spdlog/spdlog.h"
#include "src/grading/collision_detector.h"

namespace grading_mini {

REGISTER_METRIC(RegulatoryCollisionChecker, "regulatory_collision_checker");

absl::Status RegulatoryCollisionChecker::Init(
    const google::protobuf::Message* /*config*/) {
  SPDLOG_INFO(
      "RegulatoryCollisionChecker init: detects collision from ego/npc geometry");
  return absl::OkStatus();
}

absl::Status RegulatoryCollisionChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  total_frames_++;

  const CollisionDetectResult det = DetectRegulatoryCollision(input);
  if (!det.collided) {
    output->set_bool_value(true);
    return absl::OkStatus();
  }

  collision_frames_++;
  if (det.exempt) {
    exempt_frames_++;
    output->set_bool_value(true);
    SPDLOG_WARN(
        "Frame {}: collision EXEMPT ({}) other_id={} ego_v={:.2f} rel_v={:.2f}",
        input.frame_id(), det.exempt_reason, det.other_id, det.ego_speed_mps,
        det.relative_speed_mps);
  } else {
    non_exempt_frames_++;
    output->set_bool_value(false);
    SPDLOG_ERROR(
        "Frame {}: COLLISION at_fault kind={} other_id={} ego_v={:.2f} "
        "rel_v={:.2f} approach={:.1f}deg",
        input.frame_id(), det.kind, det.other_id, det.ego_speed_mps,
        det.relative_speed_mps, det.approach_angle_deg);
  }
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> RegulatoryCollisionChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  MetricSummary summary;
  summary.set_metric_name(name_);
  summary.set_passed(non_exempt_frames_ == 0);
  summary.set_detail(
      "non_exempt=" + std::to_string(non_exempt_frames_) +
      " exempt=" + std::to_string(exempt_frames_) +
      " total_collision_frames=" + std::to_string(collision_frames_) + "/" +
      std::to_string(total_frames_));
  return summary;
}

}  // namespace grading_mini
