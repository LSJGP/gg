#include "src/grading/metrics/safety/regulatory_collision_checker.h"

#include "spdlog/spdlog.h"

namespace grading_mini {

REGISTER_METRIC(RegulatoryCollisionChecker, "regulatory_collision_checker");

absl::Status RegulatoryCollisionChecker::Init(
    const google::protobuf::Message* /*config*/) {
  SPDLOG_INFO(
      "RegulatoryCollisionChecker init: PASS iff no non-exempt collision");
  return absl::OkStatus();
}

absl::Status RegulatoryCollisionChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  total_frames_++;

  if (!input.has_collision_event() || !input.collision_event().collided()) {
    output->set_bool_value(true);
    return absl::OkStatus();
  }

  const auto& ev = input.collision_event();
  collision_frames_++;
  if (ev.exempt()) {
    exempt_frames_++;
    output->set_bool_value(true);
    SPDLOG_WARN(
        "Frame {}: collision EXEMPT ({}) other_id={} ego_v={:.2f} rel_v={:.2f}",
        input.frame_id(), ev.exempt_reason(), ev.other_id(),
        ev.ego_speed_mps(), ev.relative_speed_mps());
  } else {
    non_exempt_frames_++;
    output->set_bool_value(false);
    SPDLOG_ERROR(
        "Frame {}: COLLISION at_fault kind={} other_id={} ego_v={:.2f} "
        "rel_v={:.2f} approach={:.1f}deg",
        input.frame_id(), ev.kind(), ev.other_id(), ev.ego_speed_mps(),
        ev.relative_speed_mps(), ev.approach_angle_deg());
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
      " total_collision_frames=" + std::to_string(collision_frames_) +
      "/" + std::to_string(total_frames_));
  return summary;
}

}  // namespace grading_mini
