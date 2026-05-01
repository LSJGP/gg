#include "src/grading/metrics/example/speed_checker.h"

#include "spdlog/spdlog.h"

namespace grading_mini {

REGISTER_METRIC(SpeedChecker, "speed_checker");

absl::Status SpeedChecker::Init(const google::protobuf::Message* config) {
  if (config) {
    auto* typed = dynamic_cast<const proto::SpeedChecker*>(config);
    if (typed && typed->max_speed_threshold() > 0) {
      max_speed_ = typed->max_speed_threshold();
    }
  }
  SPDLOG_INFO("SpeedChecker init: threshold={:.1f} m/s", max_speed_);
  return absl::OkStatus();
}

absl::Status SpeedChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& history,
    MetricFrameOutput* output) {
  double speed = input.vehicle_state().speed();
  bool exceeded = speed > max_speed_;
  if (exceeded) violation_count_++;
  total_frames_++;

  output->set_bool_value(!exceeded);

  proto::SpeedCheckerCustomInfo info;
  info.set_current_speed(speed);
  info.set_exceeded(exceeded);
  output->mutable_custom_info()->PackFrom(info);

  if (exceeded) {
    SPDLOG_WARN("Frame {}: speed {:.1f} > {:.1f}", input.frame_id(), speed,
                max_speed_);
  }
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> SpeedChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& history) {
  MetricSummary summary;
  summary.set_metric_name(name_);
  summary.set_passed(violation_count_ == 0);
  summary.set_detail("violations=" + std::to_string(violation_count_) +
                     "/" + std::to_string(total_frames_));
  return summary;
}

}  // namespace grading_mini
