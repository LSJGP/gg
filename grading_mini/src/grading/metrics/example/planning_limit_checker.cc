#include "src/grading/metrics/example/planning_limit_checker.h"

#include "proto/grading/metrics/example_metric.pb.h"
#include "spdlog/spdlog.h"

namespace grading_mini {

REGISTER_METRIC(PlanningLimitChecker, "planning_limit_checker");

absl::Status PlanningLimitChecker::Init(const google::protobuf::Message* config) {
  if (config) {
    if (const auto* typed =
            dynamic_cast<const proto::PlanningLimitCheckerConfig*>(config)) {
      if (typed->max_desired_speed_mps() > 0.0) {
        max_speed_mps_ = typed->max_desired_speed_mps();
      }
    }
  }
  SPDLOG_INFO("PlanningLimitChecker init: max_desired_speed={:.1f} m/s",
              max_speed_mps_);
  return absl::OkStatus();
}

absl::Status PlanningLimitChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  total_frames_++;

  if (!input.has_planning_command()) {
    violation_frames_++;
    output->set_bool_value(false);
    SPDLOG_WARN("Frame {}: missing planning_command", input.frame_id());
    return absl::OkStatus();
  }

  const double desired = input.planning_command().desired_speed_mps();
  const bool ok = desired <= max_speed_mps_ + 1e-6;
  if (!ok) violation_frames_++;

  output->set_bool_value(ok);
  if (!ok) {
    SPDLOG_WARN("Frame {}: desired_speed {:.2f} > {:.2f}", input.frame_id(),
                desired, max_speed_mps_);
  }
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> PlanningLimitChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  MetricSummary summary;
  summary.set_metric_name(name_);
  summary.set_passed(violation_frames_ == 0);
  summary.set_detail("bad_frames=" + std::to_string(violation_frames_) + "/" +
                     std::to_string(total_frames_));
  return summary;
}

}  // namespace grading_mini
