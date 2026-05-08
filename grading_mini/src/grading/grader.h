#pragma once

#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "proto/grading/metric_input.pb.h"
#include "proto/grading/metric_output.pb.h"
#include "src/grading/metric_manager.h"

namespace grading_mini {

class Grader {
 public:
  Grader() = default;

  absl::Status Init(const std::vector<std::string>& metric_names);
  absl::Status ProcessFrame(const MetricFrameInput& input);
  absl::StatusOr<proto::GradingReport> Finish();

  // Per-metric pass/fail recorded for the most-recently-processed frame.
  // Order follows the DAG topological order. Empty until the first frame is
  // processed. Useful for streaming/online integration where the caller wants
  // to print a one-line "tick" each frame.
  std::vector<std::pair<std::string, bool>> LastFrameVerdicts() const {
    return manager_.LastFrameVerdicts();
  }

 private:
  MetricManager manager_;
};

}  // namespace grading_mini
