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

 private:
  MetricManager manager_;
};

}  // namespace grading_mini
