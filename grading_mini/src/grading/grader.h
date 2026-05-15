#pragma once

#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "proto/grading/metric_input.pb.h"
#include "proto/grading/metric_output.pb.h"
#include "src/grading/metric_manager.h"

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace grading_mini {

struct MetricInitSpec {
  std::string name;
  const google::protobuf::Message* config = nullptr;
};

class Grader {
 public:
  Grader() = default;

  absl::Status Init(const std::vector<MetricInitSpec>& specs);
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
