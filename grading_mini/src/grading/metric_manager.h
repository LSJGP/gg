#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "proto/grading/metric_input.pb.h"
#include "proto/grading/metric_output.pb.h"
#include "src/grading/metric_base.h"
#include "src/grading/metric_result_payload.h"
#include "src/grading/metric_scheduling_policy.h"

namespace grading_mini {

class MetricManager {
 public:
  absl::Status AddMetric(const std::string& name,
                         std::unique_ptr<MetricBase> metric);

  absl::Status BuildGraph();

  absl::Status RunOneFrame(const MetricFrameInput& input);

  absl::StatusOr<proto::GradingReport> GenerateReport();

  // Returns (metric_name, bool_value) for the last frame processed, in DAG
  // topological order. Caller-side helper for streaming integrations.
  std::vector<std::pair<std::string, bool>> LastFrameVerdicts() const;

 private:
  std::unordered_map<std::string, std::unique_ptr<MetricBase>> metrics_;
  std::unordered_map<std::string,
                     std::unique_ptr<Payload<MetricFrameOutput>>>
      payloads_;
  std::unique_ptr<DAGScheduler::UpdatePlan> plan_;
};

}  // namespace grading_mini
