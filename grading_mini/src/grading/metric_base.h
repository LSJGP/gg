#pragma once

#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/message.h"

#include "proto/grading/metric_input.pb.h"
#include "proto/grading/metric_output.pb.h"
#include "src/grading/metric_result_payload.h"

namespace grading_mini {

using MetricFrameInput = proto::MetricFrameInput;
using MetricFrameOutput = proto::MetricFrameOutput;
using MetricSummary = proto::MetricSummary;

class MetricBase {
 public:
  MetricBase() = default;
  virtual ~MetricBase() = default;

  virtual absl::Status Init(const google::protobuf::Message* config) {
    return absl::OkStatus();
  }

  virtual absl::Status CalculateOneFrame(
      const MetricFrameInput& input,
      const std::deque<MetricFrameOutput>& history,
      MetricFrameOutput* output) = 0;

  virtual absl::StatusOr<MetricSummary> SummarizeResult(
      const std::deque<MetricFrameOutput>& history) = 0;

  const std::string& name() const { return name_; }
  void set_name(const std::string& n) { name_ = n; }

  const std::vector<std::string>& dependencies() const { return deps_; }

  absl::Status SetDependencyPayload(
      const std::string& dep_name,
      const Payload<MetricFrameOutput>* payload) {
    dep_payloads_[dep_name] = payload;
    return absl::OkStatus();
  }

 protected:
  const Payload<MetricFrameOutput>& DependencyResult(
      const std::string& dep_name) const;

  std::string name_;
  std::vector<std::string> deps_;
  std::unordered_map<std::string, const Payload<MetricFrameOutput>*>
      dep_payloads_;
};

}  // namespace grading_mini
