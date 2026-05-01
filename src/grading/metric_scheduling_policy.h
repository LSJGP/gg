#pragma once

#include <string>
#include <vector>

#include "absl/status/statusor.h"

namespace grading_mini {

class DAGScheduler {
 public:
  using UpdatePlan = std::vector<std::vector<std::string>>;

  explicit DAGScheduler(const std::vector<std::string>& nodes);

  absl::Status AddDependency(const std::string& node,
                             const std::string& depends_on);

  absl::StatusOr<UpdatePlan> GenerateUpdatePlan();

 private:
  std::vector<std::string> nodes_;
  std::vector<std::pair<std::string, std::string>> edges_;
};

}  // namespace grading_mini
