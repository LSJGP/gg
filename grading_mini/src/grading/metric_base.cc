#include "src/grading/metric_base.h"

#include "spdlog/spdlog.h"

namespace grading_mini {

const Payload<MetricFrameOutput>& MetricBase::DependencyResult(
    const std::string& dep_name) const {
  auto it = dep_payloads_.find(dep_name);
  if (it == dep_payloads_.end()) {
    SPDLOG_CRITICAL("Dependency [{}] not found for metric [{}]", dep_name,
                    name_);
    std::abort();
  }
  return *it->second;
}

}  // namespace grading_mini
