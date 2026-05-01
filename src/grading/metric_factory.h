#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "absl/status/statusor.h"
#include "src/grading/macros.h"
#include "src/grading/metric_base.h"

namespace grading_mini {

class MetricFactory {
  GRADING_DECLARE_SINGLETON(MetricFactory);

 public:
  using Creator = std::function<std::unique_ptr<MetricBase>()>;

  bool Register(const std::string& name, Creator creator);
  bool Unregister(const std::string& name);
  absl::StatusOr<std::unique_ptr<MetricBase>> Create(const std::string& name);

 private:
  std::unordered_map<std::string, Creator> registry_;
};

}  // namespace grading_mini
