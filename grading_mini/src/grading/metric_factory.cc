#include "src/grading/metric_factory.h"

#include "spdlog/spdlog.h"

namespace grading_mini {

MetricFactory::MetricFactory() = default;

bool MetricFactory::Register(const std::string& name, Creator creator) {
  if (registry_.count(name)) {
    SPDLOG_WARN("Metric [{}] already registered, skip", name);
    return false;
  }
  registry_[name] = std::move(creator);
  SPDLOG_DEBUG("Registered metric: {}", name);
  return true;
}

bool MetricFactory::Unregister(const std::string& name) {
  return registry_.erase(name) > 0;
}

absl::StatusOr<std::unique_ptr<MetricBase>> MetricFactory::Create(
    const std::string& name) {
  auto it = registry_.find(name);
  if (it == registry_.end()) {
    return absl::NotFoundError("Metric [" + name + "] not registered");
  }
  return it->second();
}

}  // namespace grading_mini
