#include "src/grading/grader.h"

#include "spdlog/spdlog.h"

#include "src/grading/macros.h"
#include "src/grading/metric_factory.h"

namespace grading_mini {

absl::Status Grader::Init(const std::vector<std::string>& metric_names) {
  std::vector<MetricInitSpec> specs;
  specs.reserve(metric_names.size());
  for (const auto& n : metric_names) {
    specs.push_back({n, nullptr});
  }
  return Init(specs);
}

absl::Status Grader::Init(const std::vector<MetricInitSpec>& specs) {
  for (const auto& spec : specs) {
    auto metric_or = MetricFactory::Instance()->Create(spec.name);
    RETURN_IF_ERROR(metric_or.status());
    auto& metric = metric_or.value();
    metric->set_name(spec.name);
    RETURN_IF_ERROR(metric->Init(spec.config));
    RETURN_IF_ERROR(manager_.AddMetric(spec.name, std::move(metric)));
    SPDLOG_INFO("Enabled metric: {}", spec.name);
  }
  return manager_.BuildGraph();
}

absl::Status Grader::ProcessFrame(const MetricFrameInput& input) {
  return manager_.RunOneFrame(input);
}

absl::StatusOr<proto::GradingReport> Grader::Finish() {
  return manager_.GenerateReport();
}

}  // namespace grading_mini
