#include "src/grading/grader.h"

#include "spdlog/spdlog.h"

#include "src/grading/macros.h"
#include "src/grading/metric_factory.h"

namespace grading_mini {

absl::Status Grader::Init(const std::vector<std::string>& metric_names) {
  for (const auto& name : metric_names) {
    auto metric_or = MetricFactory::Instance()->Create(name);
    RETURN_IF_ERROR(metric_or.status());
    auto& metric = metric_or.value();
    metric->set_name(name);
    RETURN_IF_ERROR(metric->Init(nullptr));
    RETURN_IF_ERROR(manager_.AddMetric(name, std::move(metric)));
    SPDLOG_INFO("Enabled metric: {}", name);
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
