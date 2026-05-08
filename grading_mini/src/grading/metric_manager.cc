#include "src/grading/metric_manager.h"

#include "spdlog/spdlog.h"

#include "src/grading/macros.h"

namespace grading_mini {

absl::Status MetricManager::AddMetric(const std::string& name,
                                      std::unique_ptr<MetricBase> metric) {
  if (metrics_.count(name)) {
    return absl::AlreadyExistsError("Metric [" + name + "] already exists");
  }
  metrics_.emplace(name, std::move(metric));
  payloads_.emplace(name, std::make_unique<Payload<MetricFrameOutput>>());
  return absl::OkStatus();
}

absl::Status MetricManager::BuildGraph() {
  std::vector<std::string> names;
  names.reserve(metrics_.size());
  for (const auto& [n, _] : metrics_) names.push_back(n);

  DAGScheduler scheduler(names);
  for (const auto& [name, metric] : metrics_) {
    for (const auto& dep : metric->dependencies()) {
      RETURN_IF_ERROR(scheduler.AddDependency(name, dep));
    }
  }

  auto plan_or = scheduler.GenerateUpdatePlan();
  RETURN_IF_ERROR(plan_or.status());
  plan_ = std::make_unique<DAGScheduler::UpdatePlan>(
      std::move(plan_or).value());

  for (const auto& name : names) {
    for (const auto& dep : metrics_[name]->dependencies()) {
      auto it = payloads_.find(dep);
      if (it == payloads_.end()) {
        return absl::NotFoundError("Payload for dep [" + dep + "] not found");
      }
      RETURN_IF_ERROR(
          metrics_[name]->SetDependencyPayload(dep, it->second.get()));
    }
  }

  SPDLOG_INFO("DAG built: {} metrics, {} levels", names.size(), plan_->size());
  return absl::OkStatus();
}

absl::Status MetricManager::RunOneFrame(const MetricFrameInput& input) {
  if (!plan_) return absl::FailedPreconditionError("Graph not built");

  for (const auto& level : *plan_) {
    for (const auto& name : level) {
      auto& metric = metrics_[name];
      auto& payload = payloads_[name];
      MetricFrameOutput output;

      RETURN_IF_ERROR(
          metric->CalculateOneFrame(input, payload->data(), &output));

      output.set_frame_id(input.frame_id());
      payload->InsertData(output);
      payload->MaintainOnce();
    }
  }
  return absl::OkStatus();
}

std::vector<std::pair<std::string, bool>>
MetricManager::LastFrameVerdicts() const {
  std::vector<std::pair<std::string, bool>> out;
  if (!plan_) return out;
  for (const auto& level : *plan_) {
    for (const auto& name : level) {
      auto it = payloads_.find(name);
      if (it == payloads_.end() || it->second->data().empty()) continue;
      out.emplace_back(name, it->second->data().back().bool_value());
    }
  }
  return out;
}

absl::StatusOr<proto::GradingReport> MetricManager::GenerateReport() {
  if (!plan_) return absl::FailedPreconditionError("Graph not built");

  proto::GradingReport report;
  report.set_overall_passed(true);

  for (const auto& level : *plan_) {
    for (const auto& name : level) {
      auto& metric = metrics_[name];
      auto& payload = payloads_[name];

      auto summary_or = metric->SummarizeResult(payload->data());
      if (!summary_or.ok()) {
        SPDLOG_WARN("Summarize [{}] failed: {}", name,
                    std::string(summary_or.status().message()));
        continue;
      }

      auto summary = std::move(summary_or).value();
      if (summary.metric_name().empty()) summary.set_metric_name(name);
      report.add_summaries()->CopyFrom(summary);

      if (!summary.passed()) report.set_overall_passed(false);
    }
  }
  return report;
}

}  // namespace grading_mini
