#include "src/grading/metric_scheduling_policy.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace grading_mini {

DAGScheduler::DAGScheduler(const std::vector<std::string>& nodes)
    : nodes_(nodes) {}

absl::Status DAGScheduler::AddDependency(const std::string& node,
                                         const std::string& depends_on) {
  edges_.emplace_back(node, depends_on);
  return absl::OkStatus();
}

absl::StatusOr<DAGScheduler::UpdatePlan> DAGScheduler::GenerateUpdatePlan() {
  std::unordered_set<std::string> node_set(nodes_.begin(), nodes_.end());
  std::unordered_map<std::string, int> in_degree;
  std::unordered_map<std::string, std::vector<std::string>> dependents;

  for (const auto& n : nodes_) in_degree[n] = 0;

  for (const auto& [node, dep] : edges_) {
    if (!node_set.count(dep)) continue;
    dependents[dep].push_back(node);
    in_degree[node]++;
  }

  UpdatePlan plan;
  std::queue<std::string> q;
  for (const auto& [n, deg] : in_degree) {
    if (deg == 0) q.push(n);
  }

  size_t visited = 0;
  while (!q.empty()) {
    std::vector<std::string> level;
    size_t level_size = q.size();
    for (size_t i = 0; i < level_size; ++i) {
      auto cur = q.front();
      q.pop();
      level.push_back(cur);
      visited++;
      for (const auto& next : dependents[cur]) {
        if (--in_degree[next] == 0) q.push(next);
      }
    }
    plan.push_back(std::move(level));
  }

  if (visited != nodes_.size()) {
    return absl::InvalidArgumentError("Cyclic dependency detected");
  }
  return plan;
}

}  // namespace grading_mini
