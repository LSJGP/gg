#include "cpp/scenario_loader.h"

#include <filesystem>

#include "cpp/proto_io.h"

namespace hyw_sim {
namespace fs = std::filesystem;

bool LoadScenarioFromDir(const std::string& scenario_dir, ScenarioBundle* out,
                         std::string* error) {
  out->meta.Clear();
  out->dynamic.Clear();
  out->map.Clear();

  const fs::path base = fs::path(scenario_dir);
  const fs::path meta_path = base / "scenario_meta.json";
  const fs::path objs_path = base / "dynamic_objects.json";
  const fs::path graph_path = base / "lane_graph.json";
  if (!fs::is_regular_file(meta_path) || !fs::is_regular_file(objs_path) ||
      !fs::is_regular_file(graph_path)) {
    if (error) {
      *error = "missing required scenario files in: " + scenario_dir;
    }
    return false;
  }

  if (!ReadScenarioMetaFromFile(meta_path.string(), &out->meta, error)) {
    return false;
  }
  if (!ReadDynamicObjectsFromFile(objs_path.string(), &out->dynamic, error)) {
    return false;
  }
  if (!ReadStaticMapFromFile(graph_path.string(), &out->map, error)) {
    return false;
  }

  if (!out->meta.has_init_pose() || !out->meta.has_goal_pose()) {
    if (error) *error = "scenario_meta.json missing init_pose or goal_pose";
    return false;
  }
  if (out->dynamic.timestamps_seconds_size() == 0) {
    if (error) *error = "dynamic_objects.json missing timestamps_seconds";
    return false;
  }
  return true;
}

} // namespace hyw_sim
