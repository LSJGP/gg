#include "cpp/scenario_loader.h"

#include <filesystem>

#include "cpp/proto_io.h"

namespace hyw_sim {
namespace fs = std::filesystem;

bool LoadScenarioMetaAndMap(const std::string& scenario_dir,
                            proto::ScenarioMeta* meta, proto::StaticMap* map,
                            std::string* error) {
  if (!meta || !map) {
    if (error) *error = "null meta/map pointers";
    return false;
  }
  meta->Clear();
  map->Clear();

  const fs::path base = fs::path(scenario_dir);
  const fs::path meta_path = base / "scenario_meta.json";
  const fs::path graph_path = base / "lane_graph.json";
  if (!fs::is_regular_file(meta_path) || !fs::is_regular_file(graph_path)) {
    if (error) {
      *error = "missing required scenario files in: " + scenario_dir;
    }
    return false;
  }

  if (!ReadScenarioMetaFromFile(meta_path.string(), meta, error)) {
    return false;
  }
  if (!ReadStaticMapFromFile(graph_path.string(), map, error)) {
    return false;
  }

  if (!meta->has_init_pose() || !meta->has_goal_pose()) {
    if (error) *error = "scenario_meta.json missing init_pose or goal_pose";
    return false;
  }
  return true;
}

bool LoadScenarioFromDir(const std::string& scenario_dir, ScenarioLoadMode mode,
                         ScenarioBundle* bundle,
                         std::unique_ptr<DynamicNpcSource>* dynamic_source,
                         std::string* error) {
  if (!bundle || !dynamic_source) {
    if (error) *error = "null output pointers";
    return false;
  }
  bundle->meta.Clear();
  bundle->dynamic.Clear();
  bundle->map.Clear();
  *dynamic_source = nullptr;

  if (!LoadScenarioMetaAndMap(scenario_dir, &bundle->meta, &bundle->map, error)) {
    return false;
  }

  if (mode == ScenarioLoadMode::kBulk) {
    const fs::path objs_path = fs::path(scenario_dir) / "dynamic_objects.json";
    if (!fs::is_regular_file(objs_path)) {
      if (error) *error = "missing dynamic_objects.json in: " + scenario_dir;
      return false;
    }
    if (!ReadDynamicObjectsFromFile(objs_path.string(), &bundle->dynamic, error)) {
      return false;
    }
    if (bundle->dynamic.timestamps_seconds_size() == 0) {
      if (error) *error = "dynamic_objects.json missing timestamps_seconds";
      return false;
    }
    *dynamic_source = CreateBulkDynamicSource(std::move(bundle->dynamic));
    return true;
  }

  *dynamic_source = CreateStreamDynamicSource(scenario_dir, error);
  if (!*dynamic_source) {
    return false;
  }
  if (dynamic_source->get()->timestamps().empty()) {
    if (error) *error = "stream header missing timestamps_seconds";
    return false;
  }
  return true;
}

}  // namespace hyw_sim
