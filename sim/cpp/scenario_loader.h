#pragma once

#include <memory>
#include <string>

#include "cpp/dynamic_source.h"
#include "cpp/input_format.h"
#include "proto/sim/map.pb.h"
#include "proto/sim/scenario.pb.h"

namespace hyw_sim {

struct ScenarioBundle {
  proto::ScenarioMeta meta;
  proto::DynamicObjects dynamic;
  proto::StaticMap map;
};

bool LoadScenarioMetaAndMap(const std::string& scenario_dir,
                            ScenarioInputFormat input_format,
                            proto::ScenarioMeta* meta, proto::StaticMap* map,
                            std::string* error);

/// Loads meta + lane_graph; dynamic via bulk (fills bundle.dynamic) or stream (dynamic_source).
bool LoadScenarioFromDir(const std::string& scenario_dir, ScenarioLoadMode mode,
                         ScenarioInputFormat input_format, ScenarioBundle* bundle,
                         std::unique_ptr<DynamicNpcSource>* dynamic_source,
                         std::string* error);

}  // namespace hyw_sim
