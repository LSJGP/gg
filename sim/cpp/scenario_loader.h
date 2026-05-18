#pragma once

#include <string>

#include "proto/sim/map.pb.h"
#include "proto/sim/scenario.pb.h"

namespace hyw_sim {

struct ScenarioBundle {
  proto::ScenarioMeta meta;
  proto::DynamicObjects dynamic;
  proto::StaticMap map;
};

bool LoadScenarioFromDir(const std::string& scenario_dir, ScenarioBundle* out,
                         std::string* error);

}  // namespace hyw_sim
