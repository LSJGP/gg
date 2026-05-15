#pragma once

#include <string>

#include "cpp/types.h"

namespace hyw_sim {

bool LoadScenarioFromDir(const std::string& scenario_dir, Scenario* out,
                         std::string* error);

}  // namespace hyw_sim
