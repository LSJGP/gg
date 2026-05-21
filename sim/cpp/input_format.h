#pragma once

#include <filesystem>
#include <string>

namespace hyw_sim {

enum class ScenarioInputFormat { kAuto, kJson, kProto };

ScenarioInputFormat ParseInputFormat(const std::string& s);

/// Resolve scenario_meta / lane_graph / dynamic_objects file stem to a path.
std::filesystem::path ResolveScenarioFile(const std::filesystem::path& scenario_dir,
                                          const char* stem,
                                          ScenarioInputFormat fmt);

bool UsesProtoInput(ScenarioInputFormat fmt);

std::filesystem::path ResolveStreamFile(
    const std::filesystem::path& dynamic_objects_dir, const char* stem,
    ScenarioInputFormat fmt);

}  // namespace hyw_sim
