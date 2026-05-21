#include "cpp/input_format.h"

#include <stdexcept>

namespace hyw_sim {
namespace fs = std::filesystem;

ScenarioInputFormat ParseInputFormat(const std::string& s) {
  if (s == "auto") return ScenarioInputFormat::kAuto;
  if (s == "json") return ScenarioInputFormat::kJson;
  if (s == "proto") return ScenarioInputFormat::kProto;
  throw std::runtime_error("unknown --input-format: " + s + " (use auto, json, or proto)");
}

bool UsesProtoInput(ScenarioInputFormat fmt) {
  return fmt == ScenarioInputFormat::kProto;
}

fs::path ResolveStreamFile(const fs::path& dynamic_objects_dir, const char* stem,
                           ScenarioInputFormat fmt) {
  const fs::path pb = dynamic_objects_dir / (std::string(stem) + ".pb");
  const fs::path js = dynamic_objects_dir / (std::string(stem) + ".json");
  if (fmt == ScenarioInputFormat::kProto) {
    return pb;
  }
  if (fmt == ScenarioInputFormat::kJson) {
    return js;
  }
  if (fs::is_regular_file(pb)) {
    return pb;
  }
  return js;
}

fs::path ResolveScenarioFile(const fs::path& scenario_dir, const char* stem,
                             ScenarioInputFormat fmt) {
  const fs::path pb = scenario_dir / (std::string(stem) + ".pb");
  const fs::path js = scenario_dir / (std::string(stem) + ".json");
  if (fmt == ScenarioInputFormat::kProto) {
    return pb;
  }
  if (fmt == ScenarioInputFormat::kJson) {
    return js;
  }
  if (fs::is_regular_file(pb)) {
    return pb;
  }
  return js;
}

}  // namespace hyw_sim
