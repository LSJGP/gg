#include "cpp/scenario_loader.h"

#include <filesystem>
#include <string>

#include "cpp/json_utils.h"

namespace hyw_sim {
namespace fs = std::filesystem;
namespace {

Pose2D ParsePose(const google::protobuf::Struct *pose) {
  Pose2D out;
  if (!pose)
    return out;
  out.x = GetFieldNumber(*pose, "x", 0.0);
  out.y = GetFieldNumber(*pose, "y", 0.0);
  out.yaw = GetFieldNumber(*pose, "yaw", 0.0);
  return out;
}

TrackState ParseTrackState(const google::protobuf::Struct &st) {
  TrackState out;
  out.valid = GetFieldBool(st, "valid", false);
  if (!out.valid)
    return out;
  out.x = GetFieldNumber(st, "x", 0.0);
  out.y = GetFieldNumber(st, "y", 0.0);
  out.z = GetFieldNumber(st, "z", 0.0);
  out.yaw = GetFieldNumber(st, "yaw", 0.0);
  out.vx = GetFieldNumber(st, "vx", 0.0);
  out.vy = GetFieldNumber(st, "vy", 0.0);
  out.length = GetFieldNumber(st, "length", 4.5);
  out.width = GetFieldNumber(st, "width", 1.85);
  out.height = GetFieldNumber(st, "height", 1.6);
  return out;
}

} // namespace

bool LoadScenarioFromDir(const std::string &scenario_dir, Scenario *out,
                         std::string *error) {
  out->tracks.clear();
  out->timestamps_seconds.clear();

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

  google::protobuf::Struct meta;
  google::protobuf::Struct objs;
  if (!ReadJsonFileToStruct(meta_path.string(), &meta, error))
    return false;
  if (!ReadJsonFileToStruct(objs_path.string(), &objs, error))
    return false;

  out->scenario_id = GetFieldString(meta, "scenario_id", "");
  out->init_pose = ParsePose(GetFieldStruct(meta, "init_pose"));
  out->goal_pose = ParsePose(GetFieldStruct(meta, "goal_pose"));
  out->current_time_index =
      static_cast<int64_t>(GetFieldNumber(objs, "current_time_index", 0.0));
  out->sdc_track_index =
      static_cast<int64_t>(GetFieldNumber(objs, "sdc_track_index", -1.0));

  if (const auto *ts = GetFieldList(objs, "timestamps_seconds")) {
    out->timestamps_seconds.reserve(ts->values_size());
    for (const auto &v : ts->values()) {
      out->timestamps_seconds.push_back(GetNumber(v, 0.0));
    }
  }

  const auto *tracks = GetFieldList(objs, "tracks");
  if (!tracks) {
    if (error)
      *error = "dynamic_objects.json missing tracks";
    return false;
  }
  out->tracks.reserve(tracks->values_size());
  for (const auto &tv : tracks->values()) {
    if (tv.kind_case() != google::protobuf::Value::kStructValue)
      continue;
    const auto &t = tv.struct_value();
    Track track;
    track.track_index =
        static_cast<int64_t>(GetFieldNumber(t, "track_index", 0.0));
    track.id = static_cast<int64_t>(GetFieldNumber(t, "id", 0.0));
    track.object_type = GetFieldString(t, "object_type", "OTHER");
    track.is_sdc = GetFieldBool(t, "is_sdc", false);
    if (const auto *states = GetFieldList(t, "states")) {
      track.states.reserve(states->values_size());
      for (const auto &sv : states->values()) {
        if (sv.kind_case() != google::protobuf::Value::kStructValue) {
          track.states.emplace_back();
          continue;
        }
        track.states.push_back(ParseTrackState(sv.struct_value()));
      }
    }
    out->tracks.push_back(std::move(track));
  }

  if (out->timestamps_seconds.empty()) {
    if (error)
      *error = "timestamps_seconds is empty";
    return false;
  }
  return true;
}

} // namespace hyw_sim
