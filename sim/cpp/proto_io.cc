#include "cpp/proto_io.h"

#include <fstream>
#include <sstream>

#include "cpp/json_utils.h"
#include "google/protobuf/util/json_util.h"

namespace hyw_sim {
namespace {

std::string ReadWholeFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) return {};
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

bool ParseJsonToMessage(const std::string& text, google::protobuf::Message* out,
                        std::string* error) {
  google::protobuf::util::JsonParseOptions opts;
  opts.ignore_unknown_fields = true;
  const auto s = google::protobuf::util::JsonStringToMessage(text, out, opts);
  if (!s.ok()) {
    if (error) *error = std::string(s.message());
    return false;
  }
  return true;
}

void FillVec3FromList(const google::protobuf::ListValue& pt, proto::Vec3* out) {
  if (pt.values_size() < 2) return;
  out->set_x(GetNumber(pt.values(0), 0.0));
  out->set_y(GetNumber(pt.values(1), 0.0));
  out->set_z(pt.values_size() > 2 ? GetNumber(pt.values(2), 0.0) : 0.0);
}

void ParseVec3Polyline(const google::protobuf::ListValue* list,
                       google::protobuf::RepeatedPtrField<proto::Vec3>* out) {
  if (!list) return;
  for (const auto& pv : list->values()) {
    if (pv.kind_case() != google::protobuf::Value::kListValue) continue;
    auto* v = out->Add();
    FillVec3FromList(pv.list_value(), v);
  }
}

void ParseBoundarySegments(
    const google::protobuf::ListValue* list,
    google::protobuf::RepeatedPtrField<proto::BoundarySegment>* out) {
  if (!list) return;
  for (const auto& bv : list->values()) {
    if (bv.kind_case() != google::protobuf::Value::kStructValue) continue;
    const auto& b = bv.struct_value();
    auto* seg = out->Add();
    seg->set_lane_start_index(
        static_cast<int32_t>(GetFieldNumber(b, "lane_start_index", 0.0)));
    seg->set_lane_end_index(
        static_cast<int32_t>(GetFieldNumber(b, "lane_end_index", 0.0)));
    seg->set_boundary_feature_id(
        static_cast<int64_t>(GetFieldNumber(b, "boundary_feature_id", 0.0)));
    seg->set_boundary_type(GetFieldString(b, "boundary_type", "TYPE_UNKNOWN"));
  }
}

void ParseLaneNeighbors(
    const google::protobuf::ListValue* list,
    google::protobuf::RepeatedPtrField<proto::LaneNeighbor>* out) {
  if (!list) return;
  for (const auto& nv : list->values()) {
    if (nv.kind_case() != google::protobuf::Value::kStructValue) continue;
    const auto& n = nv.struct_value();
    auto* ln = out->Add();
    ln->set_feature_id(static_cast<int64_t>(GetFieldNumber(n, "feature_id", 0.0)));
    ln->set_self_start_index(
        static_cast<int32_t>(GetFieldNumber(n, "self_start_index", 0.0)));
    ln->set_self_end_index(
        static_cast<int32_t>(GetFieldNumber(n, "self_end_index", 0.0)));
    ln->set_neighbor_start_index(
        static_cast<int32_t>(GetFieldNumber(n, "neighbor_start_index", 0.0)));
    ln->set_neighbor_end_index(
        static_cast<int32_t>(GetFieldNumber(n, "neighbor_end_index", 0.0)));
    ParseBoundarySegments(GetFieldList(n, "boundaries"), ln->mutable_boundaries());
  }
}

bool StaticMapFromStruct(const google::protobuf::Struct& doc, proto::StaticMap* out,
                         std::string* error) {
  out->Clear();
  out->set_source(GetFieldString(doc, "source", ""));
  out->set_scenario_id(GetFieldString(doc, "scenario_id", ""));

  if (const auto* wo = GetFieldStruct(doc, "world_offset")) {
    auto* w = out->mutable_world_offset();
    w->set_x(GetFieldNumber(*wo, "x", 0.0));
    w->set_y(GetFieldNumber(*wo, "y", 0.0));
    w->set_z(GetFieldNumber(*wo, "z", 0.0));
  }

  if (const auto* counts = GetFieldStruct(doc, "map_feature_counts")) {
    auto* c = out->mutable_map_feature_counts();
    c->set_lanes(static_cast<int32_t>(GetFieldNumber(*counts, "lanes", 0.0)));
    c->set_road_lines(
        static_cast<int32_t>(GetFieldNumber(*counts, "road_lines", 0.0)));
    c->set_road_edges(
        static_cast<int32_t>(GetFieldNumber(*counts, "road_edges", 0.0)));
    c->set_crosswalks(
        static_cast<int32_t>(GetFieldNumber(*counts, "crosswalks", 0.0)));
    c->set_stop_signs(
        static_cast<int32_t>(GetFieldNumber(*counts, "stop_signs", 0.0)));
    c->set_driveways(
        static_cast<int32_t>(GetFieldNumber(*counts, "driveways", 0.0)));
    c->set_speed_bumps(
        static_cast<int32_t>(GetFieldNumber(*counts, "speed_bumps", 0.0)));
  }

  const auto* lanes_raw = GetFieldList(doc, "lanes");
  if (!lanes_raw || lanes_raw->values_size() == 0) {
    if (error) *error = "lane_graph.json missing or empty lanes";
    return false;
  }

  for (const auto& lv : lanes_raw->values()) {
    if (lv.kind_case() != google::protobuf::Value::kStructValue) continue;
    const auto& l = lv.struct_value();
    auto* lane = out->add_lanes();
    lane->set_id(static_cast<int64_t>(GetFieldNumber(l, "id", 0.0)));
    lane->set_type(GetFieldString(l, "type", "UNDEFINED"));
    lane->set_speed_limit_kmh(GetFieldNumber(l, "speed_limit_kmh", 50.0));
    lane->set_interpolating(GetFieldBool(l, "interpolating", false));
    ParseVec3Polyline(GetFieldList(l, "centerline"), lane->mutable_centerline());
    if (const auto* entries = GetFieldList(l, "entry_lanes")) {
      for (const auto& ev : entries->values()) {
        lane->add_entry_lanes(static_cast<int64_t>(GetNumber(ev, 0.0)));
      }
    }
    if (const auto* exits = GetFieldList(l, "exit_lanes")) {
      for (const auto& ev : exits->values()) {
        lane->add_exit_lanes(static_cast<int64_t>(GetNumber(ev, 0.0)));
      }
    }
    ParseBoundarySegments(GetFieldList(l, "left_boundaries"),
                          lane->mutable_left_boundaries());
    ParseBoundarySegments(GetFieldList(l, "right_boundaries"),
                          lane->mutable_right_boundaries());
    ParseLaneNeighbors(GetFieldList(l, "left_neighbors"),
                       lane->mutable_left_neighbors());
    ParseLaneNeighbors(GetFieldList(l, "right_neighbors"),
                       lane->mutable_right_neighbors());
  }

  auto parse_polyline_features = [&](const char* key, auto add_fn) {
    if (const auto* raw = GetFieldList(doc, key)) {
      for (const auto& fv : raw->values()) {
        if (fv.kind_case() != google::protobuf::Value::kStructValue) continue;
        const auto& f = fv.struct_value();
        add_fn(f);
      }
    }
  };

  parse_polyline_features("road_lines", [&](const google::protobuf::Struct& f) {
    auto* rl = out->add_road_lines();
    rl->set_id(static_cast<int64_t>(GetFieldNumber(f, "id", 0.0)));
    rl->set_type(GetFieldString(f, "type", "TYPE_UNKNOWN"));
    ParseVec3Polyline(GetFieldList(f, "polyline"), rl->mutable_polyline());
  });
  parse_polyline_features("road_edges", [&](const google::protobuf::Struct& f) {
    auto* re = out->add_road_edges();
    re->set_id(static_cast<int64_t>(GetFieldNumber(f, "id", 0.0)));
    re->set_type(GetFieldString(f, "type", "TYPE_UNKNOWN"));
    ParseVec3Polyline(GetFieldList(f, "polyline"), re->mutable_polyline());
  });
  parse_polyline_features("crosswalks", [&](const google::protobuf::Struct& f) {
    auto* cw = out->add_crosswalks();
    cw->set_id(static_cast<int64_t>(GetFieldNumber(f, "id", 0.0)));
    ParseVec3Polyline(GetFieldList(f, "polygon"), cw->mutable_polygon());
  });
  parse_polyline_features("driveways", [&](const google::protobuf::Struct& f) {
    auto* dw = out->add_driveways();
    dw->set_id(static_cast<int64_t>(GetFieldNumber(f, "id", 0.0)));
    ParseVec3Polyline(GetFieldList(f, "polygon"), dw->mutable_polygon());
  });
  parse_polyline_features("speed_bumps", [&](const google::protobuf::Struct& f) {
    auto* sb = out->add_speed_bumps();
    sb->set_id(static_cast<int64_t>(GetFieldNumber(f, "id", 0.0)));
    ParseVec3Polyline(GetFieldList(f, "polygon"), sb->mutable_polygon());
  });

  if (const auto* raw = GetFieldList(doc, "stop_signs")) {
    for (const auto& sv : raw->values()) {
      if (sv.kind_case() != google::protobuf::Value::kStructValue) continue;
      const auto& s = sv.struct_value();
      auto* ss = out->add_stop_signs();
      ss->set_id(static_cast<int64_t>(GetFieldNumber(s, "id", 0.0)));
      if (const auto* pos = GetFieldStruct(s, "position")) {
        auto* p = ss->mutable_position();
        p->set_x(GetFieldNumber(*pos, "x", 0.0));
        p->set_y(GetFieldNumber(*pos, "y", 0.0));
        p->set_z(GetFieldNumber(*pos, "z", 0.0));
      }
      if (const auto* lanes = GetFieldList(s, "lanes")) {
        for (const auto& lv : lanes->values()) {
          ss->add_lanes(static_cast<int64_t>(GetNumber(lv, 0.0)));
        }
      }
    }
  }

  if (out->lanes_size() == 0) {
    if (error) *error = "lane_graph.json has no parseable lanes";
    return false;
  }
  return true;
}

}  // namespace

bool ReadJsonFileToMessage(const std::string& path, google::protobuf::Message* out,
                           std::string* error) {
  const std::string text = ReadWholeFile(path);
  if (text.empty()) {
    if (error) *error = "file is empty or cannot be read: " + path;
    return false;
  }
  return ParseJsonToMessage(text, out, error);
}

bool ReadScenarioMetaFromFile(const std::string& path, proto::ScenarioMeta* out,
                              std::string* error) {
  return ReadJsonFileToMessage(path, out, error);
}

bool ReadDynamicObjectsFromFile(const std::string& path,
                                proto::DynamicObjects* out, std::string* error) {
  return ReadJsonFileToMessage(path, out, error);
}

bool ReadStaticMapFromFile(const std::string& path, proto::StaticMap* out,
                           std::string* error) {
  google::protobuf::Struct doc;
  if (!ReadJsonFileToStruct(path, &doc, error)) return false;
  return StaticMapFromStruct(doc, out, error);
}

}  // namespace hyw_sim
