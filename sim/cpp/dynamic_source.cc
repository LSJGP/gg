#include "cpp/dynamic_source.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <unordered_map>

#include "cpp/input_format.h"
#include "cpp/json_utils.h"
#include "cpp/proto_io.h"

namespace hyw_sim {
namespace {
namespace fs = std::filesystem;

double WrapAngle(double rad) {
  return std::atan2(std::sin(rad), std::cos(rad));
}

proto::NpcSnapshot ToSnapshot(const proto::Track& tr, const proto::TrackState& st) {
  proto::NpcSnapshot out;
  out.set_id(tr.id());
  out.set_object_type(tr.object_type());
  out.set_x(st.x());
  out.set_y(st.y());
  out.set_z(st.z());
  out.set_heading(st.yaw());
  out.set_vx(st.vx());
  out.set_vy(st.vy());
  out.set_length(st.length());
  out.set_width(st.width());
  out.set_height(st.height());
  return out;
}

proto::NpcSnapshot SnapshotFromFrameNpcState(const proto::FrameNpcState& fn) {
  proto::NpcSnapshot out;
  out.set_id(fn.id());
  out.set_object_type(fn.object_type());
  if (!fn.state().valid()) {
    return out;
  }
  const auto& st = fn.state();
  out.set_x(st.x());
  out.set_y(st.y());
  out.set_z(st.z());
  out.set_heading(st.yaw());
  out.set_vx(st.vx());
  out.set_vy(st.vy());
  out.set_length(st.length());
  out.set_width(st.width());
  out.set_height(st.height());
  return out;
}

proto::NpcSnapshot SnapshotFromStateStruct(const google::protobuf::Struct& st) {
  proto::NpcSnapshot out;
  out.set_id(static_cast<int64_t>(GetFieldNumber(st, "id", 0.0)));
  out.set_object_type(GetFieldString(st, "object_type", "OTHER"));
  if (!GetFieldBool(st, "valid", false)) {
    return out;
  }
  out.set_x(GetFieldNumber(st, "x", 0.0));
  out.set_y(GetFieldNumber(st, "y", 0.0));
  out.set_z(GetFieldNumber(st, "z", 0.0));
  out.set_heading(GetFieldNumber(st, "yaw", 0.0));
  out.set_vx(GetFieldNumber(st, "vx", 0.0));
  out.set_vy(GetFieldNumber(st, "vy", 0.0));
  out.set_length(GetFieldNumber(st, "length", 4.5));
  out.set_width(GetFieldNumber(st, "width", 1.85));
  out.set_height(GetFieldNumber(st, "height", 1.6));
  return out;
}

class BulkDynamicSource : public DynamicNpcSource {
 public:
  explicit BulkDynamicSource(proto::DynamicObjects dynamic)
      : dynamic_(std::move(dynamic)) {
    timestamps_.assign(dynamic_.timestamps_seconds().begin(),
                       dynamic_.timestamps_seconds().end());
  }

  const std::vector<double>& timestamps() const override { return timestamps_; }

  std::vector<proto::NpcSnapshot> NpcsAtIndex(int idx) const override {
    std::vector<proto::NpcSnapshot> out;
    for (const auto& tr : dynamic_.tracks()) {
      if (tr.is_sdc() || idx < 0 || idx >= tr.states_size()) continue;
      const auto& st = tr.states(idx);
      if (!st.valid()) continue;
      out.push_back(ToSnapshot(tr, st));
    }
    return out;
  }

  std::vector<proto::NpcSnapshot> InterpNPCs(int lo, int hi, double a) const override {
    std::vector<proto::NpcSnapshot> out;
    for (const auto& tr : dynamic_.tracks()) {
      if (tr.is_sdc()) continue;
      if (lo >= tr.states_size() || hi >= tr.states_size()) continue;
      const auto& s0 = tr.states(lo);
      const auto& s1 = tr.states(hi);
      if (s0.valid() && s1.valid()) {
        proto::NpcSnapshot n;
        n.set_id(tr.id());
        n.set_object_type(tr.object_type());
        n.set_x(s0.x() + a * (s1.x() - s0.x()));
        n.set_y(s0.y() + a * (s1.y() - s0.y()));
        n.set_z(s0.z() + a * (s1.z() - s0.z()));
        n.set_heading(WrapAngle(s0.yaw() + a * WrapAngle(s1.yaw() - s0.yaw())));
        n.set_vx(s0.vx() + a * (s1.vx() - s0.vx()));
        n.set_vy(s0.vy() + a * (s1.vy() - s0.vy()));
        n.set_length(s1.length());
        n.set_width(s1.width());
        n.set_height(s1.height());
        out.push_back(n);
      } else if (s0.valid()) {
        out.push_back(ToSnapshot(tr, s0));
      } else if (s1.valid()) {
        out.push_back(ToSnapshot(tr, s1));
      }
    }
    return out;
  }

  bool GetSdcTrack(proto::Track* out) const override {
    if (!out) return false;
    const int64_t sdc_idx = dynamic_.sdc_track_index();
    for (const auto& tr : dynamic_.tracks()) {
      if (tr.is_sdc() || (sdc_idx >= 0 && tr.track_index() == sdc_idx)) {
        *out = tr;
        return true;
      }
    }
    return false;
  }

 private:
  proto::DynamicObjects dynamic_;
  std::vector<double> timestamps_;
};

class StreamDynamicSource : public DynamicNpcSource {
 public:
  static bool Create(const std::string& scenario_dir, ScenarioInputFormat input_format,
                     std::unique_ptr<StreamDynamicSource>* out, std::string* error) {
    const fs::path base = fs::path(scenario_dir) / "dynamic_objects";
    const fs::path header_path = ResolveStreamFile(base, "header", input_format);
    const fs::path frames_dir = base / "frames";
    if (!fs::is_regular_file(header_path) || !fs::is_directory(frames_dir)) {
      if (error) {
        *error =
            "stream layout missing; run: python3 tools/split_existing_dynamic_objects.py "
            + scenario_dir;
      }
      return false;
    }

    const bool use_proto = UsesProtoInput(input_format) ||
                           (input_format == ScenarioInputFormat::kAuto &&
                            header_path.extension() == ".pb");
    auto src = std::unique_ptr<StreamDynamicSource>(
        new StreamDynamicSource(base, use_proto));

    if (use_proto) {
      proto::StreamDynamicHeader header;
      if (!ReadStreamHeaderFromFile(header_path.string(), &header, error)) {
        return false;
      }
      if (header.timestamps_seconds_size() == 0) {
        if (error) *error = "header.pb missing timestamps_seconds";
        return false;
      }
      src->timestamps_.reserve(static_cast<size_t>(header.timestamps_seconds_size()));
      for (double t : header.timestamps_seconds()) {
        src->timestamps_.push_back(t);
      }
    } else {
      google::protobuf::Struct header;
      if (!ReadJsonFileToStruct(header_path.string(), &header, error)) {
        return false;
      }
      const auto* ts_list = GetFieldList(header, "timestamps_seconds");
      if (!ts_list || ts_list->values_size() == 0) {
        if (error) *error = "header.json missing timestamps_seconds";
        return false;
      }
      src->timestamps_.reserve(static_cast<size_t>(ts_list->values_size()));
      for (const auto& v : ts_list->values()) {
        src->timestamps_.push_back(GetNumber(v, 0.0));
      }
    }

    const fs::path sdc_path = ResolveStreamFile(base, "sdc_states", input_format);
    if (fs::is_regular_file(sdc_path)) {
      const bool sdc_ok =
          use_proto ? ReadBinaryFileToMessage(sdc_path.string(), &src->sdc_track_, error)
                    : ReadJsonFileToMessage(sdc_path.string(), &src->sdc_track_, error);
      if (!sdc_ok) {
        return false;
      }
    }

    *out = std::move(src);
    return true;
  }

  const std::vector<double>& timestamps() const override { return timestamps_; }

  std::vector<proto::NpcSnapshot> NpcsAtIndex(int idx) const override {
    return LoadFrameCached(idx);
  }

  std::vector<proto::NpcSnapshot> InterpNPCs(int lo, int hi, double a) const override {
    const auto at_lo = LoadFrameCached(lo);
    const auto at_hi = LoadFrameCached(hi);

    auto find_by_id = [](const std::vector<proto::NpcSnapshot>& v, int64_t id) {
      for (const auto& n : v) {
        if (n.id() == id) return &n;
      }
      return static_cast<const proto::NpcSnapshot*>(nullptr);
    };

    std::vector<proto::NpcSnapshot> out;
    for (const auto& n0 : at_lo) {
      const auto* n1 = find_by_id(at_hi, n0.id());
      if (n1 != nullptr) {
        proto::NpcSnapshot n;
        n.set_id(n0.id());
        n.set_object_type(n0.object_type());
        n.set_x(n0.x() + a * (n1->x() - n0.x()));
        n.set_y(n0.y() + a * (n1->y() - n0.y()));
        n.set_z(n0.z() + a * (n1->z() - n0.z()));
        n.set_heading(WrapAngle(n0.heading() + a * WrapAngle(n1->heading() - n0.heading())));
        n.set_vx(n0.vx() + a * (n1->vx() - n0.vx()));
        n.set_vy(n0.vy() + a * (n1->vy() - n0.vy()));
        n.set_length(n1->length());
        n.set_width(n1->width());
        n.set_height(n1->height());
        out.push_back(n);
      } else {
        out.push_back(n0);
      }
    }
    for (const auto& n1 : at_hi) {
      if (find_by_id(at_lo, n1.id()) == nullptr) {
        out.push_back(n1);
      }
    }
    return out;
  }

  int64_t stream_io_us() const override { return stream_io_us_; }

  bool GetSdcTrack(proto::Track* out) const override {
    if (!out || sdc_track_.states_size() == 0) return false;
    *out = sdc_track_;
    return true;
  }

 private:
  StreamDynamicSource(fs::path base, bool use_proto)
      : base_dir_(std::move(base)), use_proto_(use_proto) {}

  std::vector<proto::NpcSnapshot> LoadFrameCached(int idx) const {
    const auto it = frame_cache_.find(idx);
    if (it != frame_cache_.end()) {
      return it->second;
    }
    auto snaps = LoadFrame(idx);
    if (frame_cache_.size() >= 2) {
      frame_cache_.erase(frame_cache_.begin());
    }
    frame_cache_[idx] = snaps;
    return snaps;
  }

  std::vector<proto::NpcSnapshot> LoadFrame(int idx) const {
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<proto::NpcSnapshot> out;
    std::string err;

    if (use_proto_) {
      std::ostringstream path;
      path << base_dir_.string() << "/frames/" << std::setw(5) << std::setfill('0') << idx
           << ".pb";
      proto::DynamicFrame frame;
      if (!ReadDynamicFrameFromFile(path.str(), &frame, &err)) {
        return out;
      }
      for (const auto& fn : frame.states()) {
        if (!fn.state().valid()) continue;
        out.push_back(SnapshotFromFrameNpcState(fn));
      }
    } else {
      std::ostringstream path;
      path << base_dir_.string() << "/frames/" << std::setw(5) << std::setfill('0') << idx
           << ".json";
      google::protobuf::Struct doc;
      if (!ReadJsonFileToStruct(path.str(), &doc, &err)) {
        return out;
      }
      const auto* states = GetFieldList(doc, "states");
      if (states) {
        for (const auto& sv : states->values()) {
          if (sv.kind_case() != google::protobuf::Value::kStructValue) continue;
          const auto& st = sv.struct_value();
          if (!GetFieldBool(st, "valid", false)) continue;
          out.push_back(SnapshotFromStateStruct(st));
        }
      }
    }

    const auto t1 = std::chrono::steady_clock::now();
    stream_io_us_ += std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    return out;
  }

  fs::path base_dir_;
  bool use_proto_ = false;
  std::vector<double> timestamps_;
  proto::Track sdc_track_;

  mutable std::unordered_map<int, std::vector<proto::NpcSnapshot>> frame_cache_;
  mutable int64_t stream_io_us_ = 0;
};

}  // namespace

std::unique_ptr<DynamicNpcSource> CreateBulkDynamicSource(
    proto::DynamicObjects dynamic) {
  return std::make_unique<BulkDynamicSource>(std::move(dynamic));
}

std::unique_ptr<DynamicNpcSource> CreateStreamDynamicSource(
    const std::string& scenario_dir, ScenarioInputFormat input_format,
    std::string* error) {
  std::unique_ptr<StreamDynamicSource> out;
  if (!StreamDynamicSource::Create(scenario_dir, input_format, &out, error)) {
    return nullptr;
  }
  return out;
}

}  // namespace hyw_sim
