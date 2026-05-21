#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "proto/sim/runtime.pb.h"
#include "proto/sim/scenario.pb.h"

namespace hyw_sim {

enum class ScenarioLoadMode { kBulk, kStream };

/// NPC dynamic data: bulk (full DynamicObjects) or per-frame disk reads.
class DynamicNpcSource {
 public:
  virtual ~DynamicNpcSource() = default;

  virtual const std::vector<double>& timestamps() const = 0;
  virtual std::vector<proto::NpcSnapshot> NpcsAtIndex(int idx) const = 0;
  virtual std::vector<proto::NpcSnapshot> InterpNPCs(int lo, int hi,
                                                       double a) const = 0;
  virtual int64_t stream_io_us() const { return 0; }

  /// SDC track for --reference-source sdc (stream loads from sdc_states.json).
  virtual bool GetSdcTrack(proto::Track* out) const = 0;
};

std::unique_ptr<DynamicNpcSource> CreateBulkDynamicSource(
    proto::DynamicObjects dynamic);

std::unique_ptr<DynamicNpcSource> CreateStreamDynamicSource(
    const std::string& scenario_dir, std::string* error);

}  // namespace hyw_sim
