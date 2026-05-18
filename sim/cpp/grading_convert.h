#pragma once

#include "proto/grading/metric_input.pb.h"
#include "proto/sim/map.pb.h"
#include "proto/sim/runtime.pb.h"

namespace hyw_sim {

grading_mini::proto::MetricFrameInput ToMetricFrameInput(
    const proto::FrameRecord& frame, const proto::StaticMap* scene_map,
    const proto::VehicleParams& ego_params);

}  // namespace hyw_sim
