#pragma once

#include "cpp/lane_graph.h"
#include "proto/sim/runtime.pb.h"

namespace hyw_sim {

proto::RoadContext BuildRoadContext(const LaneGraph& graph,
                                    const proto::VehicleState& ego);

}  // namespace hyw_sim
