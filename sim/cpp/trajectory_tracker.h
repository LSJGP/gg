#pragma once

#include "proto/sim/runtime.pb.h"

namespace hyw_sim {

void StepVehicle(proto::VehicleState* ego, const proto::PlanCommand& cmd, double dt,
                 const proto::VehicleParams& params);

proto::PlanCommand TrajectoryToCommand(const proto::PlannerTrajectory& trajectory,
                                       double dt);

proto::PlannerTrajectory BuildTrajectoryFromCommand(
    const proto::PlanCommand& cmd, const proto::VehicleState& ego,
    const proto::VehicleParams& params, const proto::PlannerConfig& config);

}  // namespace hyw_sim
