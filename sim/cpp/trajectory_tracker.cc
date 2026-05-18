#include "cpp/trajectory_tracker.h"

#include <algorithm>
#include <cmath>

namespace hyw_sim {

void StepVehicle(proto::VehicleState* ego, const proto::PlanCommand& cmd, double dt,
                 const proto::VehicleParams& params) {
  const double accel = std::clamp(cmd.target_acceleration(), -params.max_decel(),
                                  params.max_accel());
  const double target_steer =
      std::clamp(cmd.steering_angle(), -params.max_steer(), params.max_steer());
  const double ds_max = params.max_steer_rate() * dt;
  const double steer =
      std::clamp(target_steer, ego->steer() - ds_max, ego->steer() + ds_max);

  const double new_speed =
      std::clamp(ego->speed() + accel * dt, 0.0, params.max_speed());
  const double wb = std::max(0.5, params.wheelbase());
  const double new_heading =
      ego->heading() + (new_speed / wb) * std::tan(steer) * dt;
  ego->set_x(ego->x() + new_speed * std::cos(new_heading) * dt);
  ego->set_y(ego->y() + new_speed * std::sin(new_heading) * dt);
  ego->set_heading(new_heading);
  ego->set_speed(new_speed);
  ego->set_acceleration(accel);
  ego->set_steer(steer);
}

proto::PlanCommand TrajectoryToCommand(const proto::PlannerTrajectory& trajectory,
                                       double dt) {
  proto::PlanCommand cmd;
  if (trajectory.points().empty()) {
    return cmd;
  }
  const auto& p0 = trajectory.points(0);
  cmd.set_target_acceleration(p0.target_acceleration());
  cmd.set_steering_angle(p0.steering_angle());
  cmd.set_desired_speed_mps(p0.desired_speed_mps());

  if (trajectory.points_size() > 1) {
    const auto& p1 = trajectory.points(1);
    const double t1 = std::max(p1.t_s(), dt);
    if (t1 > 1e-6 && p1.t_s() <= dt * 1.5) {
      const double dv = p1.speed() - p0.speed();
      cmd.set_target_acceleration(dv / t1);
    }
  }
  return cmd;
}

proto::PlannerTrajectory BuildTrajectoryFromCommand(
    const proto::PlanCommand& cmd, const proto::VehicleState& ego,
    const proto::VehicleParams& params, const proto::PlannerConfig& config) {
  proto::PlannerTrajectory traj;
  const double horizon_s = config.horizon_s() > 0.0 ? config.horizon_s() : 3.0;
  const double point_dt = config.point_dt_s() > 0.0 ? config.point_dt_s() : 0.1;

  proto::VehicleState roll = ego;
  for (double t = 0.0; t <= horizon_s + 1e-9; t += point_dt) {
    auto* pt = traj.add_points();
    pt->set_t_s(t);
    pt->set_x(roll.x());
    pt->set_y(roll.y());
    pt->set_heading(roll.heading());
    pt->set_speed(roll.speed());
    pt->set_target_acceleration(cmd.target_acceleration());
    pt->set_steering_angle(cmd.steering_angle());
    pt->set_desired_speed_mps(cmd.desired_speed_mps());
    if (t + point_dt <= horizon_s + 1e-9) {
      StepVehicle(&roll, cmd, point_dt, params);
    }
  }
  return traj;
}

}  // namespace hyw_sim
