#include "src/grading/collision_detector.h"

#include <cmath>

#include "src/grading/geometry.h"

namespace grading_mini {
namespace {

constexpr double kExemptRearEndEgoSpeedMax = 5.0;
constexpr double kExemptRearEndRelSpeedMin = 2.0;
constexpr double kExemptCutInLatVelMin = 0.5;
constexpr double kExemptHeadOnAngleDeg = 135.0;
constexpr double kDefaultEgoHeight = 1.6;

Obb2D MakeEgoObb(const proto::MetricFrameInput& input) {
  Obb2D box;
  const auto& vs = input.vehicle_state();
  const auto& p = input.ego_vehicle();
  const double length = p.length() > 0.0 ? p.length() : 4.5;
  const double width = p.width() > 0.0 ? p.width() : 1.85;
  const double rear = p.rear_overhang() > 0.0 ? p.rear_overhang() : 0.95;
  const double d = length / 2.0 - rear;
  box.cx = vs.x() + d * std::cos(vs.heading());
  box.cy = vs.y() + d * std::sin(vs.heading());
  box.heading = vs.heading();
  box.half_length = length / 2.0;
  box.half_width = width / 2.0;
  return box;
}

Obb2D MakeNpcObb(const proto::NpcState& n) {
  Obb2D box;
  box.cx = n.x();
  box.cy = n.y();
  box.heading = n.heading();
  box.half_length = std::max(0.5, n.length() * 0.5);
  box.half_width = std::max(0.3, n.width() * 0.5);
  return box;
}

CollisionDetectResult ClassifyCollision(const proto::MetricFrameInput& input,
                                        const proto::NpcState& n) {
  const auto& vs = input.vehicle_state();
  const double dx = n.x() - vs.x();
  const double dy = n.y() - vs.y();
  const double c = std::cos(-vs.heading());
  const double s = std::sin(-vs.heading());
  const double fx = c * dx - s * dy;
  const double fy = s * dx + c * dy;
  const double bearing = std::atan2(fy, fx);

  const double ego_speed = vs.speed();
  const double npc_speed = std::hypot(n.vx(), n.vy());
  const double rvx = n.vx() - ego_speed * std::cos(vs.heading());
  const double rvy = n.vy() - ego_speed * std::sin(vs.heading());
  const double rel_speed = std::hypot(rvx, rvy);

  double approach_angle = 0.0;
  if (npc_speed > 0.5) {
    const double ndx = n.vx() / npc_speed;
    const double ndy = n.vy() / npc_speed;
    const double edx = std::cos(vs.heading());
    const double edy = std::sin(vs.heading());
    const double cosang = std::clamp(edx * ndx + edy * ndy, -1.0, 1.0);
    approach_angle = std::acos(cosang) * 180.0 / M_PI;
  }

  std::string zone;
  std::string kind;
  const double abs_b = std::fabs(bearing);
  if (abs_b < M_PI / 4.0) {
    zone = "front";
    kind = "ego_front_into_npc";
  } else if (abs_b > 3.0 * M_PI / 4.0) {
    zone = "rear";
    kind = "npc_rear_into_ego";
  } else {
    zone = "side";
    kind = "side_collision";
  }

  CollisionDetectResult out;
  out.collided = true;
  out.other_id = n.id();
  out.kind = kind;
  out.relative_speed_mps = rel_speed;
  out.ego_speed_mps = ego_speed;
  out.approach_angle_deg = approach_angle;

  if (zone == "rear") {
    if (ego_speed < kExemptRearEndEgoSpeedMax &&
        (npc_speed - ego_speed) > kExemptRearEndRelSpeedMin) {
      out.exempt = true;
      out.ego_at_fault = false;
      out.exempt_reason = "rear_end_on_slow_ego";
    }
  } else if (zone == "side") {
    const double sx = -std::sin(vs.heading());
    const double sy = std::cos(vs.heading());
    const double lat_v_npc = sx * n.vx() + sy * n.vy();
    if ((bearing > 0 && lat_v_npc < -kExemptCutInLatVelMin) ||
        (bearing < 0 && lat_v_npc > kExemptCutInLatVelMin)) {
      out.exempt = true;
      out.ego_at_fault = false;
      out.exempt_reason = "forced_cut_in";
    }
  } else {
    if (approach_angle > kExemptHeadOnAngleDeg) {
      out.exempt = true;
      out.ego_at_fault = false;
      out.exempt_reason = "wrong_way_head_on";
    }
  }
  return out;
}

bool IsBetterCollision(const CollisionDetectResult& a,
                       const CollisionDetectResult& b) {
  if (!b.collided) return true;
  if (!a.collided) return false;
  if ((!a.exempt && b.exempt) ||
      (a.exempt == b.exempt && a.relative_speed_mps > b.relative_speed_mps)) {
    return true;
  }
  return false;
}

}  // namespace

CollisionDetectResult DetectRegulatoryCollision(const proto::MetricFrameInput& input) {
  CollisionDetectResult best;
  const Obb2D ego_box = MakeEgoObb(input);
  (void)kDefaultEgoHeight;

  for (const auto& n : input.npcs()) {
    if (!ObbOverlap(ego_box, MakeNpcObb(n))) continue;
    const CollisionDetectResult info = ClassifyCollision(input, n);
    if (IsBetterCollision(info, best)) {
      best = info;
    }
  }
  return best;
}

}  // namespace grading_mini
