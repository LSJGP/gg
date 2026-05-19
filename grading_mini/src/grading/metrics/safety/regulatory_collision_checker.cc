#include "src/grading/metrics/safety/regulatory_collision_checker.h"

#include <array>
#include <cmath>

#include "spdlog/spdlog.h"

namespace grading_mini {
namespace {

struct Obb2D {
  double cx = 0.0;
  double cy = 0.0;
  double heading = 0.0;
  double half_length = 0.0;
  double half_width = 0.0;
};

std::array<std::array<double, 2>, 2> ObbAxes(const Obb2D& box) {
  const double c = std::cos(box.heading);
  const double s = std::sin(box.heading);
  return {{{c, s}, {-s, c}}};
}

std::pair<double, double> ProjectOnAxis(
    const std::array<std::array<double, 2>, 4>& corners,
    const std::array<double, 2>& axis) {
  double mn = corners[0][0] * axis[0] + corners[0][1] * axis[1];
  double mx = mn;
  for (int i = 1; i < 4; ++i) {
    const double v = corners[i][0] * axis[0] + corners[i][1] * axis[1];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  return {mn, mx};
}

std::array<std::array<double, 2>, 4> ObbCorners(const Obb2D& box) {
  const double c = std::cos(box.heading);
  const double s = std::sin(box.heading);
  const double l = box.half_length;
  const double w = box.half_width;
  std::array<std::array<double, 2>, 4> out{};
  const std::array<std::array<double, 2>, 4> local = {
      std::array<double, 2>{l, w},
      std::array<double, 2>{l, -w},
      std::array<double, 2>{-l, -w},
      std::array<double, 2>{-l, w},
  };
  for (int i = 0; i < 4; ++i) {
    const double lx = local[i][0];
    const double ly = local[i][1];
    out[i][0] = box.cx + c * lx - s * ly;
    out[i][1] = box.cy + s * lx + c * ly;
  }
  return out;
}

bool ObbOverlap(const Obb2D& a, const Obb2D& b) {
  const auto ca = ObbCorners(a);
  const auto cb = ObbCorners(b);
  const auto aa = ObbAxes(a);
  const auto ab = ObbAxes(b);
  const std::array<std::array<double, 2>, 4> axes = {
      aa[0], aa[1], ab[0], ab[1]};
  for (const auto& axis : axes) {
    const auto pa = ProjectOnAxis(ca, axis);
    const auto pb = ProjectOnAxis(cb, axis);
    if (pa.second < pb.first || pb.second < pa.first) return false;
  }
  return true;
}

constexpr double kExemptRearEndEgoSpeedMax = 5.0;
constexpr double kExemptRearEndRelSpeedMin = 2.0;
constexpr double kExemptCutInLatVelMin = 0.5;
constexpr double kExemptHeadOnAngleDeg = 135.0;

struct CollisionDetectResult {
  bool collided = false;
  int64_t other_id = 0;
  std::string kind;
  bool ego_at_fault = true;
  bool exempt = false;
  std::string exempt_reason;
  double relative_speed_mps = 0.0;
  double ego_speed_mps = 0.0;
  double approach_angle_deg = 0.0;
};

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

CollisionDetectResult DetectRegulatoryCollision(const proto::MetricFrameInput& input) {
  CollisionDetectResult best;
  const Obb2D ego_box = MakeEgoObb(input);

  for (const auto& n : input.npcs()) {
    if (!ObbOverlap(ego_box, MakeNpcObb(n))) continue;
    const CollisionDetectResult info = ClassifyCollision(input, n);
    if (IsBetterCollision(info, best)) {
      best = info;
    }
  }
  return best;
}

}  // namespace

REGISTER_METRIC(RegulatoryCollisionChecker, "regulatory_collision_checker");

absl::Status RegulatoryCollisionChecker::Init(
    const google::protobuf::Message* /*config*/) {
  SPDLOG_INFO(
      "RegulatoryCollisionChecker init: detects collision from ego/npc geometry");
  return absl::OkStatus();
}

absl::Status RegulatoryCollisionChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  total_frames_++;

  const CollisionDetectResult det = DetectRegulatoryCollision(input);
  if (!det.collided) {
    output->set_bool_value(true);
    return absl::OkStatus();
  }

  collision_frames_++;
  if (det.exempt) {
    exempt_frames_++;
    output->set_bool_value(true);
    SPDLOG_WARN(
        "Frame {}: collision EXEMPT ({}) other_id={} ego_v={:.2f} rel_v={:.2f}",
        input.frame_id(), det.exempt_reason, det.other_id, det.ego_speed_mps,
        det.relative_speed_mps);
  } else {
    non_exempt_frames_++;
    output->set_bool_value(false);
    SPDLOG_ERROR(
        "Frame {}: COLLISION at_fault kind={} other_id={} ego_v={:.2f} "
        "rel_v={:.2f} approach={:.1f}deg",
        input.frame_id(), det.kind, det.other_id, det.ego_speed_mps,
        det.relative_speed_mps, det.approach_angle_deg);
  }
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> RegulatoryCollisionChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  MetricSummary summary;
  summary.set_metric_name(name_);
  summary.set_passed(non_exempt_frames_ == 0);
  summary.set_detail(
      "non_exempt=" + std::to_string(non_exempt_frames_) +
      " exempt=" + std::to_string(exempt_frames_) +
      " total_collision_frames=" + std::to_string(collision_frames_) + "/" +
      std::to_string(total_frames_));
  return summary;
}

}  // namespace grading_mini
