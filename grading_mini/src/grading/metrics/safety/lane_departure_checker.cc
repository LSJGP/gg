#include "src/grading/metrics/safety/lane_departure_checker.h"

#include "proto/grading/metrics/safety_metric.pb.h"
#include "spdlog/spdlog.h"

#include <array>
#include <cmath>
#include <limits>

namespace grading_mini {
namespace {

struct Obb2D {
  double cx = 0.0;
  double cy = 0.0;
  double heading = 0.0;
  double half_length = 0.0;
  double half_width = 0.0;
};

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

double PointToSegmentDist(double px, double py, double x1, double y1, double x2,
                          double y2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  const double l2 = dx * dx + dy * dy;
  if (l2 < 1e-12) {
    return std::hypot(px - x1, py - y1);
  }
  const double t =
      std::max(0.0, std::min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2));
  const double qx = x1 + t * dx;
  const double qy = y1 + t * dy;
  return std::hypot(px - qx, py - qy);
}

}  // namespace

REGISTER_METRIC(LaneDepartureChecker, "lane_departure_checker");

absl::Status LaneDepartureChecker::Init(const google::protobuf::Message* config) {
  if (config) {
    if (const auto* typed =
            dynamic_cast<const proto::LaneDepartureCheckerConfig*>(config)) {
      if (typed->min_road_edge_clearance_m() > 0.0) {
        min_road_edge_clearance_m_ = typed->min_road_edge_clearance_m();
      }
      if (typed->min_lane_boundary_clearance_m() >= 0.0) {
        min_lane_boundary_clearance_m_ = typed->min_lane_boundary_clearance_m();
      }
    }
  }
  SPDLOG_INFO(
      "LaneDepartureChecker init: road_edge_clearance={:.2f}m lane_boundary={:.2f}m",
      min_road_edge_clearance_m_, min_lane_boundary_clearance_m_);
  return absl::OkStatus();
}

double LaneDepartureChecker::MinRoadEdgeDistance(const proto::SceneMap& map,
                                               double x, double y) const {
  double best = std::numeric_limits<double>::infinity();
  for (const auto& edge : map.road_edges()) {
    if (edge.polyline_size() < 2) continue;
    for (int i = 0; i + 1 < edge.polyline_size(); ++i) {
      const auto& p0 = edge.polyline(i);
      const auto& p1 = edge.polyline(i + 1);
      best = std::min(best, PointToSegmentDist(x, y, p0.x(), p0.y(), p1.x(), p1.y()));
    }
  }
  for (const auto& line : map.road_lines()) {
    if (line.polyline_size() < 2) continue;
    for (int i = 0; i + 1 < line.polyline_size(); ++i) {
      const auto& p0 = line.polyline(i);
      const auto& p1 = line.polyline(i + 1);
      best = std::min(best, PointToSegmentDist(x, y, p0.x(), p0.y(), p1.x(), p1.y()));
    }
  }
  return best;
}

double LaneDepartureChecker::MinCornerRoadEdgeDistance(
    const MetricFrameInput& input) const {
  if (!has_scene_map_) {
    return std::numeric_limits<double>::infinity();
  }
  const auto& vs = input.vehicle_state();
  const auto& p = input.ego_vehicle();
  const double length = p.length() > 0.0 ? p.length() : 4.5;
  const double width = p.width() > 0.0 ? p.width() : 1.85;
  const double rear = p.rear_overhang() > 0.0 ? p.rear_overhang() : 0.95;

  Obb2D box;
  const double d = length / 2.0 - rear;
  box.cx = vs.x() + d * std::cos(vs.heading());
  box.cy = vs.y() + d * std::sin(vs.heading());
  box.heading = vs.heading();
  box.half_length = length / 2.0;
  box.half_width = width / 2.0;

  double best = std::numeric_limits<double>::infinity();
  for (const auto& corner : ObbCorners(box)) {
    best = std::min(best, MinRoadEdgeDistance(scene_map_, corner[0], corner[1]));
  }
  return best;
}

absl::Status LaneDepartureChecker::CalculateOneFrame(
    const MetricFrameInput& input,
    const std::deque<MetricFrameOutput>& /*history*/,
    MetricFrameOutput* output) {
  total_frames_++;

  if (input.has_scene_map()) {
    scene_map_ = input.scene_map();
    has_scene_map_ = true;
  }

  bool ok = true;
  std::string reason;

  if (input.has_road_context()) {
    const auto& rc = input.road_context();
    if (rc.dist_to_left_boundary_m() < min_lane_boundary_clearance_m_) {
      ok = false;
      reason = "left_lane_boundary";
    }
    if (rc.dist_to_right_boundary_m() < min_lane_boundary_clearance_m_) {
      ok = false;
      reason = "right_lane_boundary";
    }
    if (rc.dist_to_road_edge_m() > 0.0 &&
        rc.dist_to_road_edge_m() < min_road_edge_clearance_m_) {
      ok = false;
      reason = "road_context_edge";
    }
  }

  const double corner_edge = MinCornerRoadEdgeDistance(input);
  if (std::isfinite(corner_edge) &&
      corner_edge < min_road_edge_clearance_m_) {
    ok = false;
    reason = "road_edge_geometry";
  }

  if (!ok) {
    violation_frames_++;
    SPDLOG_WARN("Frame {}: lane departure ({}) edge_dist={:.2f}m", input.frame_id(),
                reason, corner_edge);
  }

  output->set_bool_value(ok);
  return absl::OkStatus();
}

absl::StatusOr<MetricSummary> LaneDepartureChecker::SummarizeResult(
    const std::deque<MetricFrameOutput>& /*history*/) {
  MetricSummary summary;
  summary.set_metric_name(name_);
  summary.set_passed(violation_frames_ == 0);
  summary.set_detail("violation_frames=" + std::to_string(violation_frames_) + "/" +
                     std::to_string(total_frames_));
  return summary;
}

}  // namespace grading_mini
