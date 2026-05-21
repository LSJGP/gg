#include "src/grading/metrics/safety/geometry.h"

#include <cmath>
#include <limits>

namespace grading_mini {

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

Obb2D MakeEgoObb(const proto::MetricFrameInput& input) {
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
  return box;
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

double MinDistToRoadEdges(const proto::SceneMap& map, double x, double y) {
  double best = std::numeric_limits<double>::infinity();
  for (const auto& edge : map.road_edges()) {
    if (edge.polyline_size() < 2) continue;
    for (int i = 0; i + 1 < edge.polyline_size(); ++i) {
      const auto& p0 = edge.polyline(i);
      const auto& p1 = edge.polyline(i + 1);
      best = std::min(best, PointToSegmentDist(x, y, p0.x(), p0.y(), p1.x(), p1.y()));
    }
  }
  return best;
}

}  // namespace grading_mini
