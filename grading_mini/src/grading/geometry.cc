#include "src/grading/geometry.h"

#include <cmath>

namespace grading_mini {
namespace {

std::array<std::array<double, 2>, 2> Axes(const Obb2D& box) {
  const double c = std::cos(box.heading);
  const double s = std::sin(box.heading);
  return {{{c, s}, {-s, c}}};
}

std::pair<double, double> Project(const std::array<std::array<double, 2>, 4>& corners,
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

}  // namespace

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
  const auto aa = Axes(a);
  const auto ab = Axes(b);
  const std::array<std::array<double, 2>, 4> axes = {
      aa[0], aa[1], ab[0], ab[1]};
  for (const auto& axis : axes) {
    const auto pa = Project(ca, axis);
    const auto pb = Project(cb, axis);
    if (pa.second < pb.first || pb.second < pa.first) return false;
  }
  return true;
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

}  // namespace grading_mini
