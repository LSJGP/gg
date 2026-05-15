#include "cpp/geometry.h"

#include <cmath>

namespace hyw_sim {
namespace {

std::array<std::array<double, 2>, 2> Axes(const OBB& box) {
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

std::array<std::array<double, 2>, 4> Corners(const OBB& box) {
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

bool Overlap(const OBB& a, const OBB& b) {
  // 3D overlap = XY OBB overlap + Z interval overlap.
  if (std::fabs(a.cz - b.cz) > (a.half_height + b.half_height)) return false;

  const auto ca = Corners(a);
  const auto cb = Corners(b);
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

}  // namespace hyw_sim
