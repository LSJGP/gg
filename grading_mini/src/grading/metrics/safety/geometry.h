#pragma once

#include <array>

namespace grading_mini {

struct Obb2D {
  double cx = 0.0;
  double cy = 0.0;
  double heading = 0.0;
  double half_length = 0.0;
  double half_width = 0.0;
};

std::array<std::array<double, 2>, 4> ObbCorners(const Obb2D& box);
bool ObbOverlap(const Obb2D& a, const Obb2D& b);

double PointToSegmentDist(double px, double py, double x1, double y1, double x2,
                          double y2);

}  // namespace grading_mini