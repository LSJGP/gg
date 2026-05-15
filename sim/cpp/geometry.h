#pragma once

#include <array>

namespace hyw_sim {

struct OBB {
  double cx = 0.0;
  double cy = 0.0;
  double cz = 0.0;
  double heading = 0.0;
  double half_length = 0.0;
  double half_width = 0.0;
  double half_height = 0.0;
};

std::array<std::array<double, 2>, 4> Corners(const OBB& box);
bool Overlap(const OBB& a, const OBB& b);

}  // namespace hyw_sim
