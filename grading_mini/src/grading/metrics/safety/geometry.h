#pragma once

#include <array>

#include "proto/grading/metric_input.pb.h"
#include "proto/grading/scene.pb.h"

namespace grading_mini {

struct Obb2D {
  double cx = 0.0;
  double cy = 0.0;
  double heading = 0.0;
  double half_length = 0.0;
  double half_width = 0.0;
};

std::array<std::array<double, 2>, 4> ObbCorners(const Obb2D& box);

Obb2D MakeEgoObb(const proto::MetricFrameInput& input);

double PointToSegmentDist(double px, double py, double x1, double y1, double x2,
                          double y2);

double MinDistToRoadEdges(const proto::SceneMap& map, double x, double y);

}  // namespace grading_mini
