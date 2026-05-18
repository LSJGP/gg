#pragma once

#include <string>

#include "proto/grading/metric_input.pb.h"

namespace grading_mini {

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

CollisionDetectResult DetectRegulatoryCollision(const proto::MetricFrameInput& input);

}  // namespace grading_mini
