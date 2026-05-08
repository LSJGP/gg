#pragma once

#include "proto/grading/metric_input.pb.h"

namespace grading_mini {

class SimplePlanner {
 public:
  explicit SimplePlanner(double max_speed_mps = 33.3);

  void Plan(proto::MetricFrameInput* input) const;

  double max_speed_mps() const { return max_speed_mps_; }

 private:
  double max_speed_mps_;
};

}  // namespace grading_mini
