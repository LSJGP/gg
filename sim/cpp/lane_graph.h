#pragma once

#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#include "cpp/planner.h"
#include "cpp/types.h"

namespace hyw_sim {

struct Lane {
  int64_t id = 0;
  std::string type = "UNDEFINED";
  double speed_limit_kmh = 50.0;
  std::vector<std::tuple<double, double, double>> centerline;
  std::vector<int64_t> entry_lanes;
  std::vector<int64_t> exit_lanes;
};

class LaneGraph {
 public:
  static bool LoadFromFile(const std::string& path, LaneGraph* out, std::string* error);

  const Lane* FindLane(int64_t id) const;

  const Lane* ClosestLane(double x, double y, double heading,
                          bool has_heading = true,
                          double max_heading_diff = 1.5707963268) const;

  std::vector<int64_t> ShortestPath(int64_t start_id, int64_t goal_id) const;

  std::vector<std::tuple<double, double, double>> RouteCenterline(
      const std::vector<int64_t>& lane_ids) const;

  double SpeedLimitMps(const std::vector<int64_t>& lane_ids,
                       double default_kmh = 50.0) const;

  size_t LaneCount() const { return lanes_.size(); }

 private:
  std::vector<Lane> lanes_;
};

struct MapRouteResult {
  std::vector<ReferencePoint> reference_points;
  double speed_limit_mps = 13.9;
  std::vector<int64_t> route_lane_ids;
};

/// Build reference polyline from lane_graph (init→goal). Fails on routing errors.
bool BuildMapReference(const Scenario& scenario, const LaneGraph& graph,
                       double reference_step, MapRouteResult* out,
                       std::string* error);

std::vector<ReferencePoint> BuildSdcReference(const Scenario& scenario);

}  // namespace hyw_sim
