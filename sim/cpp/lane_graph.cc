#include "cpp/lane_graph.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_map>

#include "cpp/json_utils.h"

namespace hyw_sim {
namespace {

constexpr double kPi = 3.14159265358979323846;

double WrapPi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
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

std::vector<std::tuple<double, double, double>> ResamplePolyline(
    const std::vector<std::tuple<double, double, double>>& pts, double step) {
  if (pts.size() < 2) return pts;
  std::vector<std::tuple<double, double, double>> out;
  out.push_back(pts[0]);
  double pending = step;
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const double x1 = std::get<0>(pts[i]);
    const double y1 = std::get<1>(pts[i]);
    const double z1 = std::get<2>(pts[i]);
    const double x2 = std::get<0>(pts[i + 1]);
    const double y2 = std::get<1>(pts[i + 1]);
    const double z2 = std::get<2>(pts[i + 1]);
    const double seg = std::hypot(x2 - x1, y2 - y1);
    if (seg < 1e-9) continue;
    double consumed = 0.0;
    while (consumed + pending <= seg) {
      consumed += pending;
      const double t = consumed / seg;
      out.emplace_back(x1 + t * (x2 - x1), y1 + t * (y2 - y1),
                       z1 + t * (z2 - z1));
      pending = step;
    }
    pending -= (seg - consumed);
    if (pending < 0.0) pending = step;
  }
  const auto& last = pts.back();
  const auto& back = out.back();
  if (std::hypot(std::get<0>(back) - std::get<0>(last),
                 std::get<1>(back) - std::get<1>(last)) > 1e-3) {
    out.push_back(last);
  }
  return out;
}

std::vector<ReferencePoint> PolylineToReference(
    const std::vector<std::tuple<double, double, double>>& pts,
    double speed_mps) {
  std::vector<ReferencePoint> out;
  if (pts.empty()) return out;
  out.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    ReferencePoint rp;
    rp.x = std::get<0>(pts[i]);
    rp.y = std::get<1>(pts[i]);
    double heading = 0.0;
    if (i + 1 < pts.size()) {
      const double dx = std::get<0>(pts[i + 1]) - rp.x;
      const double dy = std::get<1>(pts[i + 1]) - rp.y;
      if (std::hypot(dx, dy) > 1e-6) {
        heading = std::atan2(dy, dx);
      }
    } else if (i > 0) {
      const double dx = rp.x - std::get<0>(pts[i - 1]);
      const double dy = rp.y - std::get<1>(pts[i - 1]);
      heading = std::atan2(dy, dx);
    }
    rp.heading = heading;
    rp.speed = speed_mps;
    rp.valid = true;
    out.push_back(rp);
  }
  return out;
}

}  // namespace

bool LaneGraph::LoadFromFile(const std::string& path, LaneGraph* out,
                             std::string* error) {
  out->lanes_.clear();
  google::protobuf::Struct doc;
  if (!ReadJsonFileToStruct(path, &doc, error)) return false;

  const auto* lanes_raw = GetFieldList(doc, "lanes");
  if (!lanes_raw || lanes_raw->values_size() == 0) {
    if (error) *error = "lane_graph.json missing or empty lanes";
    return false;
  }

  for (const auto& lv : lanes_raw->values()) {
    if (lv.kind_case() != google::protobuf::Value::kStructValue) continue;
    const auto& l = lv.struct_value();
    Lane lane;
    lane.id = static_cast<int64_t>(GetFieldNumber(l, "id", 0.0));
    lane.type = GetFieldString(l, "type", "UNDEFINED");
    lane.speed_limit_kmh = GetFieldNumber(l, "speed_limit_kmh", 50.0);

    if (const auto* cl = GetFieldList(l, "centerline")) {
      for (const auto& pv : cl->values()) {
        if (pv.kind_case() != google::protobuf::Value::kListValue) continue;
        const auto& pt = pv.list_value();
        if (pt.values_size() < 2) continue;
        const double x = GetNumber(pt.values(0), 0.0);
        const double y = GetNumber(pt.values(1), 0.0);
        const double z =
            pt.values_size() > 2 ? GetNumber(pt.values(2), 0.0) : 0.0;
        lane.centerline.emplace_back(x, y, z);
      }
    }

    if (const auto* entries = GetFieldList(l, "entry_lanes")) {
      for (const auto& ev : entries->values()) {
        lane.entry_lanes.push_back(static_cast<int64_t>(GetNumber(ev, 0.0)));
      }
    }
    if (const auto* exits = GetFieldList(l, "exit_lanes")) {
      for (const auto& ev : exits->values()) {
        lane.exit_lanes.push_back(static_cast<int64_t>(GetNumber(ev, 0.0)));
      }
    }
    out->lanes_.push_back(std::move(lane));
  }

  if (out->lanes_.empty()) {
    if (error) *error = "lane_graph.json has no parseable lanes";
    return false;
  }
  return true;
}

const Lane* LaneGraph::FindLane(int64_t id) const {
  for (const auto& lane : lanes_) {
    if (lane.id == id) return &lane;
  }
  return nullptr;
}

const Lane* LaneGraph::ClosestLane(double x, double y, double heading,
                                   bool has_heading,
                                   double max_heading_diff) const {
  const Lane* best = nullptr;
  double best_d = std::numeric_limits<double>::infinity();
  for (const auto& lane : lanes_) {
    if (lane.type == "BIKE_LANE") continue;
    if (lane.centerline.size() < 2) continue;
    for (size_t i = 0; i + 1 < lane.centerline.size(); ++i) {
      const double x1 = std::get<0>(lane.centerline[i]);
      const double y1 = std::get<1>(lane.centerline[i]);
      const double x2 = std::get<0>(lane.centerline[i + 1]);
      const double y2 = std::get<1>(lane.centerline[i + 1]);
      const double d = PointToSegmentDist(x, y, x1, y1, x2, y2);
      if (has_heading) {
        const double seg_h = std::atan2(y2 - y1, x2 - x1);
        if (std::fabs(WrapPi(seg_h - heading)) > max_heading_diff) continue;
      }
      if (d < best_d) {
        best_d = d;
        best = &lane;
      }
    }
  }
  return best;
}

std::vector<int64_t> LaneGraph::ShortestPath(int64_t start_id,
                                             int64_t goal_id) const {
  if (start_id == goal_id) return {start_id};
  std::unordered_map<int64_t, int64_t> prev;
  prev[start_id] = -1;
  std::deque<int64_t> q;
  q.push_back(start_id);
  while (!q.empty()) {
    const int64_t cur = q.front();
    q.pop_front();
    if (cur == goal_id) {
      std::vector<int64_t> path;
      int64_t node = goal_id;
      while (node != -1) {
        path.push_back(node);
        node = prev[node];
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    const Lane* cur_lane = FindLane(cur);
    if (!cur_lane) continue;
    for (int64_t nxt : cur_lane->exit_lanes) {
      if (!FindLane(nxt)) continue;
      if (prev.count(nxt)) continue;
      prev[nxt] = cur;
      q.push_back(nxt);
    }
  }
  return {};
}

std::vector<std::tuple<double, double, double>> LaneGraph::RouteCenterline(
    const std::vector<int64_t>& lane_ids) const {
  std::vector<std::tuple<double, double, double>> out;
  for (int64_t lid : lane_ids) {
    const Lane* lane = FindLane(lid);
    if (!lane || lane->centerline.size() < 2) continue;
    if (out.empty()) {
      out = lane->centerline;
      continue;
    }
    const auto& last = out.back();
    const auto& first = lane->centerline.front();
    if (std::hypot(std::get<0>(first) - std::get<0>(last),
                   std::get<1>(first) - std::get<1>(last)) < 0.5) {
      out.insert(out.end(), lane->centerline.begin() + 1, lane->centerline.end());
    } else {
      out.insert(out.end(), lane->centerline.begin(), lane->centerline.end());
    }
  }
  return out;
}

double LaneGraph::SpeedLimitMps(const std::vector<int64_t>& lane_ids,
                                double default_kmh) const {
  double min_kmh = std::numeric_limits<double>::infinity();
  for (int64_t id : lane_ids) {
    const Lane* lane = FindLane(id);
    if (!lane) continue;
    min_kmh = std::min(min_kmh, lane->speed_limit_kmh);
  }
  if (!std::isfinite(min_kmh)) return default_kmh / 3.6;
  return min_kmh / 3.6;
}

bool BuildMapReference(const Scenario& scenario, const LaneGraph& graph,
                       double reference_step, MapRouteResult* out,
                       std::string* error) {
  if (!out) {
    if (error) *error = "null MapRouteResult";
    return false;
  }
  out->reference_points.clear();
  out->route_lane_ids.clear();

  if (reference_step < 0.1) {
    if (error) *error = "reference_step must be >= 0.1";
    return false;
  }

  const Pose2D& init = scenario.init_pose;
  const Pose2D& goal = scenario.goal_pose;

  const Lane* start_lane =
      graph.ClosestLane(init.x, init.y, init.yaw, true, kPi / 2.0);
  if (!start_lane) {
    if (error) *error = "no drivable lane near init_pose for routing";
    return false;
  }

  const Lane* goal_lane =
      graph.ClosestLane(goal.x, goal.y, goal.yaw, true, kPi / 2.0);
  if (!goal_lane) {
    if (error) *error = "no drivable lane near goal_pose for routing";
    return false;
  }

  std::vector<int64_t> route =
      graph.ShortestPath(start_lane->id, goal_lane->id);
  if (route.empty()) {
    if (error) {
      *error = "no lane path from start lane " + std::to_string(start_lane->id) +
               " to goal lane " + std::to_string(goal_lane->id);
    }
    return false;
  }

  auto raw = graph.RouteCenterline(route);
  if (raw.size() < 2) {
    if (error) *error = "route centerline has fewer than 2 points";
    return false;
  }

  if (std::hypot(std::get<0>(raw[0]) - init.x, std::get<1>(raw[0]) - init.y) >
      0.5) {
    raw.insert(raw.begin(), std::make_tuple(init.x, init.y, std::get<2>(raw[0])));
  }
  if (std::hypot(std::get<0>(raw.back()) - goal.x,
                 std::get<1>(raw.back()) - goal.y) > 0.5) {
    const double z = std::get<2>(raw.back());
    raw.push_back(std::make_tuple(goal.x, goal.y, z));
  }

  const double speed_mps = graph.SpeedLimitMps(route);
  auto resampled = ResamplePolyline(raw, reference_step);
  if (resampled.size() < 2) {
    if (error) *error = "resampled reference path has fewer than 2 points";
    return false;
  }

  out->speed_limit_mps = speed_mps;
  out->route_lane_ids = std::move(route);
  out->reference_points = PolylineToReference(resampled, speed_mps);
  return true;
}

std::vector<ReferencePoint> BuildSdcReference(const Scenario& scenario) {
  std::vector<ReferencePoint> reference_points;
  const Track* sdc_track = nullptr;
  for (const auto& tr : scenario.tracks) {
    if (tr.is_sdc ||
        (scenario.sdc_track_index >= 0 &&
         tr.track_index == scenario.sdc_track_index)) {
      sdc_track = &tr;
      break;
    }
  }
  if (sdc_track == nullptr || sdc_track->states.empty()) {
    return reference_points;
  }

  const size_t max_steps = sdc_track->states.size();
  reference_points.assign(max_steps, ReferencePoint{});
  ReferencePoint last_valid;
  bool has_last_valid = false;
  for (size_t i = 0; i < max_steps; ++i) {
    if (i < sdc_track->states.size() && sdc_track->states[i].valid) {
      const auto& st = sdc_track->states[i];
      ReferencePoint rp;
      rp.x = st.x;
      rp.y = st.y;
      rp.heading = st.yaw;
      rp.speed = std::hypot(st.vx, st.vy);
      rp.valid = true;
      reference_points[i] = rp;
      last_valid = rp;
      has_last_valid = true;
    } else if (has_last_valid) {
      reference_points[i] = last_valid;
    }
  }
  return reference_points;
}

}  // namespace hyw_sim
