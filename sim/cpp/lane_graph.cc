#include "cpp/lane_graph.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_map>

#include "cpp/proto_io.h"

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

std::vector<std::tuple<double, double, double>> LaneCenterlineTuples(
    const proto::Lane& lane) {
  std::vector<std::tuple<double, double, double>> out;
  out.reserve(static_cast<size_t>(lane.centerline_size()));
  for (const auto& p : lane.centerline()) {
    out.emplace_back(p.x(), p.y(), p.z());
  }
  return out;
}

google::protobuf::RepeatedPtrField<proto::ReferencePoint> PolylineToReference(
    const std::vector<std::tuple<double, double, double>>& pts,
    double speed_mps) {
  google::protobuf::RepeatedPtrField<proto::ReferencePoint> out;
  if (pts.empty()) return out;
  for (size_t i = 0; i < pts.size(); ++i) {
    auto* rp = out.Add();
    rp->set_x(std::get<0>(pts[i]));
    rp->set_y(std::get<1>(pts[i]));
    double heading = 0.0;
    if (i + 1 < pts.size()) {
      const double dx = std::get<0>(pts[i + 1]) - rp->x();
      const double dy = std::get<1>(pts[i + 1]) - rp->y();
      if (std::hypot(dx, dy) > 1e-6) {
        heading = std::atan2(dy, dx);
      }
    } else if (i > 0) {
      const double dx = rp->x() - std::get<0>(pts[i - 1]);
      const double dy = rp->y() - std::get<1>(pts[i - 1]);
      heading = std::atan2(dy, dx);
    }
    rp->set_heading(heading);
    rp->set_speed(speed_mps);
    rp->set_valid(true);
  }
  return out;
}

}  // namespace

bool LaneGraph::LoadFromFile(const std::string& path, LaneGraph* out,
                             std::string* error) {
  proto::StaticMap map;
  if (!ReadStaticMapFromFile(path, &map, error)) return false;
  *out = LaneGraph(std::move(map));
  return true;
}

LaneGraph::LaneGraph(proto::StaticMap map) : map_(std::move(map)) {}

const proto::Lane* LaneGraph::FindLane(int64_t id) const {
  for (const auto& lane : map_.lanes()) {
    if (lane.id() == id) return &lane;
  }
  return nullptr;
}

const proto::Lane* LaneGraph::ClosestLane(double x, double y, double heading,
                                          bool has_heading,
                                          double max_heading_diff) const {
  const proto::Lane* best = nullptr;
  double best_d = std::numeric_limits<double>::infinity();
  for (const auto& lane : map_.lanes()) {
    if (lane.type() == "BIKE_LANE") continue;
    if (lane.centerline_size() < 2) continue;
    for (int i = 0; i + 1 < lane.centerline_size(); ++i) {
      const double x1 = lane.centerline(i).x();
      const double y1 = lane.centerline(i).y();
      const double x2 = lane.centerline(i + 1).x();
      const double y2 = lane.centerline(i + 1).y();
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
    const proto::Lane* cur_lane = FindLane(cur);
    if (!cur_lane) continue;
    for (int64_t nxt : cur_lane->exit_lanes()) {
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
    const proto::Lane* lane = FindLane(lid);
    if (!lane || lane->centerline_size() < 2) continue;
    const auto cl = LaneCenterlineTuples(*lane);
    if (out.empty()) {
      out = cl;
      continue;
    }
    const auto& last = out.back();
    const auto& first = cl.front();
    if (std::hypot(std::get<0>(first) - std::get<0>(last),
                   std::get<1>(first) - std::get<1>(last)) < 0.5) {
      out.insert(out.end(), cl.begin() + 1, cl.end());
    } else {
      out.insert(out.end(), cl.begin(), cl.end());
    }
  }
  return out;
}

double LaneGraph::SpeedLimitMps(const std::vector<int64_t>& lane_ids,
                                double default_kmh) const {
  double min_kmh = std::numeric_limits<double>::infinity();
  for (int64_t id : lane_ids) {
    const proto::Lane* lane = FindLane(id);
    if (!lane) continue;
    min_kmh = std::min(min_kmh, lane->speed_limit_kmh());
  }
  if (!std::isfinite(min_kmh)) return default_kmh / 3.6;
  return min_kmh / 3.6;
}

bool BuildMapReference(const proto::ScenarioMeta& meta, const LaneGraph& graph,
                       double reference_step, proto::MapRouteResult* out,
                       std::string* error) {
  if (!out) {
    if (error) *error = "null MapRouteResult";
    return false;
  }
  out->clear_reference_points();
  out->clear_route_lane_ids();

  if (reference_step < 0.1) {
    if (error) *error = "reference_step must be >= 0.1";
    return false;
  }

  const auto& init = meta.init_pose();
  const auto& goal = meta.goal_pose();

  const proto::Lane* start_lane =
      graph.ClosestLane(init.x(), init.y(), init.yaw(), true, kPi / 2.0);
  if (!start_lane) {
    if (error) *error = "no drivable lane near init_pose for routing";
    return false;
  }

  const proto::Lane* goal_lane =
      graph.ClosestLane(goal.x(), goal.y(), goal.yaw(), true, kPi / 2.0);
  if (!goal_lane) {
    if (error) *error = "no drivable lane near goal_pose for routing";
    return false;
  }

  std::vector<int64_t> route =
      graph.ShortestPath(start_lane->id(), goal_lane->id());
  if (route.empty()) {
    if (error) {
      *error = "no lane path from start lane " + std::to_string(start_lane->id()) +
               " to goal lane " + std::to_string(goal_lane->id());
    }
    return false;
  }

  auto raw = graph.RouteCenterline(route);
  if (raw.size() < 2) {
    if (error) *error = "route centerline has fewer than 2 points";
    return false;
  }

  if (std::hypot(std::get<0>(raw[0]) - init.x(), std::get<1>(raw[0]) - init.y()) >
      0.5) {
    raw.insert(raw.begin(),
               std::make_tuple(init.x(), init.y(), std::get<2>(raw[0])));
  }
  if (std::hypot(std::get<0>(raw.back()) - goal.x(),
                 std::get<1>(raw.back()) - goal.y()) > 0.5) {
    const double z = std::get<2>(raw.back());
    raw.push_back(std::make_tuple(goal.x(), goal.y(), z));
  }

  const double speed_mps = graph.SpeedLimitMps(route);
  auto resampled = ResamplePolyline(raw, reference_step);
  if (resampled.size() < 2) {
    if (error) *error = "resampled reference path has fewer than 2 points";
    return false;
  }

  out->set_speed_limit_mps(speed_mps);
  for (int64_t id : route) {
    out->add_route_lane_ids(id);
  }
  out->mutable_reference_points()->CopyFrom(
      PolylineToReference(resampled, speed_mps));
  return true;
}

std::vector<proto::ReferencePoint> BuildSdcReference(
    const proto::DynamicObjects& dynamic) {
  std::vector<proto::ReferencePoint> reference_points;
  const proto::Track* sdc_track = nullptr;
  for (const auto& tr : dynamic.tracks()) {
    if (tr.is_sdc() || (dynamic.sdc_track_index() >= 0 &&
                        tr.track_index() == dynamic.sdc_track_index())) {
      sdc_track = &tr;
      break;
    }
  }
  if (sdc_track == nullptr || sdc_track->states_size() == 0) {
    return reference_points;
  }

  const int max_steps = sdc_track->states_size();
  reference_points.resize(static_cast<size_t>(max_steps));
  proto::ReferencePoint last_valid;
  bool has_last_valid = false;
  for (int i = 0; i < max_steps; ++i) {
    if (sdc_track->states(i).valid()) {
      const auto& st = sdc_track->states(i);
      proto::ReferencePoint rp;
      rp.set_x(st.x());
      rp.set_y(st.y());
      rp.set_heading(st.yaw());
      rp.set_speed(std::hypot(st.vx(), st.vy()));
      rp.set_valid(true);
      reference_points[static_cast<size_t>(i)] = rp;
      last_valid = rp;
      has_last_valid = true;
    } else if (has_last_valid) {
      reference_points[static_cast<size_t>(i)] = last_valid;
    }
  }
  return reference_points;
}

}  // namespace hyw_sim
