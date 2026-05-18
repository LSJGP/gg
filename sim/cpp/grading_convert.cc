#include "cpp/grading_convert.h"

namespace hyw_sim {
namespace {

void CopyVec3(const proto::Vec3& src, grading_mini::proto::Vec3* dst) {
  dst->set_x(src.x());
  dst->set_y(src.y());
  dst->set_z(src.z());
}

void CopyBoundary(const proto::BoundarySegment& src,
                  grading_mini::proto::BoundarySegment* dst) {
  dst->set_lane_start_index(src.lane_start_index());
  dst->set_lane_end_index(src.lane_end_index());
  dst->set_boundary_feature_id(src.boundary_feature_id());
  dst->set_boundary_type(src.boundary_type());
}

grading_mini::proto::SceneMap ToSceneMap(const proto::StaticMap& map) {
  grading_mini::proto::SceneMap out;
  out.set_source(map.source());
  out.set_scenario_id(map.scenario_id());
  for (const auto& lane : map.lanes()) {
    auto* l = out.add_lanes();
    l->set_id(lane.id());
    l->set_type(lane.type());
    l->set_speed_limit_kmh(lane.speed_limit_kmh());
    for (const auto& p : lane.centerline()) {
      CopyVec3(p, l->add_centerline());
    }
    for (const auto& b : lane.left_boundaries()) {
      CopyBoundary(b, l->add_left_boundaries());
    }
    for (const auto& b : lane.right_boundaries()) {
      CopyBoundary(b, l->add_right_boundaries());
    }
  }
  for (const auto& rl : map.road_lines()) {
    auto* o = out.add_road_lines();
    o->set_id(rl.id());
    o->set_type(rl.type());
    for (const auto& p : rl.polyline()) {
      CopyVec3(p, o->add_polyline());
    }
  }
  for (const auto& re : map.road_edges()) {
    auto* o = out.add_road_edges();
    o->set_id(re.id());
    o->set_type(re.type());
    for (const auto& p : re.polyline()) {
      CopyVec3(p, o->add_polyline());
    }
  }
  for (const auto& cw : map.crosswalks()) {
    auto* o = out.add_crosswalks();
    o->set_id(cw.id());
    for (const auto& p : cw.polygon()) {
      CopyVec3(p, o->add_polygon());
    }
  }
  return out;
}

void CopyRoadContext(const proto::RoadContext& src,
                     grading_mini::proto::RoadContext* dst) {
  dst->set_dist_to_left_boundary_m(src.dist_to_left_boundary_m());
  dst->set_dist_to_right_boundary_m(src.dist_to_right_boundary_m());
  dst->set_dist_to_road_edge_m(src.dist_to_road_edge_m());
  dst->set_closest_lane_id(src.closest_lane_id());
  dst->set_lateral_offset_m(src.lateral_offset_m());
}

void CopyNpc(const proto::NpcSnapshot& src, grading_mini::proto::NpcState* dst) {
  dst->set_id(src.id());
  dst->set_object_type(src.object_type());
  dst->set_x(src.x());
  dst->set_y(src.y());
  dst->set_z(src.z());
  dst->set_heading(src.heading());
  dst->set_vx(src.vx());
  dst->set_vy(src.vy());
  dst->set_length(src.length());
  dst->set_width(src.width());
  dst->set_height(src.height());
}

}  // namespace

grading_mini::proto::MetricFrameInput ToMetricFrameInput(
    const proto::FrameRecord& frame, const proto::StaticMap* scene_map,
    const proto::VehicleParams& ego_params) {
  grading_mini::proto::MetricFrameInput out;
  out.set_frame_id(frame.frame_id());
  out.set_timestamp_us(frame.timestamp_us());

  const auto& ego = frame.ego();
  auto* vs = out.mutable_vehicle_state();
  vs->set_x(ego.x());
  vs->set_y(ego.y());
  vs->set_heading(ego.heading());
  vs->set_speed(ego.speed());
  vs->set_acceleration(ego.acceleration());
  vs->set_steer(ego.steer());

  auto* pc = out.mutable_planning_command();
  pc->set_desired_speed_mps(frame.command().desired_speed_mps());

  for (const auto& n : frame.npcs()) {
    CopyNpc(n, out.add_npcs());
  }

  if (scene_map != nullptr) {
    *out.mutable_scene_map() = ToSceneMap(*scene_map);
  }

  auto* ev = out.mutable_ego_vehicle();
  ev->set_length(ego_params.length());
  ev->set_width(ego_params.width());
  ev->set_wheelbase(ego_params.wheelbase());
  ev->set_rear_overhang(ego_params.rear_overhang());

  CopyRoadContext(frame.road(), out.mutable_road_context());

  return out;
}

}  // namespace hyw_sim
