#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace hyw_sim {

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct TrackState {
  bool valid = false;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double length = 4.5;
  double width = 1.85;
  double height = 1.6;
};

struct Track {
  int64_t track_index = 0;
  int64_t id = 0;
  std::string object_type = "OTHER";
  bool is_sdc = false;
  std::vector<TrackState> states;
};

struct Scenario {
  std::string scenario_id;
  Pose2D init_pose;
  Pose2D goal_pose;
  std::vector<double> timestamps_seconds;
  int64_t current_time_index = 0;
  int64_t sdc_track_index = -1;
  std::vector<Track> tracks;
};

struct NPCSnapshot {
  int64_t id = 0;
  std::string object_type = "OTHER";
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double heading = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double length = 4.5;
  double width = 1.85;
  double height = 1.6;
};

struct VehicleState {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double acceleration = 0.0;
  double steer = 0.0;
};

struct PlanCommand {
  double target_acceleration = 0.0;
  double steering_angle = 0.0;
  double desired_speed_mps = 0.0;
};

struct CollisionInfo {
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

struct FrameRecord {
  int64_t frame_id = 0;
  int64_t timestamp_us = 0;
  VehicleState ego;
  PlanCommand command;
  CollisionInfo collision;
  int num_npcs = 0;
};

struct VehicleParams {
  double length = 4.5;
  double width = 1.85;
  double wheelbase = 2.7;
  double rear_overhang = 0.95;
  double max_speed = 33.3;
  double max_accel = 2.5;
  double max_decel = 6.0;
  double max_steer = 0.6108652382;       // 35 deg
  double max_steer_rate = 3.1415926535;  // 180 deg/s
};

}  // namespace hyw_sim
