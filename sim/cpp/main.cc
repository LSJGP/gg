#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cpp/grading_bridge.h"
#include "cpp/input_format.h"
#include "cpp/lane_graph.h"
#include "cpp/planner.h"
#include "cpp/proto_io.h"
#include "cpp/scenario_loader.h"
#include "cpp/sim_logger.h"
#include "cpp/world.h"

namespace fs = std::filesystem;

namespace {

constexpr double kEgoMaxAccelMps2 = 2.5;
constexpr double kEgoMaxDecelMps2 = 6.0;
constexpr double kEgoMaxSteerRad = 35.0 * M_PI / 180.0;
constexpr double kEgoMaxSteerRateRadPerS = 180.0 * M_PI / 180.0;

using Clock = std::chrono::steady_clock;

double MsSince(const Clock::time_point& t0, const Clock::time_point& t1) {
  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

struct Args {
  std::string scenario_dir;
  std::string output = "../output/log/sim_log.json";
  std::string source_tag = "waymo_sim_cpp";
  double dt = 0.1;
  double max_seconds = 0.0;
  std::string planner = "local_dwa";
  std::string reference_source = "map";
  double reference_step = 1.0;
  double desired_speed = 13.9;
  double ego_length = 4.5;
  double ego_width = 1.85;
  double ego_wheelbase = 2.7;
  double ego_rear_overhang = 0.95;
  double ego_max_speed = 33.3;
  std::string grading_bin;
  std::string grading_report;
  std::string cpp_mode = "online";
  std::string log_dir;
  std::string log_level = "info";
  std::string metrics_config;
  std::string scenario_load = "bulk";
  std::string input_format = "auto";
  bool benchmark = false;
};

void PrintUsage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " --scenario-dir <dir> [options]\n"
      << "  --scenario-load <bulk|stream>  (default: bulk)\n"
      << "  --input-format <auto|json|proto>  (default: auto)\n"
      << "  --benchmark  (implies --cpp-mode off, skip sim_log write)\n"
      << "  --output <path>\n"
      << "  --dt <seconds>\n"
      << "  --max-seconds <seconds>\n"
      << "  --planner <name>  (default: local_dwa)\n"
      << "  --reference-source <map|sdc>  (default: map)\n"
      << "  --reference-step <meters>  (default: 1.0, map mode)\n"
      << "  --desired-speed <mps>\n"
      << "  --grading-bin <path-to-grading_main>\n"
      << "  --grading-report <report-path>\n"
      << "  --metrics-config <grading_metrics.json>\n"
      << "  --cpp-mode <online|offline|both|off>\n"
      << "  --log-dir <dir>\n"
      << "  --log-level <trace|debug|info|warn|error|off>\n";
}

bool ParseArgs(int argc, char** argv, Args* args) {
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + name);
      }
      return argv[++i];
    };
    if (k == "--scenario-dir") {
      args->scenario_dir = next("--scenario-dir");
    } else if (k == "--scenario-load") {
      args->scenario_load = next("--scenario-load");
    } else if (k == "--input-format") {
      args->input_format = next("--input-format");
    } else if (k == "--benchmark") {
      args->benchmark = true;
    } else if (k == "--output") {
      args->output = next("--output");
    } else if (k == "--source-tag") {
      args->source_tag = next("--source-tag");
    } else if (k == "--dt") {
      args->dt = std::stod(next("--dt"));
    } else if (k == "--max-seconds") {
      args->max_seconds = std::stod(next("--max-seconds"));
    } else if (k == "--planner") {
      args->planner = next("--planner");
    } else if (k == "--reference-source") {
      args->reference_source = next("--reference-source");
    } else if (k == "--reference-step") {
      args->reference_step = std::stod(next("--reference-step"));
    } else if (k == "--desired-speed") {
      args->desired_speed = std::stod(next("--desired-speed"));
    } else if (k == "--ego-length") {
      args->ego_length = std::stod(next("--ego-length"));
    } else if (k == "--ego-width") {
      args->ego_width = std::stod(next("--ego-width"));
    } else if (k == "--ego-wheelbase") {
      args->ego_wheelbase = std::stod(next("--ego-wheelbase"));
    } else if (k == "--ego-rear-overhang") {
      args->ego_rear_overhang = std::stod(next("--ego-rear-overhang"));
    } else if (k == "--ego-max-speed") {
      args->ego_max_speed = std::stod(next("--ego-max-speed"));
    } else if (k == "--grading-bin") {
      args->grading_bin = next("--grading-bin");
    } else if (k == "--grading-report") {
      args->grading_report = next("--grading-report");
    } else if (k == "--cpp-mode") {
      args->cpp_mode = next("--cpp-mode");
    } else if (k == "--log-dir") {
      args->log_dir = next("--log-dir");
    } else if (k == "--log-level") {
      args->log_level = next("--log-level");
    } else if (k == "--metrics-config") {
      args->metrics_config = next("--metrics-config");
    } else if (k == "-h" || k == "--help") {
      PrintUsage(argv[0]);
      return false;
    } else {
      throw std::runtime_error("unknown arg: " + k);
    }
  }
  return !args->scenario_dir.empty();
}

hyw_sim::ScenarioLoadMode ParseLoadMode(const std::string& s) {
  if (s == "bulk") return hyw_sim::ScenarioLoadMode::kBulk;
  if (s == "stream") return hyw_sim::ScenarioLoadMode::kStream;
  throw std::runtime_error("unknown --scenario-load: " + s + " (use bulk or stream)");
}

hyw_sim::proto::VehicleParams MakeVehicleParams(const Args& args) {
  hyw_sim::proto::VehicleParams params;
  params.set_length(args.ego_length);
  params.set_width(args.ego_width);
  params.set_wheelbase(args.ego_wheelbase);
  params.set_rear_overhang(args.ego_rear_overhang);
  params.set_max_speed(args.ego_max_speed);
  params.set_max_accel(kEgoMaxAccelMps2);
  params.set_max_decel(kEgoMaxDecelMps2);
  params.set_max_steer(kEgoMaxSteerRad);
  params.set_max_steer_rate(kEgoMaxSteerRateRadPerS);
  return params;
}

void PrintBenchmarkJson(const std::string& scenario_load,
                        const std::string& input_format,
                        const std::string& scenario_dir, double load_meta_map_ms,
                        double load_dynamic_ms, double load_frames_ms,
                        double sim_loop_ms, double total_ms, size_t frames) {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "{\"scenario_load\":\"" << scenario_load << "\""
            << ",\"input_format\":\"" << input_format << "\""
            << ",\"scenario_dir\":\"" << hyw_sim::SimFileLogger::EscapeJsonString(scenario_dir)
            << "\""
            << ",\"load_meta_map_ms\":" << load_meta_map_ms
            << ",\"load_dynamic_ms\":" << load_dynamic_ms
            << ",\"load_frames_ms\":" << load_frames_ms
            << ",\"sim_loop_ms\":" << sim_loop_ms
            << ",\"total_ms\":" << total_ms
            << ",\"frames\":" << frames << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  const auto total_t0 = Clock::now();
  Args args;
  try {
    if (!ParseArgs(argc, argv, &args)) {
      PrintUsage(argv[0]);
      return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << "[sim_cpp] bad args: " << e.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }

  if (args.benchmark) {
    args.cpp_mode = "off";
    args.log_dir.clear();
  }

  hyw_sim::ScenarioLoadMode load_mode;
  hyw_sim::ScenarioInputFormat input_format;
  try {
    load_mode = ParseLoadMode(args.scenario_load);
    input_format = hyw_sim::ParseInputFormat(args.input_format);
  } catch (const std::exception& e) {
    std::cerr << "[sim_cpp] " << e.what() << "\n";
    return 1;
  }

  hyw_sim::ScenarioBundle bundle;
  std::unique_ptr<hyw_sim::DynamicNpcSource> dynamic_source;
  std::string err;

  const auto meta_map_t0 = Clock::now();
  if (!hyw_sim::LoadScenarioMetaAndMap(args.scenario_dir, input_format, &bundle.meta,
                                       &bundle.map, &err)) {
    std::cerr << "[sim_cpp] failed loading scenario meta/map: " << err << "\n";
    return 2;
  }
  const auto meta_map_t1 = Clock::now();
  const double load_meta_map_ms = MsSince(meta_map_t0, meta_map_t1);

  const auto dynamic_t0 = Clock::now();
  if (load_mode == hyw_sim::ScenarioLoadMode::kBulk) {
    const fs::path objs_path = hyw_sim::ResolveScenarioFile(
        fs::path(args.scenario_dir), "dynamic_objects", input_format);
    if (!hyw_sim::ReadDynamicObjectsFromFile(objs_path.string(), &bundle.dynamic,
                                             &err)) {
      std::cerr << "[sim_cpp] failed loading dynamic_objects: " << err << "\n";
      return 2;
    }
    if (bundle.dynamic.timestamps_seconds_size() == 0) {
      std::cerr << "[sim_cpp] dynamic_objects missing timestamps_seconds\n";
      return 2;
    }
    dynamic_source = hyw_sim::CreateBulkDynamicSource(std::move(bundle.dynamic));
  } else {
    dynamic_source =
        hyw_sim::CreateStreamDynamicSource(args.scenario_dir, input_format, &err);
    if (!dynamic_source) {
      std::cerr << "[sim_cpp] failed loading stream dynamic: " << err << "\n";
      return 2;
    }
    if (dynamic_source->timestamps().empty()) {
      std::cerr << "[sim_cpp] stream header missing timestamps_seconds\n";
      return 2;
    }
  }
  const auto dynamic_t1 = Clock::now();
  const double load_dynamic_ms = MsSince(dynamic_t0, dynamic_t1);

  const hyw_sim::proto::VehicleParams params = MakeVehicleParams(args);
  hyw_sim::LaneGraph lane_graph(std::move(bundle.map));

  hyw_sim::proto::PlannerInputs planner_inputs;
  *planner_inputs.mutable_goal() = bundle.meta.goal_pose();
  planner_inputs.mutable_trajectory_config()->set_horizon_s(3.0);
  planner_inputs.mutable_trajectory_config()->set_point_dt_s(args.dt);
  double route_speed_mps = args.desired_speed;

  if (args.reference_source == "map") {
    hyw_sim::proto::MapRouteResult route;
    if (!hyw_sim::BuildMapReference(bundle.meta, lane_graph, args.reference_step,
                                    &route, &err)) {
      std::cerr << "[sim_cpp] fail: " << err << "\n";
      return 2;
    }
    planner_inputs.mutable_reference_points()->CopyFrom(route.reference_points());
    route_speed_mps = route.speed_limit_mps();
    if (!args.benchmark) {
      std::cout << "[sim_cpp] route: " << route.route_lane_ids_size() << " lanes, "
                << route.reference_points_size() << " ref points, limit="
                << (route_speed_mps * 3.6) << " km/h\n";
    }
  } else if (args.reference_source == "sdc") {
    hyw_sim::proto::Track sdc_track;
    std::vector<hyw_sim::proto::ReferencePoint> sdc_ref;
    if (load_mode == hyw_sim::ScenarioLoadMode::kBulk) {
      sdc_ref = hyw_sim::BuildSdcReference(bundle.dynamic);
    } else if (dynamic_source->GetSdcTrack(&sdc_track)) {
      sdc_ref = hyw_sim::BuildSdcReferenceFromTrack(sdc_track);
    }
    if (sdc_ref.empty()) {
      std::cerr << "[sim_cpp] fail: no SDC track for --reference-source sdc\n";
      return 2;
    }
    planner_inputs.mutable_reference_points()->Clear();
    for (const auto& rp : sdc_ref) {
      *planner_inputs.mutable_reference_points()->Add() = rp;
    }
    if (!args.benchmark) {
      std::cout << "[sim_cpp] reference: SDC track (" << sdc_ref.size()
                << " points)\n";
    }
  } else {
    std::cerr << "[sim_cpp] fail: unknown --reference-source "
              << args.reference_source << " (use map or sdc)\n";
    return 2;
  }

  planner_inputs.set_desired_speed_mps(
      std::min(args.desired_speed, route_speed_mps));
  *planner_inputs.mutable_ego_vehicle() = params;

  double initial_ego_speed_mps = 0.0;
  for (const auto& rp : planner_inputs.reference_points()) {
    if (rp.valid() && rp.speed() > 0.5) {
      initial_ego_speed_mps =
          std::min(params.max_speed(), std::max(1.0, 0.35 * rp.speed()));
      break;
    }
  }

  auto planner = hyw_sim::CreatePlanner(args.planner, planner_inputs, &err);
  if (!planner) {
    std::cerr << "[sim_cpp] failed to create planner: " << err << "\n";
    return 2;
  }
  if (!args.benchmark) {
    std::cout << "[sim_cpp] planner=" << planner->Name()
              << " scenario_load=" << args.scenario_load << "\n";
  }

  hyw_sim::SimFileLogger sim_logger;
  if (!args.benchmark && !args.log_dir.empty()) {
    const auto ll = hyw_sim::ParseSimLogLevel(args.log_level);
    if (ll != hyw_sim::SimLogLevel::kOff) {
      if (!sim_logger.Open(fs::path(args.log_dir), ll)) {
        std::cerr << "[sim_cpp] failed to open json log under --log-dir\n";
        return 7;
      }
    }
  }

  hyw_sim::proto::WorldConfig cfg;
  cfg.set_dt(args.dt);
  cfg.set_max_seconds(args.max_seconds);
  cfg.set_initial_ego_speed_mps(initial_ego_speed_mps);

  hyw_sim::WorldSimulator world(bundle.meta, std::move(dynamic_source), lane_graph,
                                params);

  hyw_sim::StreamPipeWriter stream_writer;
  const bool enable_online =
      !args.benchmark && !args.grading_bin.empty() &&
      (args.cpp_mode == "online" || args.cpp_mode == "both");

  std::string report_path = args.grading_report;
  if (!args.benchmark && report_path.empty()) {
    const fs::path outp(args.output);
    const fs::path log_dir = outp.parent_path();
    if (log_dir.filename() == "log") {
      report_path =
          (log_dir.parent_path() / "report" / "grading_report.json").string();
    } else {
      report_path = (outp.parent_path() / "grading_report.json").string();
    }
  }
  if (enable_online) {
    std::error_code mk_ec;
    fs::create_directories(fs::path(report_path).parent_path(), mk_ec);
    if (!stream_writer.Start(args.grading_bin, report_path, args.metrics_config,
                             &lane_graph.map(), params, &err)) {
      std::cerr << "[sim_cpp] failed to start grading stream: " << err << "\n";
      return 3;
    }
  }

  hyw_sim::WorldStepHooks step_hooks;
  step_hooks.on_frame = [&](const hyw_sim::proto::FrameRecord& fr) {
    if (enable_online) {
      stream_writer.EnqueueFrame(fr);
    }
    if (sim_logger.IsOpen()) {
      sim_logger.LogProto(hyw_sim::SimLogLevel::kDebug, "frame_record", fr);
    }
  };

  const hyw_sim::WorldStepHooks* hooks_ptr = nullptr;
  if (enable_online || sim_logger.IsOpen()) {
    hooks_ptr = &step_hooks;
  }

  const auto sim_t0 = Clock::now();
  const auto records = world.Run(*planner, cfg, hooks_ptr);
  const auto sim_t1 = Clock::now();

  const double sim_loop_ms = MsSince(sim_t0, sim_t1);
  const double load_frames_ms =
      static_cast<double>(world.stream_io_us()) / 1000.0;
  const double total_ms = MsSince(total_t0, Clock::now());

  if (args.benchmark) {
    PrintBenchmarkJson(args.scenario_load, args.input_format, args.scenario_dir,
                       load_meta_map_ms,
                       load_dynamic_ms, load_frames_ms, sim_loop_ms, total_ms,
                       records.size());
    return 0;
  }

  if (enable_online) {
    if (!stream_writer.Finish(&err)) {
      std::cerr << "[sim_cpp] grading stream finish failed: " << err << "\n";
      return 4;
    }
  }
  stream_writer.Close();

  if (!hyw_sim::WriteSimLogJson(args.output, args.source_tag, records, lane_graph.map(),
                                params, &err)) {
    std::cerr << "[sim_cpp] failed writing simlog: " << err << "\n";
    return 5;
  }
  std::cout << "[sim_cpp] wrote " << args.output << " (" << records.size()
            << " frames)\n";

  PrintBenchmarkJson(args.scenario_load, args.input_format, args.scenario_dir,
                     load_meta_map_ms, load_dynamic_ms, load_frames_ms, sim_loop_ms,
                     total_ms, records.size());

  if (!args.grading_bin.empty() &&
      (args.cpp_mode == "offline" || args.cpp_mode == "both")) {
    if (!hyw_sim::RunBatchGrading(args.grading_bin, args.output, report_path,
                                  args.metrics_config, &err)) {
      std::cerr << "[sim_cpp] offline grading failed: " << err << "\n";
      return 6;
    }
  }
  return 0;
}