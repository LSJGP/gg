#include <algorithm>
#include <filesystem>
#include <functional>
#include <iostream>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "cpp/grading_bridge.h"
#include "cpp/planner.h"
#include "cpp/scenario_loader.h"
#include "cpp/sim_logger.h"
#include "cpp/types.h"
#include "cpp/world.h"

namespace fs = std::filesystem;

namespace {

struct Args {
  std::string scenario_dir;
  /// Default: repo `output/log/sim_log.json` when cwd is `sim/` (typical `bazel run`).
  std::string output = "../output/log/sim_log.json";
  std::string source_tag = "waymo_sim_cpp";
  double dt = 0.1;
  double max_seconds = 0.0;
  bool stop_on_collision = false;
  std::string planner = "reference_tracker";
  double desired_speed = 13.9;
  double ego_length = 4.5;
  double ego_width = 1.85;
  double ego_wheelbase = 2.7;
  double ego_rear_overhang = 0.95;
  double ego_max_speed = 33.3;
  std::string grading_bin;
  std::string grading_report;
  std::string cpp_mode = "online";  // online/offline/both/off
  std::string log_dir;
  std::string log_level = "info";
  std::string metrics_config;
};

void PrintUsage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " --scenario-dir <dir> [options]\n"
      << "  --output <path>\n"
      << "  --dt <seconds>\n"
      << "  --max-seconds <seconds>\n"
      << "  --stop-on-collision\n"
      << "  --planner <name>\n"
      << "  --desired-speed <mps>\n"
      << "  --grading-bin <path-to-grading_main>\n"
      << "  --grading-report <report-path>\n"
      << "  --metrics-config <grading_metrics.json>  (passed to grading_main)\n"
      << "  --cpp-mode <online|offline|both|off>\n"
      << "  --log-dir <dir>   (optional; spdlog file sim_YYYYMMDD_HHMMSS.log)\n"
      << "  --log-level <trace|debug|info|warn|error|off>  (default: info when "
         "--log-dir set)\n";
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
    } else if (k == "--output") {
      args->output = next("--output");
    } else if (k == "--source-tag") {
      args->source_tag = next("--source-tag");
    } else if (k == "--dt") {
      args->dt = std::stod(next("--dt"));
    } else if (k == "--max-seconds") {
      args->max_seconds = std::stod(next("--max-seconds"));
    } else if (k == "--stop-on-collision") {
      args->stop_on_collision = true;
    } else if (k == "--planner") {
      args->planner = next("--planner");
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

}  // namespace

int main(int argc, char** argv) {
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

  hyw_sim::Scenario scenario;
  std::string err;
  if (!hyw_sim::LoadScenarioFromDir(args.scenario_dir, &scenario, &err)) {
    std::cerr << "[sim_cpp] failed loading scenario: " << err << "\n";
    return 2;
  }

  hyw_sim::VehicleParams params;
  params.length = args.ego_length;
  params.width = args.ego_width;
  params.wheelbase = args.ego_wheelbase;
  params.rear_overhang = args.ego_rear_overhang;
  params.max_speed = args.ego_max_speed;

  std::vector<hyw_sim::ReferencePoint> reference_points;
  size_t max_steps = scenario.timestamps_seconds.size();
  const hyw_sim::Track* sdc_track = nullptr;
  for (const auto& tr : scenario.tracks) {
    if (tr.is_sdc ||
        (scenario.sdc_track_index >= 0 && tr.track_index == scenario.sdc_track_index)) {
      sdc_track = &tr;
      break;
    }
  }
  if (sdc_track != nullptr && !sdc_track->states.empty()) {
    max_steps = std::max(max_steps, sdc_track->states.size());
    reference_points.assign(max_steps, hyw_sim::ReferencePoint{});
    hyw_sim::ReferencePoint last_valid;
    bool has_last_valid = false;
    for (size_t i = 0; i < max_steps; ++i) {
      if (i < sdc_track->states.size() && sdc_track->states[i].valid) {
        const auto& st = sdc_track->states[i];
        hyw_sim::ReferencePoint rp;
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
  }
  hyw_sim::PlannerInputs planner_inputs;
  planner_inputs.goal = scenario.goal_pose;
  planner_inputs.desired_speed_mps = args.desired_speed;
  planner_inputs.reference_points = std::move(reference_points);
  planner_inputs.ego_vehicle = params;
  auto planner = hyw_sim::CreatePlanner(args.planner, planner_inputs, &err);
  if (!planner) {
    std::cerr << "[sim_cpp] failed to create planner: " << err << "\n";
    const auto names = hyw_sim::AvailablePlannerNames();
    std::cerr << "[sim_cpp] available planners:";
    for (const auto& name : names) std::cerr << " " << name;
    std::cerr << "\n";
    return 2;
  }
  std::cout << "[sim_cpp] planner=" << planner->Name() << "\n";

  hyw_sim::SimFileLogger sim_logger;
  if (!args.log_dir.empty()) {
    const auto ll = hyw_sim::ParseSimLogLevel(args.log_level);
    if (ll != hyw_sim::SimLogLevel::kOff) {
      if (!sim_logger.Open(fs::path(args.log_dir), ll)) {
        std::cerr << "[sim_cpp] failed to open json log under --log-dir\n";
        return 7;
      }
      std::ostringstream sj;
      sj << "{\"scenario_dir\":\""
         << hyw_sim::SimFileLogger::EscapeJsonString(args.scenario_dir)
         << "\",\"planner\":\""
         << hyw_sim::SimFileLogger::EscapeJsonString(planner->Name()) << "\"}";
      sim_logger.Log(hyw_sim::SimLogLevel::kInfo, "run_start", sj.str());
    }
  }

  hyw_sim::WorldConfig cfg;
  cfg.dt = args.dt;
  cfg.max_seconds = args.max_seconds;
  cfg.stop_on_collision = args.stop_on_collision;

  hyw_sim::WorldSimulator world(scenario, params);

  hyw_sim::StreamPipeWriter stream_writer;
  const bool enable_online =
      !args.grading_bin.empty() &&
      (args.cpp_mode == "online" || args.cpp_mode == "both");
  std::string report_path = args.grading_report;
  if (report_path.empty()) {
    const fs::path outp(args.output);
    const fs::path log_dir = outp.parent_path();
    if (log_dir.filename() == "log") {
      report_path =
          (log_dir.parent_path() / "report" / "grading_report.json").string();
    } else {
      report_path = (outp.parent_path() / "grading_report.json").string();
    }
  }
  std::error_code mk_ec;
  fs::create_directories(fs::path(report_path).parent_path(), mk_ec);
  if (enable_online) {
    if (!stream_writer.Start(args.grading_bin, report_path, args.metrics_config,
                             &err)) {
      std::cerr << "[sim_cpp] failed to start grading stream: " << err << "\n";
      return 3;
    }
  }

  const std::function<void(const hyw_sim::FrameRecord&)> stream_hook =
      [&](const hyw_sim::FrameRecord& fr) { stream_writer.EnqueueFrame(fr); };

  const auto records = world.Run(
      *planner, cfg,
      enable_online ? &stream_hook : nullptr);
  if (enable_online) {
    if (!stream_writer.Finish(&err)) {
      std::cerr << "[sim_cpp] grading stream finish failed: " << err << "\n";
      return 4;
    }
  }
  stream_writer.Close();

  if (sim_logger.IsOpen()) {
    for (const auto& fr : records) {
      sim_logger.LogFrame(hyw_sim::SimLogLevel::kDebug, fr);
    }
    std::ostringstream se;
    se << "{\"frames\":" << records.size() << ",\"output\":\""
       << hyw_sim::SimFileLogger::EscapeJsonString(args.output) << "\"}";
    sim_logger.Log(hyw_sim::SimLogLevel::kInfo, "simulation_done", se.str());
  }

  if (!hyw_sim::WriteSimLogJson(args.output, args.source_tag, records, &err)) {
    std::cerr << "[sim_cpp] failed writing simlog: " << err << "\n";
    return 5;
  }
  std::cout << "[sim_cpp] wrote " << args.output << " (" << records.size()
            << " frames)\n";

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
