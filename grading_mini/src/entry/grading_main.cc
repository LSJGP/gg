#include <cctype>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/message.h"
#include "google/protobuf/util/json_util.h"
#include "spdlog/spdlog.h"

#include "proto/grading/grading_run_config.pb.h"
#include "proto/grading/metric_input.pb.h"
#include "proto/grading/metric_output.pb.h"
#include "proto/grading/metrics/example_metric.pb.h"
#include "proto/grading/metrics/safety_metric.pb.h"
#include "proto/grading/sim_log.pb.h"
#include "src/grading/grader.h"
#include "src/planning/simple_planner.h"

namespace {

struct FrameData {
  int64_t frame_id;
  int64_t timestamp_us;
  double x, y, heading, speed, acceleration;
};

double ParseDouble(const std::string& s, const std::string& key) {
  auto pos = s.find("\"" + key + "\"");
  if (pos == std::string::npos) return 0.0;
  pos = s.find(':', pos);
  auto start = s.find_first_of("-0123456789.", pos);
  auto end = s.find_first_not_of("-0123456789.eE+", start);
  return std::stod(s.substr(start, end - start));
}

std::vector<FrameData> ParseLegacyFrames(const std::string& content) {
  std::vector<FrameData> frames;
  size_t pos = 0;
  while ((pos = content.find('{', pos)) != std::string::npos) {
    auto end = content.find('}', pos);
    if (end == std::string::npos) break;
    std::string obj = content.substr(pos, end - pos + 1);

    FrameData fd{};
    fd.frame_id = static_cast<int64_t>(ParseDouble(obj, "frame_id"));
    fd.timestamp_us = static_cast<int64_t>(ParseDouble(obj, "timestamp_us"));
    fd.x = ParseDouble(obj, "x");
    fd.y = ParseDouble(obj, "y");
    fd.heading = ParseDouble(obj, "heading");
    fd.speed = ParseDouble(obj, "speed");
    fd.acceleration = ParseDouble(obj, "acceleration");
    frames.push_back(fd);
    pos = end + 1;
  }
  return frames;
}

grading_mini::proto::MetricFrameInput ToProto(const FrameData& fd) {
  grading_mini::proto::MetricFrameInput input;
  input.set_frame_id(fd.frame_id);
  input.set_timestamp_us(fd.timestamp_us);
  auto* vs = input.mutable_vehicle_state();
  vs->set_x(fd.x);
  vs->set_y(fd.y);
  vs->set_heading(fd.heading);
  vs->set_speed(fd.speed);
  vs->set_acceleration(fd.acceleration);
  return input;
}

std::string ReadWholeFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) return {};
  std::stringstream buf;
  buf << f.rdbuf();
  return buf.str();
}

bool EndsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string TrimCopy(std::string s) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  while (!s.empty() && !not_space(static_cast<unsigned char>(s.front())))
    s.erase(0, 1);
  while (!s.empty() && !not_space(static_cast<unsigned char>(s.back())))
    s.pop_back();
  return s;
}

bool LoadMetricInputs(const std::string& path,
                      std::vector<grading_mini::proto::MetricFrameInput>* out,
                      std::string* source) {
  out->clear();
  source->clear();

  if (EndsWith(path, ".pb")) {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) {
      SPDLOG_ERROR("Cannot open: {}", path);
      return false;
    }
    grading_mini::proto::SimLog log;
    if (!log.ParseFromIstream(&f) || log.frames_size() == 0) {
      SPDLOG_ERROR("Invalid or empty SimLog binary: {}", path);
      return false;
    }
    *source = log.source();
    out->assign(log.frames().begin(), log.frames().end());
    return true;
  }

  const std::string raw = ReadWholeFile(path);
  if (raw.empty()) {
    SPDLOG_ERROR("Cannot read or empty file: {}", path);
    return false;
  }
  const std::string content = TrimCopy(raw);
  if (content.empty()) {
    SPDLOG_ERROR("Empty content: {}", path);
    return false;
  }

  if (content[0] == '[') {
    const auto fds = ParseLegacyFrames(content);
    if (fds.empty()) {
      SPDLOG_ERROR("Legacy JSON array: no frames parsed from {}", path);
      return false;
    }
    out->reserve(fds.size());
    for (const auto& fd : fds) out->push_back(ToProto(fd));
    *source = "legacy_json_array";
    return true;
  }

  grading_mini::proto::SimLog log;
  google::protobuf::util::JsonParseOptions jopts;
  jopts.ignore_unknown_fields = true;
  const auto jst =
      google::protobuf::util::JsonStringToMessage(content, &log, jopts);
  if (!jst.ok() || log.frames_size() == 0) {
    SPDLOG_ERROR("SimLog JSON parse failed or no frames: {} — {}", path,
                 std::string(jst.message()));
    return false;
  }
  *source = log.source();
  out->assign(log.frames().begin(), log.frames().end());
  return true;
}

bool WriteReport(const grading_mini::proto::GradingReport& report,
                 const std::string& path) {
  std::string json;
  google::protobuf::util::JsonPrintOptions opts;
  opts.always_print_primitive_fields = true;
  opts.add_whitespace = true;
  auto s = google::protobuf::util::MessageToJsonString(report, &json, opts);
  if (!s.ok()) return false;
  std::ofstream f(path);
  if (!f.is_open()) return false;
  f << json;
  return true;
}

void ApplySpdlogLevel(const std::string& raw) {
  if (raw.empty()) return;
  std::string level = raw;
  for (auto& c : level) {
    c = static_cast<char>(::tolower(static_cast<unsigned char>(c)));
  }
  if (level == "off") {
    spdlog::set_level(spdlog::level::off);
    return;
  }
  spdlog::level::level_enum e = spdlog::level::info;
  if (level == "trace") e = spdlog::level::trace;
  else if (level == "debug") e = spdlog::level::debug;
  else if (level == "info") e = spdlog::level::info;
  else if (level == "warn" || level == "warning") e = spdlog::level::warn;
  else if (level == "error") e = spdlog::level::err;
  else if (level == "critical") e = spdlog::level::critical;
  spdlog::set_level(e);
}

grading_mini::proto::GradingRunConfig DefaultRunConfig() {
  grading_mini::proto::GradingRunConfig c;
  c.set_simple_planner_max_speed_mps(33.3);
  c.add_metrics()->set_name("planning_limit_checker");
  c.add_metrics()->set_name("speed_checker");
  c.add_metrics()->set_name("regulatory_collision_checker");
  return c;
}

bool LoadGradingRunConfigJson(const std::string& path,
                             grading_mini::proto::GradingRunConfig* out,
                             std::string* err) {
  const std::string raw = ReadWholeFile(path);
  if (raw.empty()) {
    if (err) *err = "cannot read metrics config: " + path;
    return false;
  }
  const std::string content = TrimCopy(raw);
  google::protobuf::util::JsonParseOptions jopts;
  jopts.ignore_unknown_fields = true;
  const auto st =
      google::protobuf::util::JsonStringToMessage(content, out, jopts);
  if (!st.ok()) {
    if (err) *err = std::string(st.message());
    return false;
  }
  return true;
}

absl::Status BuildMetricInitSpecs(
    const grading_mini::proto::GradingRunConfig& cfg,
    std::vector<grading_mini::MetricInitSpec>* specs,
    std::vector<std::unique_ptr<google::protobuf::Message>>* owned) {
  specs->clear();
  owned->clear();
  if (cfg.metrics().empty()) {
    return absl::InvalidArgumentError("GradingRunConfig.metrics is empty");
  }
  google::protobuf::util::JsonParseOptions jopts;
  jopts.ignore_unknown_fields = true;
  for (const auto& m : cfg.metrics()) {
    const std::string& n = m.name();
    if (n.empty()) {
      return absl::InvalidArgumentError("metric name is empty");
    }
    if (n == "speed_checker") {
      auto pb = std::make_unique<grading_mini::proto::SpeedChecker>();
      if (!m.params_json().empty()) {
        const auto pst = google::protobuf::util::JsonStringToMessage(
            m.params_json(), pb.get(), jopts);
        if (!pst.ok()) {
          return absl::InvalidArgumentError(absl::StrCat(
              "speed_checker params_json: ", std::string(pst.message())));
        }
      }
      owned->push_back(std::move(pb));
      specs->push_back({n, owned->back().get()});
    } else if (n == "planning_limit_checker") {
      auto pb = std::make_unique<grading_mini::proto::PlanningLimitCheckerConfig>();
      if (!m.params_json().empty()) {
        const auto pst = google::protobuf::util::JsonStringToMessage(
            m.params_json(), pb.get(), jopts);
        if (!pst.ok()) {
          return absl::InvalidArgumentError(absl::StrCat(
              "planning_limit_checker params_json: ", std::string(pst.message())));
        }
      }
      owned->push_back(std::move(pb));
      specs->push_back({n, owned->back().get()});
    } else if (n == "regulatory_collision_checker") {
      if (!m.params_json().empty()) {
        SPDLOG_WARN(
            "regulatory_collision_checker: params_json is ignored for now");
      }
      specs->push_back({n, nullptr});
    } else if (n == "lane_departure_checker") {
      auto pb = std::make_unique<grading_mini::proto::LaneDepartureCheckerConfig>();
      if (!m.params_json().empty()) {
        const auto pst = google::protobuf::util::JsonStringToMessage(
            m.params_json(), pb.get(), jopts);
        if (!pst.ok()) {
          return absl::InvalidArgumentError(absl::StrCat(
              "lane_departure_checker params_json: ", std::string(pst.message())));
        }
      }
      owned->push_back(std::move(pb));
      specs->push_back({n, owned->back().get()});
    } else if (n == "drivable_area_checker") {
      auto pb = std::make_unique<grading_mini::proto::DrivableAreaCheckerConfig>();
      if (!m.params_json().empty()) {
        const auto pst = google::protobuf::util::JsonStringToMessage(
            m.params_json(), pb.get(), jopts);
        if (!pst.ok()) {
          return absl::InvalidArgumentError(absl::StrCat(
              "drivable_area_checker params_json: ", std::string(pst.message())));
        }
      }
      owned->push_back(std::move(pb));
      specs->push_back({n, owned->back().get()});
    } else {
      return absl::InvalidArgumentError(absl::StrCat(
          "Unknown metric name: ", n,
          " (supported: planning_limit_checker, speed_checker, "
          "regulatory_collision_checker, lane_departure_checker, "
          "drivable_area_checker)"));
    }
  }
  return absl::OkStatus();
}

void PrintUsage(const char* argv0) {
  std::cerr
      << "Usage:\n"
      << "  " << argv0
      << " [--metrics-config <grading_metrics.json>] <input.pb|input.json> "
         "[output.json]\n"
      << "      Batch mode. Reads the whole SimLog, grades it, writes report.\n"
      << "  " << argv0
      << " --stream [--metrics-config <grading_metrics.json>] [output.json]\n"
      << "      Online mode. One MetricFrameInput JSON per line on stdin.\n"
      << "      Without --metrics-config, uses built-in default metrics.\n";
}

bool RunBatch(const std::string& input_path, const std::string& output_path,
              const grading_mini::proto::GradingRunConfig& run_cfg) {
  std::string sim_source;
  std::vector<grading_mini::proto::MetricFrameInput> inputs;
  if (!LoadMetricInputs(input_path, &inputs, &sim_source)) {
    SPDLOG_ERROR("Failed to load inputs");
    return false;
  }
  if (!sim_source.empty()) {
    SPDLOG_INFO("Loaded {} frames (source={})", inputs.size(), sim_source);
  } else {
    SPDLOG_INFO("Loaded {} frames", inputs.size());
  }

  std::vector<std::unique_ptr<google::protobuf::Message>> owned_configs;
  std::vector<grading_mini::MetricInitSpec> specs;
  const auto bst = BuildMetricInitSpecs(run_cfg, &specs, &owned_configs);
  if (!bst.ok()) {
    SPDLOG_ERROR("metric config: {}", std::string(bst.message()));
    return false;
  }

  const double planner_max = run_cfg.simple_planner_max_speed_mps() > 0.0
                                 ? run_cfg.simple_planner_max_speed_mps()
                                 : 33.3;
  grading_mini::SimplePlanner planner(planner_max);
  grading_mini::Grader grader;
  auto status = grader.Init(specs);
  if (!status.ok()) {
    SPDLOG_ERROR("Init failed: {}", std::string(status.message()));
    return false;
  }

  for (const auto& frame : inputs) {
    auto input = frame;
    planner.Plan(&input);
    status = grader.ProcessFrame(input);
    if (!status.ok()) {
      SPDLOG_ERROR("Frame {} failed: {}", input.frame_id(),
                   std::string(status.message()));
    }
  }

  auto report_or = grader.Finish();
  if (!report_or.ok()) {
    SPDLOG_ERROR("Report failed: {}",
                 std::string(report_or.status().message()));
    return false;
  }

  if (WriteReport(report_or.value(), output_path)) {
    SPDLOG_INFO("Report written to: {}", output_path);
  }

  std::cout << "=== Result: "
            << (report_or.value().overall_passed() ? "PASSED" : "FAILED")
            << " ===" << std::endl;
  for (const auto& s : report_or.value().summaries()) {
    std::cout << "  " << s.metric_name() << ": "
              << (s.passed() ? "PASS" : "FAIL") << " (" << s.detail() << ")"
              << std::endl;
  }
  return true;
}

bool RunStream(const std::string& output_path,
               const grading_mini::proto::GradingRunConfig& run_cfg) {
  // Line-buffer stdout so the producer side can reliably read tick lines as
  // soon as we emit them (subprocess piping uses block-buffering by default).
  std::setvbuf(stdout, nullptr, _IOLBF, 0);

  std::vector<std::unique_ptr<google::protobuf::Message>> owned_configs;
  std::vector<grading_mini::MetricInitSpec> specs;
  const auto bst = BuildMetricInitSpecs(run_cfg, &specs, &owned_configs);
  if (!bst.ok()) {
    SPDLOG_ERROR("metric config: {}", std::string(bst.message()));
    return false;
  }

  const double planner_max = run_cfg.simple_planner_max_speed_mps() > 0.0
                                 ? run_cfg.simple_planner_max_speed_mps()
                                 : 33.3;
  grading_mini::SimplePlanner planner(planner_max);
  grading_mini::Grader grader;
  auto status = grader.Init(specs);
  if (!status.ok()) {
    SPDLOG_ERROR("Init failed: {}", std::string(status.message()));
    return false;
  }

  google::protobuf::util::JsonParseOptions jopts;
  jopts.ignore_unknown_fields = true;

  std::cout << "[cpp] stream ready" << std::endl;

  std::string line;
  int64_t processed = 0;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    grading_mini::proto::MetricFrameInput input;
    auto pst = google::protobuf::util::JsonStringToMessage(line, &input, jopts);
    if (!pst.ok()) {
      SPDLOG_ERROR("Frame {} parse failed: {}", processed,
                   std::string(pst.message()));
      continue;
    }
    planner.Plan(&input);
    auto rs = grader.ProcessFrame(input);
    if (!rs.ok()) {
      SPDLOG_ERROR("Frame {} grade failed: {}", input.frame_id(),
                   std::string(rs.message()));
      continue;
    }
    auto verdicts = grader.LastFrameVerdicts();
    bool all = true;
    std::ostringstream parts;
    for (size_t i = 0; i < verdicts.size(); ++i) {
      if (i) parts << ' ';
      parts << verdicts[i].first << '=' << (verdicts[i].second ? "P" : "F");
      if (!verdicts[i].second) all = false;
    }
    std::cout << "[cpp] frame=" << input.frame_id()
              << " t=" << (input.timestamp_us() / 1e6) << "s"
              << " v=" << input.vehicle_state().speed()
              << " coll=" << (input.has_collision_event() &&
                              input.collision_event().collided() ? "Y" : "n")
              << " " << (all ? "PASS" : "FAIL")
              << " [" << parts.str() << "]" << std::endl;
    ++processed;
  }

  auto report_or = grader.Finish();
  if (!report_or.ok()) {
    SPDLOG_ERROR("Report failed: {}",
                 std::string(report_or.status().message()));
    return false;
  }

  if (WriteReport(report_or.value(), output_path)) {
    SPDLOG_INFO("Report written to: {}", output_path);
  }

  std::cout << "=== Result: "
            << (report_or.value().overall_passed() ? "PASSED" : "FAILED")
            << " === (" << processed << " frames)" << std::endl;
  for (const auto& s : report_or.value().summaries()) {
    std::cout << "  " << s.metric_name() << ": "
              << (s.passed() ? "PASS" : "FAIL") << " (" << s.detail() << ")"
              << std::endl;
  }
  return true;
}

struct GradingCli {
  bool stream_mode = false;
  bool want_help = false;
  std::string metrics_config;
  std::vector<std::string> positional;
};

bool ParseGradingCli(int argc, char** argv, GradingCli* cli, std::string* err) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--stream") {
      cli->stream_mode = true;
      continue;
    }
    if (a == "--metrics-config") {
      if (i + 1 >= argc) {
        if (err) *err = "--metrics-config requires a file path";
        return false;
      }
      cli->metrics_config = argv[++i];
      continue;
    }
    if (a == "-h" || a == "--help") {
      cli->want_help = true;
      continue;
    }
    if (a.rfind("--", 0) == 0) {
      if (err) *err = "unknown flag: " + a;
      return false;
    }
    cli->positional.push_back(a);
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  GradingCli cli;
  std::string perr;
  if (!ParseGradingCli(argc, argv, &cli, &perr)) {
    std::cerr << "[grading_main] " << perr << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
  if (cli.want_help) {
    PrintUsage(argv[0]);
    return 0;
  }

  grading_mini::proto::GradingRunConfig run_cfg;
  if (!cli.metrics_config.empty()) {
    std::string err;
    if (!LoadGradingRunConfigJson(cli.metrics_config, &run_cfg, &err)) {
      std::cerr << "[grading_main] " << err << "\n";
      return 1;
    }
  } else {
    run_cfg = DefaultRunConfig();
  }
  ApplySpdlogLevel(run_cfg.spdlog_level());

  if (cli.stream_mode) {
    const std::string out = cli.positional.empty()
                                ? "/tmp/grading_output.json"
                                : cli.positional[0];
    if (cli.positional.size() > 1) {
      std::cerr << "[grading_main] too many arguments for --stream mode\n";
      PrintUsage(argv[0]);
      return 1;
    }
    return RunStream(out, run_cfg) ? 0 : 1;
  }

  if (cli.positional.empty()) {
    PrintUsage(argv[0]);
    return 1;
  }
  const std::string& in = cli.positional[0];
  const std::string out = cli.positional.size() > 1 ? cli.positional[1]
                                                    : "/tmp/grading_output.json";
  if (cli.positional.size() > 2) {
    std::cerr << "[grading_main] too many positional arguments\n";
    PrintUsage(argv[0]);
    return 1;
  }
  return RunBatch(in, out, run_cfg) ? 0 : 1;
}
