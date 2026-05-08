#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "google/protobuf/util/json_util.h"
#include "spdlog/spdlog.h"

#include "proto/grading/metric_input.pb.h"
#include "proto/grading/metric_output.pb.h"
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

}  // namespace

namespace {

void PrintUsage(const char* argv0) {
  std::cerr
      << "Usage:\n"
      << "  " << argv0 << " <input.pb|input.json> [output.json]\n"
      << "      Batch mode. Reads the whole SimLog (binary .pb / JSON / legacy\n"
      << "      array), grades it, writes a GradingReport JSON.\n"
      << "  " << argv0 << " --stream [output.json]\n"
      << "      Online mode. Reads one MetricFrameInput JSON per line from\n"
      << "      stdin, prints a per-frame tick to stdout, and writes the\n"
      << "      GradingReport JSON when stdin closes.\n";
}

bool RunBatch(const std::string& input_path, const std::string& output_path) {
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

  grading_mini::SimplePlanner planner(33.3);
  grading_mini::Grader grader;
  auto status = grader.Init({
      "planning_limit_checker",
      "speed_checker",
      "regulatory_collision_checker",
  });
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

bool RunStream(const std::string& output_path) {
  // Line-buffer stdout so the producer side can reliably read tick lines as
  // soon as we emit them (subprocess piping uses block-buffering by default).
  std::setvbuf(stdout, nullptr, _IOLBF, 0);

  grading_mini::SimplePlanner planner(33.3);
  grading_mini::Grader grader;
  auto status = grader.Init({
      "planning_limit_checker",
      "speed_checker",
      "regulatory_collision_checker",
  });
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

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 1;
  }

  std::string arg1 = argv[1];
  if (arg1 == "--stream") {
    std::string output_path =
        (argc > 2) ? argv[2] : "/tmp/grading_output.json";
    return RunStream(output_path) ? 0 : 1;
  }
  if (arg1 == "-h" || arg1 == "--help") {
    PrintUsage(argv[0]);
    return 0;
  }

  std::string output_path = (argc > 2) ? argv[2] : "/tmp/grading_output.json";
  return RunBatch(arg1, output_path) ? 0 : 1;
}
