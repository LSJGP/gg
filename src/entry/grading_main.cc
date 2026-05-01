#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "google/protobuf/util/json_util.h"
#include "spdlog/spdlog.h"

#include "proto/grading/metric_input.pb.h"
#include "proto/grading/metric_output.pb.h"
#include "src/grading/grader.h"

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

std::vector<FrameData> LoadFrames(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    SPDLOG_ERROR("Cannot open: {}", path);
    return {};
  }
  std::stringstream buf;
  buf << f.rdbuf();
  std::string content = buf.str();

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

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <input.json> [output.json]" << std::endl;
    return 1;
  }

  std::string input_path = argv[1];
  std::string output_path = (argc > 2) ? argv[2] : "/tmp/grading_output.json";

  auto frames = LoadFrames(input_path);
  if (frames.empty()) {
    SPDLOG_ERROR("No frames loaded");
    return 1;
  }
  SPDLOG_INFO("Loaded {} frames", frames.size());

  grading_mini::Grader grader;
  auto status = grader.Init({"speed_checker"});
  if (!status.ok()) {
    SPDLOG_ERROR("Init failed: {}", std::string(status.message()));
    return 1;
  }

  for (const auto& fd : frames) {
    status = grader.ProcessFrame(ToProto(fd));
    if (!status.ok()) {
      SPDLOG_ERROR("Frame {} failed: {}", fd.frame_id,
                   std::string(status.message()));
    }
  }

  auto report_or = grader.Finish();
  if (!report_or.ok()) {
    SPDLOG_ERROR("Report failed: {}",
                 std::string(report_or.status().message()));
    return 1;
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
  return 0;
}
