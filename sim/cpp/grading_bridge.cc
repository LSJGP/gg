#include "cpp/grading_bridge.h"

#include <filesystem>
#include <fstream>

#include "cpp/grading_convert.h"
#include "google/protobuf/util/json_util.h"
#include "proto/grading/sim_log.pb.h"

namespace hyw_sim {
namespace fs = std::filesystem;
namespace {

std::string ShellSingleQuote(const std::string& p) {
  std::string out;
  out.reserve(p.size() + 8);
  out.push_back('\'');
  for (char c : p) {
    if (c == '\'') {
      out += "'\\''";
    } else {
      out.push_back(c);
    }
  }
  out.push_back('\'');
  return out;
}

}  // namespace

std::string FrameToGradingJsonLine(const proto::FrameRecord& frame,
                                   const proto::StaticMap* scene_map,
                                   const proto::VehicleParams& ego_params) {
  const proto::StaticMap* map_ptr =
      (frame.frame_id() == 0) ? scene_map : nullptr;
  const auto input = ToMetricFrameInput(frame, map_ptr, ego_params);
  std::string json;
  google::protobuf::util::JsonPrintOptions opts;
  opts.preserve_proto_field_names = true;
  const auto st = google::protobuf::util::MessageToJsonString(input, &json, opts);
  if (!st.ok()) {
    return "{}";
  }
  return json;
}

StreamPipeWriter::~StreamPipeWriter() { Close(); }

void StreamPipeWriter::WriterLoop() {
  while (true) {
    proto::FrameRecord frame;
    {
      std::unique_lock<std::mutex> lk(mu_);
      cv_.wait(lk, [&] { return !queue_.empty() || producer_done_; });
      if (queue_.empty() && producer_done_) {
        break;
      }
      if (queue_.empty()) {
        continue;
      }
      frame = std::move(queue_.front());
      queue_.pop_front();
      lk.unlock();
    }
    if (!pipe_) {
      write_failed_ = true;
      break;
    }
    const std::string line = FrameToGradingJsonLine(frame, scene_map_, ego_params_);
    if (std::fputs((line + "\n").c_str(), pipe_) < 0) {
      write_failed_ = true;
      break;
    }
    std::fflush(pipe_);
  }
}

bool StreamPipeWriter::Start(const std::string& grading_bin,
                             const std::string& report_path,
                             const std::string& metrics_config_path,
                             const proto::StaticMap* scene_map,
                             const proto::VehicleParams& ego_params,
                             std::string* error) {
  scene_map_ = scene_map;
  ego_params_ = ego_params;
  std::string cmd = grading_bin + " --stream";
  if (!metrics_config_path.empty()) {
    cmd += " --metrics-config " + ShellSingleQuote(metrics_config_path);
  }
  if (!report_path.empty()) {
    cmd += " " + ShellSingleQuote(report_path);
  }
  cmd += " 2>/dev/null";
  pipe_ = popen(cmd.c_str(), "w");
  if (!pipe_) {
    if (error) *error = "failed to start grading_main --stream";
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    producer_done_ = false;
    finish_called_ = false;
    write_failed_ = false;
    queue_.clear();
  }
  writer_ = std::thread(&StreamPipeWriter::WriterLoop, this);
  return true;
}

void StreamPipeWriter::EnqueueFrame(const proto::FrameRecord& frame) {
  std::lock_guard<std::mutex> lk(mu_);
  if (producer_done_ || finish_called_ || !pipe_) {
    return;
  }
  queue_.push_back(frame);
  cv_.notify_one();
}

bool StreamPipeWriter::Finish(std::string* error) {
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (finish_called_) {
      return !write_failed_;
    }
    finish_called_ = true;
    producer_done_ = true;
  }
  cv_.notify_all();
  if (writer_.joinable()) {
    writer_.join();
  }
  if (pipe_) {
    pclose(pipe_);
    pipe_ = nullptr;
  }
  if (write_failed_) {
    if (error) *error = "failed writing frame to grading stream";
    return false;
  }
  return true;
}

void StreamPipeWriter::Close() {
  std::string dummy;
  Finish(&dummy);
}

bool WriteSimLogJson(const std::string& output_path, const std::string& source_tag,
                     const std::vector<proto::FrameRecord>& frames,
                     const proto::StaticMap& scene_map,
                     const proto::VehicleParams& ego_params, std::string* error) {
  fs::create_directories(fs::path(output_path).parent_path());
  grading_mini::proto::SimLog log;
  log.set_source(source_tag);
  for (const auto& frame : frames) {
    const proto::StaticMap* map_ptr = (frame.frame_id() == 0) ? &scene_map : nullptr;
    *log.add_frames() = ToMetricFrameInput(frame, map_ptr, ego_params);
  }
  std::string json;
  google::protobuf::util::JsonPrintOptions opts;
  opts.preserve_proto_field_names = true;
  const auto st =
      google::protobuf::util::MessageToJsonString(log, &json, opts);
  if (!st.ok()) {
    if (error) *error = std::string(st.message());
    return false;
  }
  std::ofstream out(output_path);
  if (!out.is_open()) {
    if (error) *error = "failed to open output simlog json";
    return false;
  }
  out << json;
  return true;
}

bool RunBatchGrading(const std::string& grading_bin, const std::string& simlog_path,
                     const std::string& report_path,
                     const std::string& metrics_config_path, std::string* error) {
  std::string cmd = grading_bin;
  if (!metrics_config_path.empty()) {
    cmd += " --metrics-config " + ShellSingleQuote(metrics_config_path);
  }
  cmd += " " + ShellSingleQuote(simlog_path) + " " + ShellSingleQuote(report_path);
  const int rc = std::system(cmd.c_str());
  if (rc != 0) {
    if (error) *error = "grading_main batch mode failed";
    return false;
  }
  return true;
}

}  // namespace hyw_sim
