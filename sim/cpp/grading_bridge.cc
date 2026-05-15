#include "cpp/grading_bridge.h"

#include <filesystem>
#include <fstream>
#include <sstream>

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

std::string EscapeJson(const std::string& in) {
  std::string out;
  out.reserve(in.size());
  for (char c : in) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      default:
        out += c;
    }
  }
  return out;
}

std::string FrameToJsonLine(const FrameRecord& f) {
  std::ostringstream ss;
  ss << "{";
  ss << "\"frame_id\":" << f.frame_id << ",";
  ss << "\"timestamp_us\":" << f.timestamp_us << ",";
  ss << "\"vehicle_state\":{"
     << "\"x\":" << f.ego.x << ","
     << "\"y\":" << f.ego.y << ","
     << "\"heading\":" << f.ego.heading << ","
     << "\"speed\":" << f.ego.speed << ","
     << "\"acceleration\":" << f.ego.acceleration << "},";
  ss << "\"planning_command\":{"
     << "\"desired_speed_mps\":" << f.command.desired_speed_mps << "}";
  if (f.collision.collided) {
    ss << ",\"collision_event\":{"
       << "\"collided\":true,"
       << "\"other_id\":" << f.collision.other_id << ","
       << "\"kind\":\"" << EscapeJson(f.collision.kind) << "\","
       << "\"ego_at_fault\":" << (f.collision.ego_at_fault ? "true" : "false")
       << ","
       << "\"exempt\":" << (f.collision.exempt ? "true" : "false") << ","
       << "\"exempt_reason\":\"" << EscapeJson(f.collision.exempt_reason) << "\","
       << "\"relative_speed_mps\":" << f.collision.relative_speed_mps << ","
       << "\"ego_speed_mps\":" << f.collision.ego_speed_mps << ","
       << "\"approach_angle_deg\":" << f.collision.approach_angle_deg << "}";
  }
  ss << "}";
  return ss.str();
}

}  // namespace

StreamPipeWriter::~StreamPipeWriter() { Close(); }

void StreamPipeWriter::WriterLoop() {
  while (true) {
    FrameRecord frame;
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
    const std::string line = FrameToJsonLine(frame) + "\n";
    if (std::fwrite(line.data(), 1, line.size(), pipe_) != line.size()) {
      write_failed_ = true;
      break;
    }
    std::fflush(pipe_);
  }
  std::lock_guard<std::mutex> lk(mu_);
  if (pipe_) {
    ::pclose(pipe_);
    pipe_ = nullptr;
  }
}

bool StreamPipeWriter::Start(const std::string& grading_bin,
                             const std::string& report_path,
                             const std::string& metrics_config_path,
                             std::string* error) {
  std::string cmd = grading_bin + " --stream " + ShellSingleQuote(report_path);
  if (!metrics_config_path.empty()) {
    cmd += " --metrics-config " + ShellSingleQuote(metrics_config_path);
  }
  pipe_ = ::popen(cmd.c_str(), "w");
  if (!pipe_) {
    if (error) *error = "failed to start grading stream process";
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

void StreamPipeWriter::EnqueueFrame(const FrameRecord& frame) {
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
                     const std::vector<FrameRecord>& frames, std::string* error) {
  fs::create_directories(fs::path(output_path).parent_path());
  std::ofstream out(output_path);
  if (!out.is_open()) {
    if (error) *error = "failed to open output simlog json";
    return false;
  }
  out << "{\"source\":\"" << EscapeJson(source_tag) << "\",\"frames\":[";
  for (size_t i = 0; i < frames.size(); ++i) {
    if (i) out << ",";
    out << FrameToJsonLine(frames[i]);
  }
  out << "]}";
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
