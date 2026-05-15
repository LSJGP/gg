#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "cpp/types.h"

namespace hyw_sim {

/// Background thread writes NDJSON lines to `grading_main --stream` stdin so the
/// simulation loop is not blocked by pipe backpressure or grading speed.
class StreamPipeWriter {
 public:
  StreamPipeWriter() = default;
  ~StreamPipeWriter();
  StreamPipeWriter(const StreamPipeWriter&) = delete;
  StreamPipeWriter& operator=(const StreamPipeWriter&) = delete;

  bool Start(const std::string& grading_bin, const std::string& report_path,
             const std::string& metrics_config_path, std::string* error);
  /// Non-blocking except for a short mutex hold; copies `frame` into a queue.
  void EnqueueFrame(const FrameRecord& frame);
  /// Waits until all enqueued frames are written, closes the pipe, joins the writer.
  /// Call exactly once after the producer stops (e.g. after `WorldSimulator::Run`).
  bool Finish(std::string* error);
  void Close();

 private:
  void WriterLoop();

  FILE* pipe_ = nullptr;
  std::thread writer_;
  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<FrameRecord> queue_;
  bool producer_done_ = false;
  std::atomic<bool> write_failed_{false};
  bool finish_called_ = false;
};

bool WriteSimLogJson(const std::string& output_path, const std::string& source_tag,
                     const std::vector<FrameRecord>& frames, std::string* error);
bool RunBatchGrading(const std::string& grading_bin, const std::string& simlog_path,
                     const std::string& report_path,
                     const std::string& metrics_config_path, std::string* error);

}  // namespace hyw_sim
