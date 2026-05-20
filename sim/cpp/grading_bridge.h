#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "proto/sim/map.pb.h"
#include "proto/sim/runtime.pb.h"

namespace hyw_sim {

std::string FrameToGradingJsonLine(const proto::FrameRecord& frame,
                                   const proto::StaticMap* scene_map,
                                   const proto::VehicleParams& ego_params);

class StreamPipeWriter {
 public:
  StreamPipeWriter() = default;
  ~StreamPipeWriter();
  StreamPipeWriter(const StreamPipeWriter&) = delete;
  StreamPipeWriter& operator=(const StreamPipeWriter&) = delete;

  bool Start(const std::string& grading_bin, const std::string& report_path,
             const std::string& metrics_config_path,
             const proto::StaticMap* scene_map, const proto::VehicleParams& ego_params,
             std::string* error);
  void EnqueueFrame(const proto::FrameRecord& frame);
  bool Finish(std::string* error);
  void Close();

 private:
  void WriterLoop();

  FILE* pipe_ = nullptr;
  std::thread writer_;
  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<proto::FrameRecord> queue_;
  bool producer_done_ = false;
  std::atomic<bool> write_failed_{false};
  bool finish_called_ = false;
  const proto::StaticMap* scene_map_ = nullptr;
  proto::VehicleParams ego_params_;
};

bool WriteSimLogJson(const std::string& output_path, const std::string& source_tag,
                     const std::vector<proto::FrameRecord>& frames,
                     const proto::StaticMap& scene_map,
                     const proto::VehicleParams& ego_params, std::string* error);
bool RunBatchGrading(const std::string& grading_bin, const std::string& simlog_path,
                     const std::string& report_path,
                     const std::string& metrics_config_path, std::string* error);

}  // namespace hyw_sim
