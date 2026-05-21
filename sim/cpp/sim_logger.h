#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

#include "google/protobuf/message.h"
#include "proto/sim/runtime.pb.h"

namespace spdlog {
class logger;
}

namespace hyw_sim {

enum class SimLogLevel { kTrace = 0, kDebug, kInfo, kWarn, kError, kOff };

SimLogLevel ParseSimLogLevel(std::string_view s);

class SimFileLogger {
 public:
  SimFileLogger() = default;
  SimFileLogger(const SimFileLogger&) = delete;
  SimFileLogger& operator=(const SimFileLogger&) = delete;
  ~SimFileLogger();

  bool Open(const std::filesystem::path& log_dir, SimLogLevel min_level);

  void Log(SimLogLevel level, std::string_view message,
           std::string_view json_data_object = "{}");

  void LogProto(SimLogLevel level, std::string_view tag,
                const google::protobuf::Message& msg);

  bool IsOpen() const { return open_; }

  static std::string EscapeJsonString(std::string_view s);

 private:
  static int LevelRank(SimLogLevel l);
  bool ShouldEmit(SimLogLevel l) const;

  bool open_ = false;
  SimLogLevel min_level_ = SimLogLevel::kOff;
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace hyw_sim
