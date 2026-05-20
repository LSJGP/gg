#include "cpp/sim_logger.h"

#include <chrono>
#include <cctype>
#include <cstdio>
#include <iomanip>
#include <sstream>

#include "google/protobuf/util/json_util.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"

namespace hyw_sim {
namespace {

std::string LocalTimestampForFilename() {
  const auto t =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm_buf{};
#if defined(_WIN32)
  localtime_s(&tm_buf, &t);
#else
  localtime_r(&t, &tm_buf);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
  return oss.str();
}

spdlog::level::level_enum ToSpdLevel(SimLogLevel l) {
  switch (l) {
    case SimLogLevel::kTrace:
      return spdlog::level::trace;
    case SimLogLevel::kDebug:
      return spdlog::level::debug;
    case SimLogLevel::kInfo:
      return spdlog::level::info;
    case SimLogLevel::kWarn:
      return spdlog::level::warn;
    case SimLogLevel::kError:
      return spdlog::level::err;
    case SimLogLevel::kOff:
      return spdlog::level::off;
  }
  return spdlog::level::info;
}

}  // namespace

SimLogLevel ParseSimLogLevel(std::string_view s) {
  std::string t;
  t.reserve(s.size());
  for (unsigned char c : s) {
    t.push_back(static_cast<char>(std::tolower(c)));
  }
  if (t.empty() || t == "off") return SimLogLevel::kOff;
  if (t == "trace") return SimLogLevel::kTrace;
  if (t == "debug") return SimLogLevel::kDebug;
  if (t == "info") return SimLogLevel::kInfo;
  if (t == "warn" || t == "warning") return SimLogLevel::kWarn;
  if (t == "error") return SimLogLevel::kError;
  return SimLogLevel::kInfo;
}

int SimFileLogger::LevelRank(SimLogLevel l) {
  switch (l) {
    case SimLogLevel::kTrace:
      return 0;
    case SimLogLevel::kDebug:
      return 1;
    case SimLogLevel::kInfo:
      return 2;
    case SimLogLevel::kWarn:
      return 3;
    case SimLogLevel::kError:
      return 4;
    case SimLogLevel::kOff:
      return 99;
  }
  return 99;
}

bool SimFileLogger::ShouldEmit(SimLogLevel l) const {
  if (!open_ || !logger_ || l == SimLogLevel::kOff) return false;
  return LevelRank(l) >= LevelRank(min_level_);
}

std::string SimFileLogger::EscapeJsonString(std::string_view s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
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
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

bool SimFileLogger::Open(const std::filesystem::path& log_dir,
                         SimLogLevel min_level) {
  min_level_ = min_level;
  if (min_level == SimLogLevel::kOff) {
    return false;
  }
  std::error_code ec;
  std::filesystem::create_directories(log_dir, ec);
  if (ec) {
    return false;
  }
  const auto fname = std::string("sim_") + LocalTimestampForFilename() + ".log";
  const auto path = log_dir / fname;
  try {
    auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path.string(),
                                                                     /*truncate=*/true);
    logger_ = std::make_shared<spdlog::logger>("hyw_sim_runner", sink);
    logger_->set_level(ToSpdLevel(min_level));
    logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    // Crash-safe: flush often so tail is visible if the process dies mid-run.
    logger_->flush_on(spdlog::level::trace);
    logger_->flush();
  } catch (const std::exception&) {
    logger_.reset();
    return false;
  }
  open_ = true;
  return true;
}

void SimFileLogger::Log(SimLogLevel level, std::string_view message,
                        std::string_view json_data_object) {
  if (!ShouldEmit(level)) return;
  const std::string line =
      std::string(message) + " data=" + std::string(json_data_object);
  switch (level) {
    case SimLogLevel::kTrace:
      logger_->trace("{}", line);
      break;
    case SimLogLevel::kDebug:
      logger_->debug("{}", line);
      break;
    case SimLogLevel::kInfo:
      logger_->info("{}", line);
      break;
    case SimLogLevel::kWarn:
      logger_->warn("{}", line);
      break;
    case SimLogLevel::kError:
      logger_->error("{}", line);
      break;
    case SimLogLevel::kOff:
      break;
  }
}

void SimFileLogger::LogProto(SimLogLevel level, std::string_view tag,
                             const google::protobuf::Message& msg) {
  if (!ShouldEmit(level)) return;
  std::string json;
  google::protobuf::util::JsonPrintOptions opts;
  opts.preserve_proto_field_names = true;
  const auto st = google::protobuf::util::MessageToJsonString(msg, &json, opts);
  if (!st.ok()) {
    json = "{}";
  }
  Log(level, tag, json);
}

SimFileLogger::~SimFileLogger() {
  if (logger_) {
    logger_->flush();
    logger_.reset();
  }
  open_ = false;
}

}  // namespace hyw_sim
