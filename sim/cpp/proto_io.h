#pragma once

#include <string>

#include "cpp/input_format.h"
#include "google/protobuf/message.h"
#include "proto/sim/map.pb.h"
#include "proto/sim/scenario.pb.h"

namespace hyw_sim {

bool ReadJsonFileToMessage(const std::string& path,
                           google::protobuf::Message* out, std::string* error);

bool ReadBinaryFileToMessage(const std::string& path,
                             google::protobuf::Message* out, std::string* error);

bool ReadMessageFromFile(const std::string& path, google::protobuf::Message* out,
                         ScenarioInputFormat fmt, std::string* error);

bool ReadScenarioMetaFromFile(const std::string& path, proto::ScenarioMeta* out,
                              std::string* error);

bool ReadDynamicObjectsFromFile(const std::string& path,
                                proto::DynamicObjects* out, std::string* error);

bool ReadStaticMapFromFile(const std::string& path, proto::StaticMap* out,
                           std::string* error);

bool ReadStreamHeaderFromFile(const std::string& path,
                              proto::StreamDynamicHeader* out, std::string* error);

bool ReadDynamicFrameFromFile(const std::string& path, proto::DynamicFrame* out,
                              std::string* error);

}  // namespace hyw_sim
