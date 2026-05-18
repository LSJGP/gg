#pragma once

#include <string>

#include "google/protobuf/message.h"
#include "proto/sim/map.pb.h"
#include "proto/sim/scenario.pb.h"

namespace hyw_sim {

bool ReadJsonFileToMessage(const std::string& path,
                           google::protobuf::Message* out, std::string* error);

bool ReadScenarioMetaFromFile(const std::string& path, proto::ScenarioMeta* out,
                              std::string* error);

bool ReadDynamicObjectsFromFile(const std::string& path,
                                proto::DynamicObjects* out, std::string* error);

/// lane_graph.json uses [[x,y,z],...] polylines; convert via Struct then fill proto.
bool ReadStaticMapFromFile(const std::string& path, proto::StaticMap* out,
                           std::string* error);

}  // namespace hyw_sim
