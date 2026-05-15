#pragma once

#include <string>

#include "google/protobuf/struct.pb.h"

namespace hyw_sim {

bool ParseJsonToStruct(const std::string& text, google::protobuf::Struct* out,
                       std::string* error);
bool ReadJsonFileToStruct(const std::string& path, google::protobuf::Struct* out,
                          std::string* error);

double GetNumber(const google::protobuf::Value& value, double default_value);
double GetFieldNumber(const google::protobuf::Struct& st, const std::string& key,
                      double default_value);
std::string GetFieldString(const google::protobuf::Struct& st,
                           const std::string& key,
                           const std::string& default_value);
bool GetFieldBool(const google::protobuf::Struct& st, const std::string& key,
                  bool default_value);
const google::protobuf::Struct* GetFieldStruct(const google::protobuf::Struct& st,
                                               const std::string& key);
const google::protobuf::ListValue* GetFieldList(
    const google::protobuf::Struct& st, const std::string& key);

}  // namespace hyw_sim
