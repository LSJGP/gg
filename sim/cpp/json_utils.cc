#include "cpp/json_utils.h"

#include <fstream>
#include <sstream>

#include "google/protobuf/util/json_util.h"

namespace hyw_sim {
namespace {

std::string ReadWholeFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) return {};
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

}  // namespace

bool ParseJsonToStruct(const std::string& text, google::protobuf::Struct* out,
                       std::string* error) {
  out->Clear();
  google::protobuf::util::JsonParseOptions opts;
  opts.ignore_unknown_fields = true;
  const auto s = google::protobuf::util::JsonStringToMessage(text, out, opts);
  if (!s.ok()) {
    if (error) *error = std::string(s.message());
    return false;
  }
  return true;
}

bool ReadJsonFileToStruct(const std::string& path, google::protobuf::Struct* out,
                          std::string* error) {
  const std::string text = ReadWholeFile(path);
  if (text.empty()) {
    if (error) *error = "file is empty or cannot be read";
    return false;
  }
  return ParseJsonToStruct(text, out, error);
}

double GetNumber(const google::protobuf::Value& value, double default_value) {
  if (value.kind_case() == google::protobuf::Value::kNumberValue) {
    return value.number_value();
  }
  return default_value;
}

double GetFieldNumber(const google::protobuf::Struct& st, const std::string& key,
                      double default_value) {
  auto it = st.fields().find(key);
  if (it == st.fields().end()) return default_value;
  return GetNumber(it->second, default_value);
}

std::string GetFieldString(const google::protobuf::Struct& st,
                           const std::string& key,
                           const std::string& default_value) {
  auto it = st.fields().find(key);
  if (it == st.fields().end()) return default_value;
  if (it->second.kind_case() == google::protobuf::Value::kStringValue) {
    return it->second.string_value();
  }
  return default_value;
}

bool GetFieldBool(const google::protobuf::Struct& st, const std::string& key,
                  bool default_value) {
  auto it = st.fields().find(key);
  if (it == st.fields().end()) return default_value;
  if (it->second.kind_case() == google::protobuf::Value::kBoolValue) {
    return it->second.bool_value();
  }
  return default_value;
}

const google::protobuf::Struct* GetFieldStruct(const google::protobuf::Struct& st,
                                               const std::string& key) {
  auto it = st.fields().find(key);
  if (it == st.fields().end()) return nullptr;
  if (it->second.kind_case() != google::protobuf::Value::kStructValue) {
    return nullptr;
  }
  return &it->second.struct_value();
}

const google::protobuf::ListValue* GetFieldList(
    const google::protobuf::Struct& st, const std::string& key) {
  auto it = st.fields().find(key);
  if (it == st.fields().end()) return nullptr;
  if (it->second.kind_case() != google::protobuf::Value::kListValue) return nullptr;
  return &it->second.list_value();
}

}  // namespace hyw_sim
