#pragma once

#include <memory>
#include <string>

#include "src/grading/metric_factory.h"

namespace grading_mini {

template <typename T>
class MetricRegister {
 public:
  explicit MetricRegister(const std::string& name) : name_(name) {
    MetricFactory::Instance()->Register(
        name, []() { return std::make_unique<T>(); });
  }
  ~MetricRegister() { (void)MetricFactory::Instance()->Unregister(name_); }

 private:
  std::string name_;
};

}  // namespace grading_mini

#define REGISTER_METRIC(MetricClass, MetricName)                       \
  static ::grading_mini::MetricRegister<MetricClass>                   \
      __##MetricClass##_register__(MetricName)
