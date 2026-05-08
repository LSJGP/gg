#pragma once

#include <deque>

namespace grading_mini {

template <typename T>
class Payload {
 public:
  Payload() = default;
  explicit Payload(int queue_size) : queue_size_(queue_size) {}

  size_t size() const { return data_.size(); }
  const T& back() const { return data_.back(); }
  void InsertData(const T& data) { data_.emplace_back(data); }
  const std::deque<T>& data() const { return data_; }

  void MaintainOnce() {
    if (queue_size_ >= 0 && static_cast<int>(data_.size()) > queue_size_)
      data_.pop_front();
  }

 private:
  std::deque<T> data_;
  int queue_size_ = -1;
};

}  // namespace grading_mini
