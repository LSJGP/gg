#pragma once

#define GRADING_DECLARE_SINGLETON(ClassName)          \
 public:                                              \
  static ClassName* Instance() {                      \
    static ClassName instance;                        \
    return &instance;                                 \
  }                                                   \
                                                      \
 private:                                             \
  ClassName();                                        \
  ClassName(const ClassName&) = delete;               \
  ClassName& operator=(const ClassName&) = delete

#define RETURN_IF_ERROR(expr)                    \
  do {                                           \
    auto _status = (expr);                       \
    if (!_status.ok()) return _status;           \
  } while (0)
