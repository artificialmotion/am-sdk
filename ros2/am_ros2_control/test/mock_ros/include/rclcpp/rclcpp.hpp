// Test-only mock of the rclcpp pieces used by am_ros2_control. Not ROS.
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>

namespace rclcpp
{
class Duration
{
public:
  Duration(int32_t s, uint32_t ns)
  : ns_(static_cast<int64_t>(s) * 1000000000LL + ns) {}
  static Duration from_seconds(double s)
  {
    Duration d(0, 0);
    d.ns_ = static_cast<int64_t>(s * 1e9);
    return d;
  }
  double seconds() const {return static_cast<double>(ns_) * 1e-9;}

private:
  int64_t ns_;
};

class Time
{
public:
  Time()
  : ns_(0) {}
  explicit Time(int64_t ns)
  : ns_(ns) {}
  double seconds() const {return static_cast<double>(ns_) * 1e-9;}

private:
  int64_t ns_;
};

class Logger
{
public:
  explicit Logger(const std::string & n)
  : name_(n) {}
  const char * get_name() const {return name_.c_str();}

private:
  std::string name_;
};

inline Logger get_logger(const std::string & name) {return Logger(name);}

namespace mock
{
// Counters so tests can assert on logged errors and warnings.
inline int & error_count() {static int n = 0; return n;}
inline int & warn_count() {static int n = 0; return n;}
inline bool & quiet() {static bool q = false; return q;}
}  // namespace mock
}  // namespace rclcpp

#define AM_MOCK_LOG(level, logger, ...) \
  do { \
    if (!rclcpp::mock::quiet()) { \
      std::fprintf(stderr, "[%s] [%s] ", level, (logger).get_name()); \
      std::fprintf(stderr, __VA_ARGS__); \
      std::fprintf(stderr, "\n"); \
    } \
  } while (0)
#define RCLCPP_DEBUG(logger, ...) do {(void)(logger);} while (0)
#define RCLCPP_INFO(logger, ...) AM_MOCK_LOG("INFO", logger, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) \
  do {++rclcpp::mock::warn_count(); AM_MOCK_LOG("WARN", logger, __VA_ARGS__);} while (0)
#define RCLCPP_ERROR(logger, ...) \
  do {++rclcpp::mock::error_count(); AM_MOCK_LOG("ERROR", logger, __VA_ARGS__);} while (0)
