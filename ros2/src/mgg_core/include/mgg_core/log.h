// Minimal logging seam for the ROS-free core.
//
// The ROS 1 code called ROS_WARN_COND directly from the algorithm, which is
// one of the things that tied planner_common to ROS. The core still needs to
// report "that vertex is isolated" or "the shortest-path report is invalid",
// so it emits through a sink the host installs: mgg_ros routes it to
// RCLCPP_*, tests capture it, and a bare binary gets stderr.

#ifndef MGG_CORE_LOG_H_
#define MGG_CORE_LOG_H_

#include <functional>
#include <string>

namespace mgg {

enum class LogLevel { kDebug = 0, kInfo, kWarn, kError };

using LogSink = std::function<void(LogLevel, const std::string&)>;

/// Installs a sink. Passing nullptr restores the default (stderr for warnings
/// and errors, silence otherwise).
void setLogSink(LogSink sink);

void log(LogLevel level, const std::string& message);

inline void logWarn(const std::string& m) { log(LogLevel::kWarn, m); }
inline void logError(const std::string& m) { log(LogLevel::kError, m); }
inline void logInfo(const std::string& m) { log(LogLevel::kInfo, m); }

}  // namespace mgg

#endif  // MGG_CORE_LOG_H_
