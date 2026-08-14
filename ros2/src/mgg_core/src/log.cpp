#include "mgg_core/log.h"

#include <cstdio>

namespace mgg {
namespace {

LogSink& sink() {
  static LogSink s;
  return s;
}

const char* levelName(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug: return "DEBUG";
    case LogLevel::kInfo: return "INFO";
    case LogLevel::kWarn: return "WARN";
    case LogLevel::kError: return "ERROR";
  }
  return "?";
}

}  // namespace

void setLogSink(LogSink s) { sink() = std::move(s); }

void log(LogLevel level, const std::string& message) {
  if (sink()) {
    sink()(level, message);
    return;
  }
  // Default: surface problems, stay quiet otherwise. A library that printed
  // every info line would be unusable inside a planning loop.
  if (level == LogLevel::kWarn || level == LogLevel::kError) {
    std::fprintf(stderr, "[mgg:%s] %s\n", levelName(level), message.c_str());
  }
}

}  // namespace mgg
