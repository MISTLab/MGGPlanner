#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include <nlohmann/json.hpp>

namespace mgg_ros {
namespace exploration_event_detail {
inline std::string uuid4() {
  std::random_device random;
  std::array<unsigned char, 16> bytes{};
  for (auto& byte : bytes) byte = static_cast<unsigned char>(random());
  bytes[6] = (bytes[6] & 0x0f) | 0x40;
  bytes[8] = (bytes[8] & 0x3f) | 0x80;
  constexpr char hex[] = "0123456789abcdef";
  std::string id;
  for (const auto byte : bytes) {
    id += hex[byte >> 4];
    id += hex[byte & 15];
  }
  return id;
}

inline void requireFinite(const nlohmann::json& value) {
  if (value.is_number_float() && !std::isfinite(value.get<double>()))
    throw std::invalid_argument("nonfinite exploration event value");
  if (value.is_structured())
    for (const auto& child : value) requireFinite(child);
}
}  // namespace exploration_event_detail

// Callers log the returned line with RCLCPP_INFO; no ROS dependency here.
// Canonical kind-specific payload validation lives in the Python consumer.
inline std::string explorationEventLine(const std::string& kind,
                                       const std::string& robot_id,
                                       const nlohmann::json& payload,
                                       std::optional<double> stamp_s) {
  static std::mutex mutex;
  static std::string boot = exploration_event_detail::uuid4();
  static uint64_t seq = 0;
  static pid_t pid = getpid();
  std::lock_guard<std::mutex> lock(mutex);
  if (getpid() != pid) {
    pid = getpid();
    boot = exploration_event_detail::uuid4();
    seq = 0;
  }
  if (kind.empty() || robot_id.empty() || !payload.is_object() ||
      (stamp_s && !std::isfinite(*stamp_s)))
    throw std::invalid_argument("invalid exploration event envelope");
  exploration_event_detail::requireFinite(payload);
  const double wall = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  const nlohmann::json event = {
      {"v", 1}, {"kind", kind}, {"robot_id", robot_id},
      {"stamp", stamp_s ? nlohmann::json(*stamp_s) : nlohmann::json(nullptr)},
      {"wall", wall}, {"boot", boot}, {"seq", seq}, {"payload", payload}};
  const std::string line = "SDEVT1 " + event.dump();
  // Consumers accept 16384 bytes; leave room for RCLCPP/launch prefixes and
  // the newline before Docker splits a stdout record into log chunks.
  constexpr size_t transport_budget = 16384 - 1024;
  if (line.size() > transport_budget)
    throw std::invalid_argument("exploration event exceeds 15360-byte log transport budget");
  ++seq;
  return line;
}
}  // namespace mgg_ros
