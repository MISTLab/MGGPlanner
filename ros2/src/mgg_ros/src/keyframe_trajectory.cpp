#include "mgg_ros/keyframe_trajectory.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <utility>

#include <nlohmann/json.hpp>

namespace mgg_ros {

GraphSolutionFile::GraphSolutionFile(std::string path, std::string robot_id,
                                     std::size_t max_bytes)
    : path_(std::move(path)),
      robot_id_(std::move(robot_id)),
      max_bytes_(max_bytes) {}

bool GraphSolutionFile::read(KeyframeTrajectory& trajectory,
                             std::string& error) {
  std::error_code ec;
  const auto size = std::filesystem::file_size(path_, ec);
  if (ec) {
    error = "cannot read " + path_ + ": " + ec.message();
    return false;
  }
  if (size > max_bytes_) {
    error = path_ + " is larger than " + std::to_string(max_bytes_) + " bytes";
    return false;
  }
  std::ifstream in(path_, std::ios::binary);
  if (!in) {
    error = "cannot open " + path_;
    return false;
  }
  const std::string text((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());
  return parseGraphSolution(text, robot_id_, trajectory, error);
}

bool parseGraphSolution(const std::string& text, const std::string& robot_id,
                        KeyframeTrajectory& trajectory, std::string& error) {
  const nlohmann::json document = nlohmann::json::parse(text, nullptr, false);
  if (document.is_discarded() || !document.is_object()) {
    error = "the graph solution is not JSON";
    return false;
  }
  try {
    if (document.at("schema") != "swarmdeck.pose-snapshot.v1") {
      error = "unknown graph solution schema";
      return false;
    }
    const nlohmann::json& solution = document.at("solution");
    const nlohmann::json& revision = solution.at("revision");
    KeyframeTrajectory parsed;
    parsed.component_id = revision.at("component_id").get<std::string>();
    parsed.epoch = revision.at("epoch").get<std::uint64_t>();
    parsed.revision = revision.at("revision").get<std::uint64_t>();

    std::set<std::string> sessions;
    std::map<std::uint64_t, Eigen::Isometry3d> by_seq;
    for (const nlohmann::json& pose : solution.at("poses")) {
      const nlohmann::json& id = pose.at("keyframe_id");
      if (id.at("robot_id").get<std::string>() != robot_id) continue;
      sessions.insert(id.at("session_id").get<std::string>());
      const nlohmann::json& matrix = pose.at("T_component_keyframe");
      if (!matrix.is_array() || matrix.size() != 4) {
        error = "a keyframe pose is not a 4 x 4 matrix";
        return false;
      }
      Eigen::Matrix4d m;
      for (int r = 0; r < 4; ++r) {
        if (!matrix[r].is_array() || matrix[r].size() != 4) {
          error = "a keyframe pose is not a 4 x 4 matrix";
          return false;
        }
        for (int c = 0; c < 4; ++c) m(r, c) = matrix[r][c].get<double>();
      }
      if (!m.allFinite() ||
          (m.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-9) {
        error = "a keyframe pose is not a rigid transform";
        return false;
      }
      Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      t.matrix() = m;
      by_seq[id.at("seq").get<std::uint64_t>()] = t;
    }
    if (by_seq.empty()) {
      error = "no keyframe of " + robot_id + " in the graph solution";
      return false;
    }
    if (sessions.size() != 1) {
      error = "the graph solution holds " + std::to_string(sessions.size()) +
              " sessions of " + robot_id;
      return false;
    }
    if (by_seq.begin()->first != 0) {
      error = "the home keyframe (seq 0) of " + robot_id +
              " is not in the graph solution";
      return false;
    }
    parsed.poses.reserve(by_seq.size());
    for (const auto& [seq, pose] : by_seq) {
      (void)seq;
      parsed.poses.push_back(pose);
    }
    trajectory = std::move(parsed);
    return true;
  } catch (const nlohmann::json::exception& e) {
    error = std::string("malformed graph solution: ") + e.what();
    return false;
  }
}

}  // namespace mgg_ros
