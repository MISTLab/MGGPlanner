#include <nlohmann/json.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mgg_map_octomap/mola_map.h"

namespace {
using json = nlohmann::json;

Eigen::Isometry3d transform(const json& value) {
  if (!value.is_array() || value.size() != 4)
    throw std::invalid_argument("T_component_navigation must be 4x4");
  Eigen::Matrix4d matrix;
  for (std::size_t row = 0; row < 4; ++row) {
    if (!value[row].is_array() || value[row].size() != 4)
      throw std::invalid_argument("T_component_navigation must be 4x4");
    for (std::size_t column = 0; column < 4; ++column)
      matrix(row, column) = value[row][column].get<double>();
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.matrix() = matrix;
  return result;
}

std::vector<Eigen::Vector3d> points(const json& value) {
  const json defaults = json::array(
      {json::array({2.0, 0.0, 0.6}), json::array({3.0, 0.0, 0.6}),
       json::array({4.0, 0.0, 0.6})});
  const json& source = value.contains("points") ? value["points"] : defaults;
  if (!source.is_array() || source.size() > 4096)
    throw std::invalid_argument("points must be a bounded array");
  std::vector<Eigen::Vector3d> result;
  result.reserve(source.size());
  for (const auto& point : source) {
    if (!point.is_array() || point.size() != 3)
      throw std::invalid_argument("points must contain XYZ triples");
    result.emplace_back(point[0].get<double>(), point[1].get<double>(),
                        point[2].get<double>());
  }
  return result;
}

const char* name(const mgg::VoxelStatus status) {
  switch (status) {
    case mgg::VoxelStatus::kFree: return "free";
    case mgg::VoxelStatus::kOccupied: return "occupied";
    case mgg::VoxelStatus::kUnknown: return "unknown";
  }
  return "unknown";
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 3)
      throw std::invalid_argument("usage: mola_map_probe PEER_ROOT REQUEST.json");
    std::ifstream input(argv[2]);
    if (!input) throw std::runtime_error("cannot open request JSON");
    json request;
    input >> request;
    mgg::MolaMapConfig config;
    config.peer_root = std::filesystem::absolute(argv[1]).string();
    config.resolution = request.value("resolution_m", 0.2);
    config.snapshot_ttl_sec = request.value("snapshot_ttl_sec", 3.0);
    config.max_load_time = std::chrono::milliseconds(
        request.value("max_load_ms", 2000));
    mgg::MolaMap provider(config);
    mgg::MolaSnapshotRequest snapshot;
    snapshot.component_id = request.at("component_id").get<std::string>();
    snapshot.epoch = request.at("epoch").get<std::uint64_t>();
    snapshot.graph_revision = request.at("graph_revision").get<std::uint64_t>();
    snapshot.geometry_revision =
        request.at("geometry_revision").get<std::string>();
    snapshot.source_stamp_ns = request.at("source_stamp_ns").get<std::uint64_t>();
    snapshot.component_from_navigation =
        transform(request.at("T_component_navigation"));
    provider.requestSnapshot(snapshot);
    const auto deadline = std::chrono::steady_clock::now() +
                          config.max_load_time + std::chrono::seconds(1);
    while (!provider.getStatus() && provider.lastError().empty() &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    json output{{"status", provider.getStatus() ? "ready" : "unavailable"},
                {"detail", provider.lastError()},
                {"generation", provider.activeGeneration()},
                {"voxels", json::array()}};
    if (provider.getStatus())
      for (const auto& point : points(request))
        output["voxels"].push_back(name(provider.getVoxelStatus(point)));
    std::cout << output.dump() << '\n';
    return provider.getStatus() ? 0 : 2;
  } catch (const std::exception& error) {
    std::cout << json{{"status", "error"}, {"detail", error.what()}}.dump()
              << '\n';
    return 1;
  }
}
