#include <chrono>
#include <atomic>
#include <cmath>
#include <future>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mgg_msgs/srv/plan_objective.hpp>
#include <mgg_msgs/srv/planner_srv.hpp>
#include <mgg_msgs/msg/mapping_snapshot.hpp>
#include <mgg_msgs/srv/query_map_batch.hpp>

#include "mgg_ros/planner_node.h"

namespace mgg_ros {

class PlannerNodeTestPeer {
 public:
  static bool footprintTerrainSupported(PlannerNode& node,
                                        const Eigen::Vector3d& pose,
                                        const Eigen::Vector3d& body) {
    return node.objectiveFootprintTerrainSupported(pose, body);
  }
  static bool terrainPathSupported(
      PlannerNode& node, const std::vector<Eigen::Vector3d>& path) {
    return node.objectiveTerrainPathSupported(path);
  }
  static std::shared_ptr<mgg_msgs::srv::ValidateObjectiveRoute::Response>
  validateRoute(
      PlannerNode& node,
      const std::shared_ptr<mgg_msgs::srv::ValidateObjectiveRoute::Request>&
          request) {
    auto response =
        std::make_shared<mgg_msgs::srv::ValidateObjectiveRoute::Response>();
    node.onValidateObjectiveRoute(request, response);
    return response;
  }
  static void setValidationComponent(PlannerNode& node,
                                     const std::string& component) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.mapping_snapshot_.component_id = component;
    node.have_mapping_snapshot_ = true;
  }
  static void setFreshMappingComponent(PlannerNode& node,
                                       const std::string& component) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.mapping_snapshot_ = mgg_msgs::msg::MappingSnapshot();
    node.mapping_snapshot_.component_id = component;
    node.mapping_snapshot_.epoch = 7;
    node.mapping_snapshot_.graph_revision = 11;
    node.mapping_snapshot_.geometry_revision = std::string(64, 'a');
    node.mapping_snapshot_.source_stamp.sec = 42;
    node.mapping_snapshot_received_ = std::chrono::steady_clock::now();
    node.have_mapping_snapshot_ = true;
  }
  static void clearMappingSnapshot(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.have_mapping_snapshot_ = false;
  }
  static void setPlanningBody(PlannerNode& node,
                              const Eigen::Vector3d& size) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.size = size;
    node.robot_params_.size_extension.setZero();
  }
  static void setMaxStep(PlannerNode& node, double height) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.max_step_height = height;
  }
  static void setMaxInclination(PlannerNode& node, double inclination) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.max_inclination = inclination;
  }
  static void setMaxGroundHeight(PlannerNode& node, double height) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.max_ground_height = height;
  }
  static void setProvisionalUnknownGround(PlannerNode& node, bool enabled) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.provisional_unknown_ground_ = enabled;
  }
  static void setObservedGroundBodyEvidence(PlannerNode& node, bool enabled) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.observed_ground_body_evidence_ = enabled;
  }
  static void qualifyMolaPolicyForTest(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.map_backend_ = "mola_snapshot";
  }
  static void clearMolaPolicyForTest(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.map_backend_ = "cloud_octomap";
  }
  static void setObjectiveGridWindow(PlannerNode& node, double margin,
                                     double maximum_margin,
                                     std::size_t max_cells = 32768) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.objective_grid_limits_.detour_margin_m = margin;
    node.objective_grid_limits_.max_cells = max_cells;
    node.objective_grid_limits_.max_expansions = 16384;
    node.objective_grid_limits_.timeout = std::chrono::milliseconds(2000);
    node.objective_grid_max_margin_m_ = maximum_margin;
  }
  static std::string rebuildLocalGraph(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.buildLocalGraph();
  }
  static std::size_t localVertices(const PlannerNode& node) {
    return node.local_graph_->getNumVertices();
  }
  static mgg::StateVec localRootState(const PlannerNode& node) {
    const mgg::Vertex* root = node.local_graph_->getVertex(0);
    return root != nullptr
               ? root->state
               : mgg::StateVec::Constant(
                     std::numeric_limits<double>::quiet_NaN());
  }
  static bool retainTerrainSafeExplorationPath(
      const std::vector<mgg::StateVec>& lattice,
      const std::function<bool(const std::vector<Eigen::Vector3d>&)>& supported,
      std::vector<mgg::StateVec>& candidate) {
    return PlannerNode::retainTerrainSafeExplorationPath(lattice, supported,
                                                         candidate);
  }
  static void observeShallowRamp(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (double x = -0.3; x <= 0.3 + 1e-9; x += 0.05) {
      const double z = 0.05 * (x + 0.3) / 0.6;
      for (double y = -0.3; y <= 0.3 + 1e-9; y += 0.05) {
        for (int repeat = 0; repeat < 6; ++repeat) {
          node.cloud_map_->insertPointCloud(
              {Eigen::Vector3d(x, y, z)}, Eigen::Vector3d(x, y, 1.5));
        }
      }
    }
  }
  static void configureBackboneTest(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.type = mgg::RobotType::kGroundRobot;
    node.robot_params_.size = Eigen::Vector3d(0.20, 0.20, 0.15);
    node.robot_params_.size_extension.setZero();
    node.robot_params_.size_extension_min.setZero();
    node.robot_params_.safety_extension.setZero();
    node.robot_params_.bound_mode = mgg::BoundModeType::kExactBound;
    node.planning_params_.max_ground_height = 0.30;
    node.planning_params_.max_step_height = 0.10;
    node.planning_params_.max_inclination = 0.52;
    node.global_vertex_spacing_ = 0.50;
  }

  static void configureAerialBackboneTest(PlannerNode& node) {
    configureBackboneTest(node);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.type = mgg::RobotType::kAerialRobot;
  }

  static void acceptOdometry(PlannerNode& node, double x, double y, double z) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    msg->pose.pose.position.z = z;
    msg->pose.pose.orientation.w = 1.0;
    node.onOdometry(msg);
  }

  static void setCurrentStateWithoutExtendingBackbone(
      PlannerNode& node, double x, double y, double z) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.current_state_ = mgg::StateVec(x, y, z, 0.0);
    node.have_odometry_ = true;
  }

  static void observeGroundSupport(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    // Vertical rays supply ground for every projection probe but leave parts
    // of the swept body volume unknown.  That distinction exercises strict
    // global-edge admission rather than treating support as free clearance.
    for (int repeat = 0; repeat < 10; ++repeat) {
      for (double x = -0.3; x <= 2.1; x += 0.10) {
        for (const double y : {-0.2, 0.0, 0.2}) {
          node.cloud_map_->insertPointCloud({Eigen::Vector3d(x, y, 0.0)},
                                      Eigen::Vector3d(x, y, 1.5));
        }
      }
    }
    ++node.map_revision_;
    node.updateGlobalGraph();
  }

  static void observeBodyCorridor(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    // Model a fully observed free body volume after the ground-only phase.
    node.cloud_map_->augmentFreeBox(Eigen::Vector3d(0.75, 0.0, 0.35),
                              Eigen::Vector3d(2.5, 0.8, 0.20));
    ++node.map_revision_;
    node.updateGlobalGraph();
  }

  static void observeDestinationSupport(PlannerNode& node, double x) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 10; ++repeat) {
      for (const double dx : {-0.2, 0.0, 0.2}) {
        for (const double dy : {-0.2, 0.0, 0.2}) {
          node.cloud_map_->insertPointCloud(
              {Eigen::Vector3d(x + dx, dy, 0.0)},
              Eigen::Vector3d(x + dx, dy, 1.5));
        }
      }
    }
    node.cloud_map_->augmentFreeBox(Eigen::Vector3d(x, 0.0, 0.35),
                              Eigen::Vector3d(0.25, 0.25, 0.20));
    ++node.map_revision_;
  }

  static void observeGroundRectangle(PlannerNode& node, double xmin,
                                     double xmax, double ymin, double ymax) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 6; ++repeat) {
      for (double x = xmin; x <= xmax + 1e-9; x += 0.10) {
        for (double y = ymin; y <= ymax + 1e-9; y += 0.10) {
          node.cloud_map_->insertPointCloud({Eigen::Vector3d(x, y, 0.0)},
                                      Eigen::Vector3d(x, y, 1.5));
        }
      }
    }
  }

  static void observeGroundRectangleAt(PlannerNode& node, double xmin,
                                       double xmax, double ymin, double ymax,
                                       double z) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 6; ++repeat) {
      for (double x = xmin; x <= xmax + 1e-9; x += 0.10) {
        for (double y = ymin; y <= ymax + 1e-9; y += 0.10) {
          node.cloud_map_->insertPointCloud({Eigen::Vector3d(x, y, z)},
                                            Eigen::Vector3d(x, y, 1.5));
        }
      }
    }
  }

  static void observeFreeBodyBox(PlannerNode& node,
                                 const Eigen::Vector3d& center,
                                 const Eigen::Vector3d& size) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.cloud_map_->augmentFreeBox(center, size);
  }

  static void addOccupiedVoxel(PlannerNode& node, double x, double y,
                               double z) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 20; ++repeat) {
      node.cloud_map_->tree()->updateNode(
          octomap::point3d(static_cast<float>(x), static_cast<float>(y),
                           static_cast<float>(z)),
          true);
    }
  }

  static void addMeasuredSurface(PlannerNode& node, double x, double y,
                                 double z) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 20; ++repeat) {
      node.cloud_map_->insertPointCloud({Eigen::Vector3d(x, y, z)},
                                        Eigen::Vector3d(x, y, 1.5));
    }
    ++node.map_revision_;
  }
  static void addBlockingWall(PlannerNode& node, double x) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 20; ++repeat) {
      for (double y = -2.0; y <= 2.0; y += 0.10) {
        for (double z = 0.15; z <= 0.75; z += 0.10) {
          node.cloud_map_->tree()->updateNode(
              octomap::point3d(static_cast<float>(x), static_cast<float>(y),
                               static_cast<float>(z)),
              true);
        }
      }
    }
    ++node.map_revision_;
  }
  static void addBlockingWallSpan(PlannerNode& node, double x, double ymin,
                                  double ymax) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 20; ++repeat) {
      for (double y = ymin; y <= ymax + 1e-9; y += 0.05) {
        for (double z = 0.15; z <= 0.75 + 1e-9; z += 0.05) {
          node.cloud_map_->tree()->updateNode(
              octomap::point3d(static_cast<float>(x), static_cast<float>(y),
                               static_cast<float>(z)),
              true);
        }
      }
    }
    ++node.map_revision_;
  }

  static void finishMapRevision(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    ++node.map_revision_;
    node.updateGlobalGraph();
  }

  static void configureExploreServiceScene(PlannerNode& node,
                                           bool support_root) {
    configureBackboneTest(node);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.edge_length_min = 0.05;
    node.planning_params_.edge_length_max = 0.25;
    node.planning_params_.edge_overshoot = 0.0;
    node.planning_params_.num_vertices_max = 20;
    node.planning_params_.num_edges_max = 40;
    node.planning_params_.num_loops_max = 40;
    node.planning_params_.max_step_height = 0.30;
    node.planning_params_.path_interpolation_distance = 0.10;
    node.grid_params_.min_val = Eigen::Vector3d::Zero();
    node.grid_params_.max_val = Eigen::Vector3d(1.0, 0.0, 0.0);
    node.grid_params_.resolution = Eigen::Vector3d(0.50, 0.50, 0.20);
    node.global_space_.setBound(Eigen::Vector3d(-3.0, -3.0, -1.0),
                                Eigen::Vector3d(3.0, 3.0, 2.0));
    node.global_space_.min_extension.setZero();
    node.global_space_.max_extension.setZero();

    mgg::SensorParams sensor;
    sensor.type = mgg::SensorType::kLidar;
    sensor.max_range = 2.0;
    sensor.fov = Eigen::Vector2d(M_PI / 2.0, 0.20);
    sensor.resolution = Eigen::Vector2d(M_PI / 8.0, 0.20);
    sensor.frontier_percentage_threshold = 0.01;
    sensor.update();
    node.sensors_["test_lidar"] = sensor;
    node.planning_params_.exp_sensor_list = {"test_lidar"};

    // Cover every footprint box and offset ground probe used by the strict
    // global-edge check. Keep the lower face above the occupied floor.
    node.cloud_map_->augmentFreeBox(Eigen::Vector3d(0.60, 0.0, 0.40),
                              Eigen::Vector3d(2.4, 1.2, 0.60));
    for (int repeat = 0; repeat < 10; ++repeat) {
      for (double x = support_root ? -0.3 : 0.30; x <= 1.3; x += 0.10) {
        for (const double y : {-0.2, 0.0, 0.2}) {
          node.cloud_map_->insertPointCloud({Eigen::Vector3d(x, y, 0.0)},
                                      Eigen::Vector3d(x, y, 1.5));
        }
      }
    }
    node.cloud_map_->augmentFreeBox(Eigen::Vector3d(0.60, 0.0, 0.40),
                              Eigen::Vector3d(2.4, 1.2, 0.60));
    ++node.map_revision_;
  }

  static void addCorridorObstacle(PlannerNode& node, double x) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 20; ++repeat) {
      node.cloud_map_->insertPointCloud({Eigen::Vector3d(x, 0.0, 0.30)},
                                  Eigen::Vector3d(x, 0.0, 1.5));
    }
    ++node.map_revision_;
  }

  static void configureGridServiceScene(PlannerNode& node,
                                        double body_offset_z = 0.0,
                                        double margin = 1.0) {
    configureBackboneTest(node);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.center_offset = Eigen::Vector3d(0, 0, body_offset_z);
    node.grid_refinement_limits_.detour_margin_m = margin;
    // Functional service fixture, not a CPU latency benchmark. Core tests
    // exercise deadline expiry independently of DDS scheduling on the host.
    node.grid_refinement_limits_.timeout = std::chrono::milliseconds(1000);
    std::vector<Eigen::Vector3d> floor;
    for (int x = -8; x <= 32; ++x) {
      for (int y = -24; y <= 24; ++y) {
        floor.emplace_back(x * 0.05 + 0.025, y * 0.05 + 0.025, 0.025);
      }
    }
    for (int repeat = 0; repeat < 6; ++repeat) {
      node.cloud_map_->insertPointCloud(floor, Eigen::Vector3d(0.6, 0.0, 1.5));
    }
    // The known body volume is separate from the occupied supporting floor.
    node.cloud_map_->augmentFreeBox({0.6, 0.0, 0.5}, {3.2, 3.0, 0.8});
    ++node.map_revision_;
  }

  static void configureLongKnownRoad(PlannerNode& node) {
    configureBackboneTest(node);
    acceptOdometry(node, 0.0, 0.0, 0.075);
    {
      const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
      node.objective_grid_limits_.resolution_m = 0.50;
      node.objective_grid_limits_.detour_margin_m = 2.0;
      node.objective_grid_limits_.max_cells = 8192;
      node.objective_grid_limits_.max_expansions = 8192;
      node.objective_grid_limits_.timeout = std::chrono::milliseconds(1000);
      std::vector<Eigen::Vector3d> floor;
      for (int x = -2; x <= 602; ++x) {
        for (int y = -10; y <= 10; ++y) {
          floor.emplace_back(x * 0.05 + 0.025, y * 0.05 + 0.025, 0.025);
        }
      }
      for (int repeat = 0; repeat < 6; ++repeat) {
        node.cloud_map_->insertPointCloud(floor,
                                          Eigen::Vector3d(15.0, 0.0, 1.5));
      }
      node.cloud_map_->augmentFreeBox({15.0, 0.0, 0.50},
                                      {31.0, 1.0, 0.80});
      ++node.map_revision_;
    }
    acceptOdometry(node, 0.0, 0.0, 0.075);
  }

  static void configureLongKnownRamp(PlannerNode& node) {
    configureBackboneTest(node);
    acceptOdometry(node, 0.0, 0.0, 0.075);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.objective_grid_limits_.resolution_m = 0.20;
    node.objective_grid_limits_.detour_margin_m = 0.50;
    node.objective_grid_limits_.max_cells = 4096;
    node.objective_grid_limits_.max_expansions = 4096;
    node.objective_grid_limits_.timeout = std::chrono::milliseconds(1000);
    std::vector<Eigen::Vector3d> floor;
    for (int x = -4; x <= 64; ++x) {
      const double world_x = x * 0.05 + 0.025;
      const double ground_z = std::max(0.0, 0.10 * world_x) + 0.025;
      for (int y = -12; y <= 12; ++y) {
        floor.emplace_back(world_x, y * 0.05 + 0.025, ground_z);
      }
    }
    for (int repeat = 0; repeat < 6; ++repeat) {
      node.cloud_map_->insertPointCloud(floor,
                                        Eigen::Vector3d(1.5, 0.0, 1.5));
    }
    ++node.map_revision_;
  }

  static void configureKnownRampThenUnknown(PlannerNode& node) {
    configureBackboneTest(node);
    acceptOdometry(node, 0.0, 0.0, 0.075);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.objective_grid_limits_.resolution_m = 0.20;
    node.objective_grid_limits_.detour_margin_m = 0.50;
    node.objective_grid_max_margin_m_ = 0.50;
    node.objective_grid_limits_.max_cells = 4096;
    node.objective_grid_limits_.max_expansions = 4096;
    node.objective_grid_limits_.timeout = std::chrono::milliseconds(1000);
    std::vector<Eigen::Vector3d> floor;
    for (int x = -4; x <= 80; ++x) {
      const double world_x = x * 0.05 + 0.025;
      const double ground_z =
          world_x <= 1.50 ? std::max(0.0, 0.08 * world_x) : 0.12;
      for (int y = -12; y <= 12; ++y) {
        floor.emplace_back(world_x, y * 0.05 + 0.025, ground_z + 0.025);
      }
    }
    for (int repeat = 0; repeat < 6; ++repeat) {
      node.cloud_map_->insertPointCloud(floor,
                                        Eigen::Vector3d(1.0, 0.0, 1.5));
    }
    ++node.map_revision_;
  }

  static void configureNavigateGraphCorridor(
      PlannerNode& node, const std::vector<mgg::StateVec>& poses) {
    configureGridServiceScene(node, 0.0, 1.0);
    acceptOdometry(node, 0.0, 0.0, 0.075);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.objective_grid_limits_.resolution_m = 0.20;
    node.objective_grid_limits_.detour_margin_m = 1.0;
    node.objective_grid_limits_.max_cells = 2048;
    node.objective_grid_limits_.max_expansions = 2048;
    node.objective_grid_limits_.timeout = std::chrono::milliseconds(1000);
    node.global_graph_->reset();
    mgg::Vertex* previous = nullptr;
    for (std::size_t index = 0; index < poses.size(); ++index) {
      auto* vertex = new mgg::Vertex(static_cast<int>(index), poses[index]);
      node.global_graph_->addVertex(vertex);
      if (previous != nullptr) {
        node.global_graph_->addEdge(
            previous, vertex,
            (vertex->state.head<3>() - previous->state.head<3>()).norm());
      }
      previous = vertex;
    }
    ++node.graph_revision_;
  }

  static void addGridObstacle(PlannerNode& node, double z, bool wall,
                              double x = 0.9) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 20; ++repeat) {
      const int bound = wall ? 26 : 0;
      for (int y = -bound; y <= bound; ++y) {
        node.cloud_map_->tree()->updateNode(
            octomap::point3d(static_cast<float>(x),
                             static_cast<float>(y * 0.05),
                             static_cast<float>(z)),
            true);
      }
    }
    ++node.map_revision_;
  }

  static bool makeGridVoxelUnknown(PlannerNode& node, double x, double y,
                                   double z) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    octomap::OcTreeKey key;
    if (!node.cloud_map_->tree()->coordToKeyChecked(
            octomap::point3d(static_cast<float>(x), static_cast<float>(y),
                             static_cast<float>(z)),
            key)) {
      return false;
    }
    if (!node.cloud_map_->tree()->updateNode(key, true)) return false;
    node.cloud_map_->tree()->deleteNode(key, node.cloud_map_->tree()->getTreeDepth());
    const bool removed = node.cloud_map_->tree()->search(key) == nullptr;
    if (removed) ++node.map_revision_;
    return removed;
  }

  static void setGridGeofence(PlannerNode& node, double xmin, double xmax,
                              double ymin, double ymax) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.geofence_checking_enable = true;
    node.geofence_->clear();
    mgg::Polygon2d polygon(std::vector<Eigen::Vector2d>{
        {xmin, ymin}, {xmin, ymax}, {xmax, ymax}, {xmax, ymin}, {xmin, ymin}});
    node.geofence_->addGeofenceArea(polygon);
  }

  static void addRaisedFloor(PlannerNode& node, double step_height) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 20; ++repeat) {
      for (int x = 15; x <= 36; ++x) {
        for (int y = -12; y <= 12; ++y) {
          node.cloud_map_->tree()->updateNode(
              octomap::point3d(static_cast<float>(x * 0.05 + 0.025),
                               static_cast<float>(y * 0.05 + 0.025),
                               static_cast<float>(step_height + 0.025)),
              true);
        }
      }
    }
    ++node.map_revision_;
  }

  static void configurePartialNavigateScene(PlannerNode& node,
                                            bool blocked_by_wall = false,
                                            double step_height = 0.0,
                                            double step_cap = 0.10) {
    configureGridServiceScene(node);
    acceptOdometry(node, 0.0, 0.0, 0.075);
    if (blocked_by_wall) addGridObstacle(node, 0.325, true, 0.75);
    if (step_height > 0.0) addRaisedFloor(node, step_height);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.max_step_height = step_cap;
    node.local_graph_->reset();
    mgg::Vertex* previous = nullptr;
    for (int id = 0; id < 4; ++id) {
      auto* vertex =
          new mgg::Vertex(id, mgg::StateVec(id * 0.50, 0.0, 0.30, 0.0));
      node.local_graph_->addVertex(vertex);
      if (previous != nullptr) node.local_graph_->addEdge(previous, vertex, 0.50);
      previous = vertex;
    }
    node.local_graph_revision_ = 7;
    node.local_graph_map_revision_ = node.map_revision_;
  }

  static void configureBlindStartScene(PlannerNode& node,
                                       double supported_ground_z = 0.0) {
    configureBackboneTest(node);
    acceptOdometry(node, 0.0, 0.0, 0.075);
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.objective_start_support_max_distance_m_ = 3.0;
    node.grid_refinement_limits_.detour_margin_m = 0.0;
    node.grid_refinement_limits_.timeout = std::chrono::milliseconds(1000);
    // Observed free body corridor, while the floor beneath the first metre is
    // deliberately absent to model the VLP16 near-field ground blind spot.
    // Body clearance is explicitly observed; only the floor beneath the first
    // metre remains absent to model the VLP16 near-field ground blind spot.
    node.map_->augmentFreeBox(Eigen::Vector3d(0.675, 0.0, 0.50),
                              Eigen::Vector3d(1.65, 0.8, 0.55));
    for (int repeat = 0; repeat < 20; ++repeat) {
      for (double x = 1.0; x <= 1.5; x += 0.05) {
        for (const double y : {-0.20, 0.0, 0.20}) {
          node.cloud_map_->tree()->updateNode(
              octomap::point3d(static_cast<float>(x), static_cast<float>(y),
                               static_cast<float>(supported_ground_z + 0.025)),
              true);
        }
      }
    }
    node.map_->augmentFreeBox(Eigen::Vector3d(0.675, 0.0, 0.50),
                              Eigen::Vector3d(1.65, 0.8, 0.55));
    ++node.map_revision_;
    node.local_graph_->reset();
    auto* root = new mgg::Vertex(
        0, node.physicalAnchorAtDrivingHeight(node.current_state_));
    root->is_hanging = true;
    root->robot_id = static_cast<int>(node.planning_params_.robot_id);
    node.local_graph_->addVertex(root);
    mgg::StateVec supported(1.20, 0.0, supported_ground_z + 0.075, 0.0);
    if (!node.projectStateToDrivingHeight(supported)) {
      throw std::runtime_error("blind-start fixture failed to project support");
    }
    auto* destination = new mgg::Vertex(1, supported);
    destination->robot_id = root->robot_id;
    node.local_graph_->addVertex(destination);
    node.local_graph_->addEdge(root, destination,
                               (supported - root->state).norm());
    node.local_graph_revision_ = 41;
    node.local_graph_map_revision_ = node.map_revision_;
  }

  static void addBlindStartKnownFloor(PlannerNode& node, double x,
                                      double ground_z) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    for (int repeat = 0; repeat < 20; ++repeat) {
      for (double dx = -0.15; dx <= 0.15; dx += 0.05) {
        for (double y = -0.20; y <= 0.20; y += 0.05) {
          node.cloud_map_->tree()->updateNode(
              octomap::point3d(static_cast<float>(x + dx),
                               static_cast<float>(y),
                               static_cast<float>(ground_z + 0.025)),
              true);
        }
      }
    }
    ++node.map_revision_;
    node.local_graph_map_revision_ = node.map_revision_;
  }

  static void syncLocalGraphMapRevision(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.local_graph_map_revision_ = node.map_revision_;
  }

  static std::string rebuildBlindStartLocalGraph(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.edge_length_min = 0.05;
    node.planning_params_.edge_length_max = 0.50;
    node.planning_params_.edge_overshoot = 0.0;
    node.planning_params_.num_vertices_max = 80;
    node.planning_params_.num_edges_max = 160;
    node.planning_params_.num_loops_max = 200;
    node.grid_params_.min_val = Eigen::Vector3d(0.0, 0.0, 0.0);
    node.grid_params_.max_val = Eigen::Vector3d(1.5, 0.0, 0.30);
    node.grid_params_.resolution = Eigen::Vector3d(0.25, 0.50, 0.30);
    node.global_space_.setBound(Eigen::Vector3d(-3.0, -3.0, -1.0),
                                Eigen::Vector3d(3.0, 3.0, 2.0));
    node.global_space_.min_extension.setZero();
    node.global_space_.max_extension.setZero();
    return node.buildLocalGraph();
  }

  static std::shared_ptr<mgg_msgs::srv::PlanObjective::Response>
  requestBoundNavigate(PlannerNode& node, const mgg::StateVec& goal) {
    auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
    request->objective = mgg_msgs::srv::PlanObjective::Request::NAVIGATE;
    request->component_id = node.component_id_;
    request->graph_revision = node.graph_revision_;
    request->map_revision = node.map_revision_;
    request->goal.position.x = goal.x();
    request->goal.position.y = goal.y();
    request->goal.position.z = goal.z();
    request->goal.orientation.z = std::sin(goal[3] / 2.0);
    request->goal.orientation.w = std::cos(goal[3] / 2.0);
    if (node.have_mapping_snapshot_) {
      request->map_epoch = node.mapping_snapshot_.epoch;
      request->mapping_graph_revision = node.mapping_snapshot_.graph_revision;
      request->geometry_revision = node.mapping_snapshot_.geometry_revision;
      request->map_source_stamp = node.mapping_snapshot_.source_stamp;
    }
    auto response =
        std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
    node.onObjectiveRequest(request, response);
    return response;
  }

  static std::uint64_t localGraphRevision(const PlannerNode& node) {
    return node.local_graph_revision_;
  }

  static std::uint64_t localGraphMapRevision(const PlannerNode& node) {
    return node.local_graph_map_revision_;
  }

  static std::uint64_t globalGraphRevision(const PlannerNode& node) {
    return node.graph_revision_;
  }

  static std::string globalGraphDescription(const PlannerNode& node) {
    std::ostringstream out;
    for (const auto& entry : node.global_graph_->vertices_map_) {
      if (entry.second == nullptr) continue;
      out << "v" << entry.first << "=(" << entry.second->state.x() << ","
          << entry.second->state.y() << "," << entry.second->state.z() << ") ";
    }
    return out.str();
  }

  static int globalVertices(const PlannerNode& node) {
    return node.global_graph_->getNumVertices();
  }

  static int globalEdges(const PlannerNode& node) {
    return node.global_graph_->getNumEdges();
  }

  static std::size_t pendingBreadcrumbs(const PlannerNode& node) {
    return node.pending_global_breadcrumbs_.size();
  }

  static double pendingBreadcrumbLength(const PlannerNode& node) {
    return node.pending_global_length_m_;
  }

  static bool backboneHistoryLost(const PlannerNode& node) {
    return node.global_backbone_history_lost_;
  }

  static std::string backboneHistoryLostReason(const PlannerNode& node) {
    return node.global_backbone_history_lost_reason_;
  }

  static void setBackboneLimits(PlannerNode& node, std::size_t samples,
                                double length, std::size_t drain) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.pending_global_max_samples_ = samples;
    node.pending_global_max_length_m_ = length;
    node.pending_global_drain_max_samples_ = drain;
  }

  static void setBackboneSpacing(PlannerNode& node, double spacing) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.global_vertex_spacing_ = spacing;
  }

  static bool hasGlobalVertexNear(const PlannerNode& node, double x, double y,
                                  double tolerance) {
    for (const auto& entry : node.global_graph_->vertices_map_) {
      if (entry.second == nullptr) continue;
      if (std::hypot(entry.second->state.x() - x,
                     entry.second->state.y() - y) <= tolerance) {
        return true;
      }
    }
    return false;
  }

  static bool allGlobalEdgesAtMost(const PlannerNode& node, double limit) {
    for (const auto& entry : node.global_graph_->edge_map_) {
      for (const auto& edge : entry.second) {
        if (edge.second > limit) return false;
      }
    }
    return true;
  }

  static std::shared_ptr<mgg_msgs::srv::PlanObjective::Response>
  requestObjective(PlannerNode& node, mgg::ObjectiveKind objective,
                   const mgg::StateVec& goal,
                   const std::string& landmark_id = "",
                   std::uint64_t graph_revision = 0) {
    auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
    request->objective = static_cast<std::uint8_t>(objective);
    request->component_id = node.component_id_;
    request->goal.position.x = goal.x();
    request->goal.position.y = goal.y();
    request->goal.position.z = goal.z();
    request->goal.orientation.z = std::sin(goal[3] / 2.0);
    request->goal.orientation.w = std::cos(goal[3] / 2.0);
    request->goal_landmark_id = landmark_id;
    request->graph_revision = graph_revision;
    auto response =
        std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
    node.onObjectiveRequest(request, response);
    return response;
  }

  static std::shared_ptr<mgg_msgs::srv::PlanObjective::Response>
  requestMappedObjective(PlannerNode& node, mgg::ObjectiveKind objective,
                         const mgg::StateVec& goal,
                         const std::string& landmark_id = "") {
    auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
    request->objective = static_cast<std::uint8_t>(objective);
    request->component_id = node.mapping_snapshot_.component_id;
    request->goal.position.x = goal.x();
    request->goal.position.y = goal.y();
    request->goal.position.z = goal.z();
    request->goal.orientation.z = std::sin(goal[3] / 2.0);
    request->goal.orientation.w = std::cos(goal[3] / 2.0);
    request->goal_landmark_id = landmark_id;
    request->map_epoch = node.mapping_snapshot_.epoch;
    request->mapping_graph_revision = node.mapping_snapshot_.graph_revision;
    request->geometry_revision = node.mapping_snapshot_.geometry_revision;
    request->map_source_stamp = node.mapping_snapshot_.source_stamp;
    auto response =
        std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
    node.onObjectiveRequest(request, response);
    return response;
  }

  static std::shared_ptr<mgg_msgs::srv::RefineObjectiveRoute::Response>
  refineObjectiveRoute(
      PlannerNode& node,
      const std::shared_ptr<mgg_msgs::srv::RefineObjectiveRoute::Request>& request) {
    auto response =
        std::make_shared<mgg_msgs::srv::RefineObjectiveRoute::Response>();
    node.onRefineObjectiveRoute(request, response);
    return response;
  }

  static mgg::StateVec globalVertexState(const PlannerNode& node, int id) {
    return node.global_graph_->getVertex(id)->state;
  }
  static void makeGlobalVertexNonFinite(PlannerNode& node, int id) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.global_graph_->getVertex(id)->state.x() =
        std::numeric_limits<double>::quiet_NaN();
  }
  static void addBadHomeCorridor(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    auto* bad = new mgg::Vertex(
        1, mgg::StateVec(0.60, 0.46, 0.34, 0.0));
    auto* current = new mgg::Vertex(
        2, mgg::StateVec(1.20, 0.0, 0.30, 0.0));
    node.global_graph_->addVertex(bad);
    node.global_graph_->addVertex(current);
    node.global_graph_->addEdge(node.global_graph_->getVertex(0), bad, 0.8);
    node.global_graph_->addEdge(bad, current, 0.8);
    ++node.graph_revision_;
  }
  static void addLinearHomeCorridor(PlannerNode& node, int length) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    auto* previous = node.global_graph_->getVertex(0);
    for (int id = 1; id <= length; ++id) {
      auto* vertex =
          new mgg::Vertex(id, mgg::StateVec(id, 0.0, 0.30, 0.0));
      node.global_graph_->addVertex(vertex);
      node.global_graph_->addEdge(previous, vertex, 1.0);
      previous = vertex;
    }
    node.last_own_global_vertex_id_ = length;
    ++node.graph_revision_;
  }
  static void addSparseHomeCorridor(PlannerNode& node, double length) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    auto* current =
        new mgg::Vertex(1, mgg::StateVec(length, 0.0, 0.30, 0.0));
    node.global_graph_->addVertex(current);
    node.global_graph_->addEdge(node.global_graph_->getVertex(0), current,
                                length);
    node.last_own_global_vertex_id_ = 1;
    ++node.graph_revision_;
  }

  static bool initialAnchorSupported(const PlannerNode& node) {
    return node.initial_anchor_supported_;
  }
  static double routeProgressTolerance(const PlannerNode& node) {
    return node.objective_route_progress_tolerance_m_;
  }

  static mgg::RouteCorridor planHome(const PlannerNode& node,
                                     const mgg::StateVec& goal) {
    mgg::PlanningRequest request;
    request.objective = mgg::ObjectiveKind::kReturnHome;
    request.component_id = node.component_id_;
    request.graph_revision = node.graph_revision_;
    request.map_revision = node.map_revision_;
    request.goal.pose = goal;
    mgg::TopologicalGoalPlanner planner(
        node.component_id_, node.graph_revision_, node.map_revision_, 1.0);
    return planner.plan(*node.global_graph_, node.current_state_, request);
  }

  static mgg::FeasiblePath refine(PlannerNode& node,
                                  const mgg::RouteCorridor& corridor) {
    return node.refineCorridor(corridor);
  }

  static mgg::ProjectedEdgeStatus globalEdgeStatus(
      PlannerNode& node, mgg::StateVec from, mgg::StateVec to) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    if (!node.projectStateToDrivingHeight(from) ||
        !node.projectStateToDrivingHeight(to)) {
      return mgg::ProjectedEdgeStatus::kHanging;
    }
    std::vector<Eigen::Vector3d> projected;
    return node.ground_->getProjectedEdgeStatus(
        from.head(3), to.head(3), node.robot_params_.getPlanningSize(), true,
        projected, false);
  }

  static void acceptSnapshot(PlannerNode& node,
                             const mgg_msgs::msg::MappingSnapshot& snapshot) {
    node.onMappingSnapshot(
        std::make_shared<mgg_msgs::msg::MappingSnapshot>(snapshot));
  }

  static bool query(PlannerNode& node, mgg::FeasiblePath& path,
                    bool allow_explore_height_refinement = false) {
    PlannerNode::IndexedQueryContext context;
    {
      const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
      context = node.indexedQueryContext();
    }
    return node.queryIndexedMap(
        path, context, allow_explore_height_refinement,
        /*allow_prefix_truncation=*/allow_explore_height_refinement);
  }

  static bool queryExplicit(PlannerNode& node, mgg::FeasiblePath& path) {
    PlannerNode::IndexedQueryContext context;
    {
      const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
      context = node.indexedQueryContext();
    }
    return node.queryIndexedMap(path, context,
                                /*allow_height_refinement=*/true,
                                /*allow_prefix_truncation=*/false,
                                /*allow_bounded_unknown_tail=*/true);
  }

  /// Navigate/Home validation that may fall back to the validated prefix of a
  /// section whose terrain evidence ran out ahead.
  static bool queryContinuable(PlannerNode& node, mgg::FeasiblePath& path,
                               bool* truncated = nullptr,
                               bool qualified_ground = true) {
    PlannerNode::IndexedQueryContext context;
    {
      const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
      context = node.indexedQueryContext();
    }
    return node.queryIndexedMap(path, context, qualified_ground,
                                /*allow_prefix_truncation=*/false,
                                qualified_ground,
                                /*allow_continuable_prefix=*/true, truncated);
  }

  static bool queryAfterCapturedSnapshotReceiptAges(
      PlannerNode& node, mgg::FeasiblePath& path) {
    PlannerNode::IndexedQueryContext context;
    std::chrono::steady_clock::time_point received;
    {
      const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
      context = node.indexedQueryContext();
      received = node.mapping_snapshot_received_;
      node.mapping_snapshot_received_ =
          std::chrono::steady_clock::now() - std::chrono::seconds(10);
    }
    const bool result = node.queryIndexedMap(path, context);
    {
      const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
      node.mapping_snapshot_received_ = received;
    }
    return result;
  }

  static bool queryAfterCapturedSnapshotIdentityChanges(
      PlannerNode& node, mgg::FeasiblePath& path) {
    PlannerNode::IndexedQueryContext context;
    {
      const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
      context = node.indexedQueryContext();
      ++node.mapping_snapshot_.graph_revision;
    }
    const bool result = node.queryIndexedMap(path, context);
    {
      const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
      --node.mapping_snapshot_.graph_revision;
    }
    return result;
  }

  static bool queryReady(const PlannerNode& node) {
    return node.indexed_map_client_ && node.indexed_map_client_->service_is_ready();
  }

  static bool hasQueryClient(const PlannerNode& node) {
    return static_cast<bool>(node.indexed_map_client_);
  }

  static std::string cachedObjectiveRouteId(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.cached_objective_route_ ? node.cached_objective_route_->id : "";
  }

  static bool hasCachedObjectiveRoute(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return static_cast<bool>(node.cached_objective_route_);
  }

  static mgg::StateVec cachedObjectiveRouteEndpoint(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.cached_objective_route_
               ? node.cached_objective_route_->expected_endpoint
               : mgg::StateVec::Zero();
  }

  static std::size_t cachedObjectiveRouteNextIndex(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.cached_objective_route_
               ? node.cached_objective_route_->next_index
               : std::numeric_limits<std::size_t>::max();
  }

  static std::size_t cachedObjectiveRouteSize(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.cached_objective_route_
               ? node.cached_objective_route_->global_poses.size()
               : 0u;
  }

  static mgg_msgs::msg::MappingSnapshot mappingSnapshot(
      PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    return node.mapping_snapshot_;
  }

  static void setIndexedTerrainLimits(PlannerNode& node, double step,
                                      double inclination) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.planning_params_.max_step_height = step;
    node.planning_params_.max_inclination = inclination;
  }

  static void configureIndexedRobotGeometry(
      PlannerNode& node, const Eigen::Vector3d& physical_size,
      const Eigen::Vector3d& extension,
      const Eigen::Vector3d& center_offset, double max_ground_height) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.robot_params_.size = physical_size;
    node.robot_params_.size_extension = extension;
    node.robot_params_.size_extension_min.setZero();
    node.robot_params_.bound_mode = mgg::BoundModeType::kExtendedBound;
    node.robot_params_.center_offset = center_offset;
    node.planning_params_.max_ground_height = max_ground_height;
  }

  static void expireSnapshot(PlannerNode& node) {
    const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
    node.mapping_snapshot_received_ =
        std::chrono::steady_clock::now() - std::chrono::seconds(10);
  }
};

}  // namespace mgg_ros

namespace {

using namespace std::chrono_literals;
using Service = mgg_msgs::srv::PlanObjective;
using LegacyService = mgg_msgs::srv::PlannerSrv;

TEST(PlannerConfiguration, NonFiniteRouteProgressToleranceUsesSafeDefault) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("objective_route_progress_tolerance_m",
                        std::numeric_limits<double>::quiet_NaN()),
  });
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  EXPECT_DOUBLE_EQ(
      mgg_ros::PlannerNodeTestPeer::routeProgressTolerance(*node), 1.0);
}

TEST(PlannerConfiguration, MolaBackendRequiresExactRouteValidator) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.backend", "mola_snapshot"),
       rclcpp::Parameter("map.mola.peer_root", "/tmp")});
  EXPECT_THROW(
      { auto planner = std::make_shared<mgg_ros::PlannerNode>(options); },
      std::invalid_argument);
}

TEST(PlannerConfiguration, ObservedGroundPolicyIsExplicitAndSimulationOnly) {
  for (const std::string policy : {"unknown", "ObservedGround"}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("objective_body_evidence_policy", policy)});
    EXPECT_THROW(
        { auto planner = std::make_shared<mgg_ros::PlannerNode>(options); },
        std::invalid_argument);
  }
  rclcpp::NodeOptions hardware;
  hardware.parameter_overrides(
      {rclcpp::Parameter("objective_body_evidence_policy", "observed_ground")});
  EXPECT_THROW(
      { auto planner = std::make_shared<mgg_ros::PlannerNode>(hardware); },
      std::invalid_argument);

  rclcpp::NodeOptions mola;
  mola.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("map.backend", "mola_snapshot"),
       rclcpp::Parameter("indexed_map_query_service", "/query"),
       rclcpp::Parameter("map.mola.peer_root", "/tmp")});
  EXPECT_NO_THROW(
      { auto planner = std::make_shared<mgg_ros::PlannerNode>(mola); });
}

TEST(PlannerConfiguration, ProvisionalGroundRequiresObservedSimulationCloud) {
  for (const std::string body : {"strict_volume", "observed_ground"}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("objective_body_evidence_policy", body),
         rclcpp::Parameter("objective_ground_evidence_policy",
                           "provisional_unknown")});
    EXPECT_THROW(
        { auto planner = std::make_shared<mgg_ros::PlannerNode>(options); },
        std::invalid_argument);
  }
  rclcpp::NodeOptions invalid;
  invalid.parameter_overrides(
      {rclcpp::Parameter("objective_ground_evidence_policy", "unknown")});
  EXPECT_THROW(
      { auto planner = std::make_shared<mgg_ros::PlannerNode>(invalid); },
      std::invalid_argument);
}

TEST(PlannerObjective, ProvisionalUnknownGroundRetainsGoalAndKnownVetoes) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  auto make = [] {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true),
         rclcpp::Parameter("map.resolution", 0.15),
         rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
         rclcpp::Parameter("objective_ground_evidence_policy",
                           "provisional_unknown")});
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureBackboneTest(*node);
    Peer::acceptOdometry(*node, 0.0, 0.0, 0.075);
    // Give the map valid data away from the requested corridor while leaving
    // the entire route's ground and air unobserved.
    Peer::observeGroundRectangle(*node, -0.3, 0.3, 8.0, 8.3);
    Peer::finishMapRevision(*node);
    return node;
  };
  auto corridor = [] {
    mgg::RouteCorridor route;
    route.status = mgg::PlanningStatus::kSucceeded;
    route.request.objective = mgg::ObjectiveKind::kNavigate;
    route.request.goal.pose = mgg::StateVec(4.0, 0.0, 0.075, 1.1);
    return route;
  };

  auto clear = make();
  const mgg::FeasiblePath path = Peer::refine(*clear, corridor());
  ASSERT_EQ(path.status, mgg::PlanningStatus::kSucceeded) << path.reason;
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 4.0, 1e-9);
  EXPECT_NEAR(path.poses.back().y(), 0.0, 1e-9);
  EXPECT_NEAR(path.poses.back()[3], 1.1, 1e-9);
  EXPECT_FALSE(path.indexed_map_validated);

  // Footprint results are scoped to one refinement. A known curb inserted
  // after a successful request must be queried and veto the next request.
  Peer::addMeasuredSurface(*clear, 4.0, 0.0, 0.125);
  EXPECT_EQ(Peer::refine(*clear, corridor()).status,
            mgg::PlanningStatus::kBlocked);

  auto wall = make();
  Peer::addBlockingWall(*wall, 2.0);
  EXPECT_EQ(Peer::refine(*wall, corridor()).status,
            mgg::PlanningStatus::kBlocked);

  auto curb = make();
  Peer::addMeasuredSurface(*curb, 4.0, 0.0, 0.125);
  EXPECT_EQ(Peer::refine(*curb, corridor()).status,
            mgg::PlanningStatus::kBlocked);

  auto fenced = make();
  Peer::setGridGeofence(*fenced, -0.5, 0.5, -0.5, 0.5);
  EXPECT_EQ(Peer::refine(*fenced, corridor()).status,
            mgg::PlanningStatus::kBlocked);
}

TEST(PlannerObjective, ProvisionalUnknownGroundReachesDistantExactGoals) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBackboneTest(*node);
  Peer::setPlanningBody(*node, Eigen::Vector3d(1.023, 0.778, 0.40));
  Peer::setMaxGroundHeight(*node, 0.475);
  Peer::acceptOdometry(*node, 0.0, 0.0, 0.20);
  Peer::observeGroundRectangle(*node, -0.6, 0.6, -0.5, 0.5);
  Peer::finishMapRevision(*node);
  for (const double distance : {25.0, 50.0, 100.0}) {
    const auto response = Peer::requestObjective(
        *node, mgg::ObjectiveKind::kNavigate,
        mgg::StateVec(distance, 0.0, 0.20, 0.7));
    ASSERT_EQ(response->status,
              mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
        << distance << "m: " << response->reason;
    EXPECT_TRUE(response->partial);
    EXPECT_FALSE(response->route_id.empty());
    ASSERT_FALSE(response->path.empty());
    EXPECT_LE(response->path.back().position.x, 8.0 + 1e-9);
    ASSERT_FALSE(response->global_path.empty());
    EXPECT_NEAR(response->global_path.back().position.x, distance, 1e-9);
    EXPECT_NEAR(response->global_path.back().orientation.z, std::sin(0.35),
                1e-9);
  }
}

TEST(PlannerObjective,
     UnsupportedFarNavigateKeepsFirstHorizonOnDrivingPlane) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_route_horizon_m", 8.0),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy",
                         "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBackboneTest(*node);
  Peer::setPlanningBody(*node, Eigen::Vector3d(1.023, 0.778, 0.40));
  Peer::setMaxGroundHeight(*node, 0.475);
  Peer::acceptOdometry(*node, 0.0, 0.0, 0.20);
  Peer::observeGroundRectangle(*node, -0.6, 0.6, -0.5, 0.5);
  Peer::finishMapRevision(*node);

  const auto response = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kNavigate,
      // Deliberately use a UI/base height unrelated to the graph's driving
      // convention. Terrain at this distant endpoint is still unknown.
      mgg::StateVec(12.0, 0.0, 0.0, 0.7));
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_TRUE(response->partial);
  ASSERT_FALSE(response->path.empty());
  EXPECT_NEAR(response->path.back().position.x, 8.0, 1e-6);
  EXPECT_NEAR(response->path.back().position.z, 0.20, 1e-6);
  ASSERT_FALSE(response->global_path.empty());
  EXPECT_NEAR(response->global_path.back().position.x, 12.0, 1e-9);
  EXPECT_NEAR(response->global_path.back().position.z, 0.0, 1e-9);
  EXPECT_NEAR(response->global_path.back().orientation.z, std::sin(0.35),
              1e-9);
}

TEST(PlannerObjective, ExplicitObjectiveUsesBoundedWiderDetourWindow) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  const auto make = [](double maximum_margin,
                       std::size_t max_cells = 32768) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true),
         rclcpp::Parameter("map.resolution", 0.15),
         rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
         rclcpp::Parameter("objective_ground_evidence_policy",
                           "provisional_unknown")});
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureBackboneTest(*node);
    Peer::setObjectiveGridWindow(*node, 4.0, maximum_margin, max_cells);
    Peer::acceptOdometry(*node, 0.0, 0.0, 0.075);
    // Make the map usable without adding floor support to the unknown route.
    Peer::observeGroundRectangle(*node, -0.3, 0.3, 12.0, 12.3);
    // This known wall seals the complete +/-4 m search window. A route exists
    // around either end inside the separately bounded +/-8 m window.
    Peer::addBlockingWallSpan(*node, 5.0, -4.6, 4.6);
    Peer::finishMapRevision(*node);
    return node;
  };
  const mgg::StateVec goal(10.0, 0.0, 0.075, 0.7);

  auto narrow = make(4.0);
  const auto blocked = Peer::requestBoundNavigate(*narrow, goal);
  ASSERT_NE(blocked, nullptr);
  EXPECT_EQ(blocked->status, Service::Response::BLOCKED);
  EXPECT_NE(blocked->reason.find("within the bounded search window"),
            std::string::npos)
      << blocked->reason;

  // A requested wider window must never exceed the existing allocation cap.
  // This cap represents the +/-4 m corridor but cannot represent the detour.
  auto cell_bounded = make(8.0, 3000);
  const auto bounded = Peer::requestBoundNavigate(*cell_bounded, goal);
  ASSERT_NE(bounded, nullptr);
  EXPECT_EQ(bounded->status, Service::Response::BLOCKED);
  EXPECT_NE(bounded->reason.find("within the bounded search window"),
            std::string::npos)
      << bounded->reason;

  auto widened = make(8.0);
  const auto response = Peer::requestBoundNavigate(*widened, goal);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  EXPECT_TRUE(response->partial);
  ASSERT_FALSE(response->path.empty());
  EXPECT_LE(response->path.back().position.x, 8.0 + 1e-6);
  EXPECT_NEAR(response->path.back().orientation.z, std::sin(0.35), 1e-9);
  EXPECT_NEAR(response->path.back().orientation.w, std::cos(0.35), 1e-9);
  EXPECT_TRUE(std::any_of(response->path.begin(), response->path.end(),
                          [](const auto& pose) {
                            return std::abs(pose.position.y) > 4.7;
                          }));
}

TEST(PlannerObjective, ProvisionalUnknownGroundReturnsToPhysicalHomeWithoutFloorHits) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBackboneTest(*node);
  Peer::acceptOdometry(*node, 0.0, 0.0, 0.20);
  Peer::acceptOdometry(*node, 1.2, 0.0, 0.20);
  // Make the map available without inventing support at either physical
  // endpoint; both coordinates retain odometric provenance.
  Peer::observeGroundRectangle(*node, 8.0, 8.3, 8.0, 8.3);
  Peer::finishMapRevision(*node);
  ASSERT_FALSE(Peer::initialAnchorSupported(*node));
  const auto response = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0, 0.0, 0.20, 0.4), "kf-home");
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-9);
}

TEST(PlannerObjective, ProvisionalHomeNeverOverridesAStaleRevision) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBackboneTest(*node);
  Peer::acceptOdometry(*node, 0.0, 0.0, 0.20);
  Peer::observeGroundRectangle(*node, 8.0, 8.3, 8.0, 8.3);
  Peer::finishMapRevision(*node);
  const auto response = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0, 0.0, 0.20, 0.0), "kf-home", 999);
  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::STALE_REVISION);
}

TEST(PlannerObjective, DistantPhysicalHomeRequiresPersistentGraph) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_route_horizon_m", 8.0),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBackboneTest(*node);
  Peer::acceptOdometry(*node, 0.0, 0.0, 0.20);
  Peer::acceptOdometry(*node, 20.0, 0.0, 0.20);
  Peer::observeGroundRectangle(*node, 30.0, 30.3, 8.0, 8.3);
  Peer::finishMapRevision(*node);
  const auto response = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0, 0.0, 0.20, 0.0), "kf-home");
  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::UNREACHABLE);
  EXPECT_TRUE(response->path.empty());
}

TEST(PlannerObjective, PhysicalHomeUsesPersistentGraphCorridor) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*node);
  Peer::acceptOdometry(*node, 1.20, 0.0, 0.075);
  Peer::addBadHomeCorridor(*node);
  const auto response = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0, 0.0, 0.075, 0.2), "kf-home");
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 3u);
  EXPECT_TRUE(std::any_of(response->path.begin(), response->path.end(),
                          [](const geometry_msgs::msg::Pose& pose) {
                            return pose.position.y > 0.30;
                          }));
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-9);
}

TEST(PlannerObjective, LongHomeKeepsGlobalRouteAndRefinesBoundedWindows) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_route_horizon_m", 3.0),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*node);
  Peer::addSparseHomeCorridor(*node, 20.0);
  Peer::acceptOdometry(*node, 20.0, 0.0, 0.075);
  Peer::setFreshMappingComponent(*node, "shared-component");
  SCOPED_TRACE(Peer::globalGraphDescription(*node));

  auto response = Peer::requestMappedObjective(
      *node, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0, 0.0, 0.075, 0.4), "kf-home");
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_TRUE(response->partial);
  ASSERT_FALSE(response->route_id.empty());
  ASSERT_EQ(response->global_path.size(), 2u);
  EXPECT_LE(std::abs(response->path.back().position.x - 20.0), 3.01);
  EXPECT_NEAR(response->global_path.back().position.x, 0.0, 1e-9);
  EXPECT_NEAR(response->global_path.back().orientation.z, std::sin(0.2), 1e-9);
  const auto fixed_global = response->global_path;
  const std::string route_id = response->route_id;

  auto wrong_component =
      std::make_shared<mgg_msgs::srv::RefineObjectiveRoute::Request>();
  wrong_component->route_id = route_id;
  wrong_component->component_id = response->component_id;
  wrong_component->map_epoch = response->map_epoch;
  wrong_component->mapping_graph_revision = response->mapping_graph_revision;
  wrong_component->geometry_revision = response->geometry_revision;
  wrong_component->map_source_stamp = response->map_source_stamp;
  Peer::setFreshMappingComponent(*node, "another-component");
  const auto stale = Peer::refineObjectiveRoute(*node, wrong_component);
  EXPECT_EQ(stale->status,
            mgg_msgs::srv::RefineObjectiveRoute::Response::STALE_REVISION);
  Peer::setFreshMappingComponent(*node, "shared-component");

  // A continuation cannot jump ahead merely because the global graph later
  // grows or crosses near another part of the retained route.
  auto early =
      std::make_shared<mgg_msgs::srv::RefineObjectiveRoute::Request>();
  early->route_id = route_id;
  early->component_id = response->component_id;
  early->map_epoch = response->map_epoch;
  early->mapping_graph_revision = response->mapping_graph_revision;
  early->geometry_revision = response->geometry_revision;
  early->map_source_stamp = response->map_source_stamp;
  auto rejected = Peer::refineObjectiveRoute(*node, early);
  EXPECT_EQ(rejected->status,
            mgg_msgs::srv::RefineObjectiveRoute::Response::BLOCKED);
  EXPECT_NE(rejected->reason.find("outside the completed objective route window"),
            std::string::npos);

  bool complete = false;
  for (int chunk = 0; chunk < 10 && !complete; ++chunk) {
    ASSERT_FALSE(response->path.empty());
    const auto& endpoint = response->path.back().position;
    Peer::acceptOdometry(*node, endpoint.x, endpoint.y, endpoint.z);
    auto request =
        std::make_shared<mgg_msgs::srv::RefineObjectiveRoute::Request>();
    request->mission_id = "";
    request->route_id = route_id;
    request->component_id = response->component_id;
    request->map_epoch = response->map_epoch;
    request->mapping_graph_revision = response->mapping_graph_revision;
    request->geometry_revision = response->geometry_revision;
    request->map_source_stamp = response->map_source_stamp;
    auto next = Peer::refineObjectiveRoute(*node, request);
    ASSERT_EQ(next->status,
              mgg_msgs::srv::RefineObjectiveRoute::Response::SUCCEEDED)
        << next->reason;
    complete = !next->partial;
    response->path = next->path;
    response->partial = next->partial;
  }
  EXPECT_TRUE(complete);
  ASSERT_FALSE(response->path.empty());
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-9);
  EXPECT_NEAR(fixed_global.back().position.x, 0.0, 1e-9);
  auto expired =
      std::make_shared<mgg_msgs::srv::RefineObjectiveRoute::Request>();
  expired->route_id = route_id;
  expired->component_id = response->component_id;
  const auto after_complete = Peer::refineObjectiveRoute(*node, expired);
  EXPECT_EQ(after_complete->status,
            mgg_msgs::srv::RefineObjectiveRoute::Response::BLOCKED);
}

TEST(PlannerObjective, PartialHomeDetoursAroundOccupiedGraphWaypoint) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_route_horizon_m", 1.4),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*node);
  Peer::addLinearHomeCorridor(*node, 3);
  Peer::setCurrentStateWithoutExtendingBackbone(*node, 3.0, 0.0, 0.075);
  Peer::addGridObstacle(*node, 0.325, false, 2.0);

  const auto response = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0, 0.0, 0.075, 0.4), "kf-home");

  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  ASSERT_TRUE(response->partial);
  ASSERT_FALSE(response->route_id.empty());
  ASSERT_EQ(response->global_path.size(), 4u);
  EXPECT_NEAR(response->global_path.back().position.x, 0.0, 1e-9);
  EXPECT_NEAR(response->global_path.back().orientation.z, std::sin(0.2), 1e-9);
  ASSERT_FALSE(response->path.empty());
  EXPECT_NEAR(response->path.back().position.x, 1.6, 1e-6);
  EXPECT_TRUE(std::any_of(response->path.begin(), response->path.end(),
                          [](const geometry_msgs::msg::Pose& pose) {
                            return std::abs(pose.position.y) > 0.1;
                          }));
}

TEST(PlannerObjective, RollingHomeDetourPreservesRouteTokenAndProgress) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_route_horizon_m", 1.4),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*node);
  Peer::addLinearHomeCorridor(*node, 3);
  Peer::setCurrentStateWithoutExtendingBackbone(*node, 3.0, 0.0, 0.075);
  auto initial = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0, 0.0, 0.075, 0.4), "kf-home");
  ASSERT_EQ(initial->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << initial->reason;
  ASSERT_TRUE(initial->partial);
  ASSERT_FALSE(initial->route_id.empty());
  const std::string route_id = initial->route_id;
  const auto fixed_global = initial->global_path;
  ASSERT_FALSE(initial->path.empty());
  const auto& first_endpoint = initial->path.back().position;
  Peer::setCurrentStateWithoutExtendingBackbone(
      *node, first_endpoint.x, first_endpoint.y, first_endpoint.z);
  Peer::addGridObstacle(*node, 0.325, false, 1.0);

  auto request =
      std::make_shared<mgg_msgs::srv::RefineObjectiveRoute::Request>();
  request->route_id = route_id;
  request->component_id = initial->component_id;
  auto detour = Peer::refineObjectiveRoute(*node, request);
  ASSERT_EQ(detour->status,
            mgg_msgs::srv::RefineObjectiveRoute::Response::SUCCEEDED)
      << detour->reason;
  ASSERT_TRUE(detour->partial);
  ASSERT_FALSE(detour->path.empty());
  EXPECT_TRUE(std::any_of(detour->path.begin(), detour->path.end(),
                          [](const geometry_msgs::msg::Pose& pose) {
                            return std::abs(pose.position.y) > 0.1;
                          }));

  const auto& second_endpoint = detour->path.back().position;
  Peer::setCurrentStateWithoutExtendingBackbone(
      *node, second_endpoint.x, second_endpoint.y, second_endpoint.z);
  const auto final = Peer::refineObjectiveRoute(*node, request);
  ASSERT_EQ(final->status,
            mgg_msgs::srv::RefineObjectiveRoute::Response::SUCCEEDED)
      << final->reason;
  EXPECT_FALSE(final->partial);
  ASSERT_FALSE(final->path.empty());
  EXPECT_NEAR(final->path.back().position.x, 0.0, 1e-9);
  ASSERT_FALSE(fixed_global.empty());
  EXPECT_NEAR(fixed_global.back().position.x, 0.0, 1e-9);

  const auto expired = Peer::refineObjectiveRoute(*node, request);
  EXPECT_EQ(expired->status,
            mgg_msgs::srv::RefineObjectiveRoute::Response::BLOCKED);
}

TEST(PlannerObjective, PartialHomeNeverSkipsAnOccupiedLocalEndpoint) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_route_horizon_m", 1.4),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*node);
  Peer::addLinearHomeCorridor(*node, 3);
  Peer::setCurrentStateWithoutExtendingBackbone(*node, 3.0, 0.0, 0.075);
  Peer::addGridObstacle(*node, 0.325, false, 1.6);

  const auto response = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0, 0.0, 0.075, 0.0), "kf-home");

  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::BLOCKED);
  EXPECT_TRUE(response->path.empty());
  EXPECT_TRUE(response->route_id.empty());
  EXPECT_NE(response->reason.find("route corridor waypoint["),
            std::string::npos);
  EXPECT_EQ(response->reason.find("local endpoint fallback"),
            std::string::npos);
}

TEST(PlannerObjective, OccupiedFinalHomeRemainsBlockedAfterGraphHintFallback) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*node);
  Peer::addLinearHomeCorridor(*node, 1);
  Peer::setCurrentStateWithoutExtendingBackbone(*node, 1.0, 0.0, 0.075);
  Peer::addGridObstacle(*node, 0.325, false, 0.0);

  const auto response = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0, 0.0, 0.075, 0.0), "kf-home");

  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::BLOCKED);
  EXPECT_TRUE(response->path.empty());
  EXPECT_NE(response->reason.find("exact goal rejected"), std::string::npos);
  EXPECT_NE(response->reason.find("direct fallback"),
            std::string::npos);
}

TEST(PlannerObjective, HomeNeverSkipsMalformedGraphGeometry) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*node);
  Peer::addLinearHomeCorridor(*node, 3);
  Peer::setCurrentStateWithoutExtendingBackbone(*node, 3.0, 0.0, 0.075);
  Peer::makeGlobalVertexNonFinite(*node, 2);

  const auto response = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0, 0.0, 0.075, 0.0), "kf-home");

  EXPECT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::BLOCKED);
  EXPECT_TRUE(response->path.empty());
  EXPECT_NE(response->reason.find("state is non-finite"), std::string::npos);
  EXPECT_EQ(response->reason.find("local endpoint fallback"),
            std::string::npos);
}

TEST(PlannerExplore, FullSizeGroundRobotsExpandAtProjectedDrivingHeight) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  for (const auto& body : {
           Eigen::Vector3d(1.023, 0.778, 0.40),  // Bunker
           Eigen::Vector3d(0.85, 0.55, 0.35),   // Spot
       }) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("map.resolution", 0.15),
        rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
        rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureExploreServiceScene(*node, true);
    Peer::observeGroundRectangle(*node, -0.6, 1.3, -0.6, 0.6);
    Peer::observeFreeBodyBox(*node, Eigen::Vector3d(0.35, 0.0, 0.65),
                            Eigen::Vector3d(2.2, 1.2, 0.60));
    Peer::setPlanningBody(*node, body);
    Peer::setMaxGroundHeight(*node, 0.475);
    Peer::acceptOdometry(*node, 0.0, 0.0, body.z() / 2.0);
    const std::string summary = Peer::rebuildLocalGraph(*node);
    EXPECT_GT(Peer::localVertices(*node), 1u) << body.transpose() << ": " << summary;
    const auto response = Peer::requestObjective(
        *node, mgg::ObjectiveKind::kExplore, mgg::StateVec::Zero());
    EXPECT_EQ(response->status,
              mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
        << body.transpose() << ": " << response->reason;
    EXPECT_GE(response->path.size(), 2u);
  }
}

TEST(PlannerExplore, QuantizedFloorHeightStillConnectsPhysicalRoot) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy", "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureExploreServiceScene(*node, false);
  Peer::setPlanningBody(*node, Eigen::Vector3d(1.023, 0.778, 0.40));
  Peer::setMaxGroundHeight(*node, 0.475);
  Peer::acceptOdometry(*node, 0.0, 0.0, 0.20);
  // The first mapped road begins outside the under-body blind spot and its
  // quantized surface is not exactly the odometric driving plane.
  Peer::observeGroundRectangleAt(*node, 0.30, 1.30, -0.60, 0.60, 0.04);
  Peer::observeFreeBodyBox(*node, Eigen::Vector3d(0.65, 0.0, 0.70),
                          Eigen::Vector3d(2.0, 1.2, 0.60));
  const auto response = Peer::requestObjective(
      *node, mgg::ObjectiveKind::kExplore, mgg::StateVec::Zero());
  ASSERT_EQ(response->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << response->reason;
  EXPECT_GE(response->path.size(), 2u);
}

TEST(PlannerExplore, QualifiedMolaRootKeepsPhysicalDrivingHeight) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.15),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy",
                         "provisional_unknown")});

  const auto configure = [&options](bool mola_policy) {
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureExploreServiceScene(*node, true);
    Peer::setPlanningBody(*node, Eigen::Vector3d(1.023, 0.778, 0.40));
    Peer::setMaxGroundHeight(*node, 0.475);
    // Deliberately disagree with the measured floor. The qualified policy
    // must preserve the physical start while the legacy projection follows
    // the mapped surface.
    Peer::acceptOdometry(*node, 0.0, 0.0, 0.30);
    if (mola_policy) Peer::qualifyMolaPolicyForTest(*node);
    Peer::rebuildLocalGraph(*node);
    return node;
  };

  auto mola = configure(true);
  EXPECT_NEAR(Peer::localRootState(*mola).z(), 0.575, 1e-9);
}

TEST(PlannerExplore, InvalidSmoothedPathRestoresExactValidatedLattice) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  const std::vector<mgg::StateVec> lattice{
      mgg::StateVec(0.0, 0.0, 0.4, 0.0),
      mgg::StateVec(0.0, 1.0, 0.4, M_PI_2),
      mgg::StateVec(1.0, 1.0, 0.4, 0.0)};
  std::vector<mgg::StateVec> smoothed{
      lattice.front(), mgg::StateVec(1.0, 1.0, 0.4, M_PI_4)};
  bool examined_shortcut = false;
  const bool accepted = Peer::retainTerrainSafeExplorationPath(
      lattice,
      [&examined_shortcut](const std::vector<Eigen::Vector3d>& candidate) {
        examined_shortcut = true;
        return candidate.size() != 2;  // the diagonal shortcut is hazardous
      },
      smoothed);
  EXPECT_FALSE(accepted);
  EXPECT_TRUE(examined_shortcut);
  ASSERT_EQ(smoothed.size(), lattice.size());
  for (std::size_t i = 0; i < lattice.size(); ++i) {
    EXPECT_TRUE(smoothed[i].isApprox(lattice[i], 0.0));
  }
}

TEST(PlannerObjective, RemainingRouteValidationUsesLatestKnownHazards) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  using Service = mgg_msgs::srv::ValidateObjectiveRoute;
  auto make = [](double resolution = 0.15) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true),
         rclcpp::Parameter("mission_id", "mission-test"),
         rclcpp::Parameter("map.resolution", resolution),
         rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
         rclcpp::Parameter("objective_ground_evidence_policy",
                           "provisional_unknown")});
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureBackboneTest(*node);
    Peer::acceptOdometry(*node, 0.0, 0.0, 0.075);
    Peer::observeGroundRectangle(*node, -0.3, 0.3, 8.0, 8.3);
    Peer::finishMapRevision(*node);
    return node;
  };
  auto request = [](double lookahead = 3.0) {
    auto req = std::make_shared<Service::Request>();
    req->mission_id = "mission-test";
    req->component_id = "world";
    req->frame_id = "world";
    req->lookahead_m = lookahead;
    for (double x : {0.0, 1.0, 2.0, 3.0, 4.0}) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.z = 0.075;
      pose.orientation.w = 1.0;
      req->path.push_back(pose);
    }
    return req;
  };

  auto clear = make();
  EXPECT_EQ(Peer::validateRoute(*clear, request())->status, Service::Response::VALID);

  auto ahead = make();
  Peer::addMeasuredSurface(*ahead, 2.0, 0.0, 0.125);
  EXPECT_EQ(Peer::validateRoute(*ahead, request())->status,
            Service::Response::INVALID);

  auto behind = make();
  Peer::acceptOdometry(*behind, 3.0, 0.0, 0.075);
  Peer::addMeasuredSurface(*behind, 1.0, 0.0, 0.125);
  EXPECT_EQ(Peer::validateRoute(*behind, request())->status,
            Service::Response::VALID);

  auto bounded = make();
  Peer::addMeasuredSurface(*bounded, 4.0, 0.0, 0.125);
  EXPECT_EQ(Peer::validateRoute(*bounded, request(2.0))->status,
            Service::Response::VALID);
  EXPECT_EQ(Peer::validateRoute(*bounded, request(12.0))->status,
            Service::Response::UNAVAILABLE);

  auto stale = request();
  stale->mission_id = "old-mission";
  EXPECT_EQ(Peer::validateRoute(*clear, stale)->status,
            Service::Response::UNAVAILABLE);
  stale = request();
  stale->path[0].position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(Peer::validateRoute(*clear, stale)->status,
            Service::Response::UNAVAILABLE);

  auto component = make();
  Peer::setValidationComponent(*component, "live-component");
  auto old_component = request();
  EXPECT_EQ(Peer::validateRoute(*component, old_component)->status,
            Service::Response::UNAVAILABLE);
  old_component->component_id = "live-component";
  EXPECT_EQ(Peer::validateRoute(*component, old_component)->status,
            Service::Response::VALID);

  auto stationary = make();
  auto stopped = request();
  stopped->path.resize(2);
  stopped->path[1] = stopped->path[0];
  Peer::addOccupiedVoxel(*stationary, 0.0, 0.0, 0.30);
  EXPECT_EQ(Peer::validateRoute(*stationary, stopped)->status,
            Service::Response::INVALID);

  // Qualified MOLA simulation validates an any-yaw body with the exact
  // circumscribed circle. A voxel under a diagonal-square-only corner is
  // outside that envelope, while a voxel inside the circle remains a veto.
  auto outside_circle = make(0.05);
  Peer::setPlanningBody(*outside_circle,
                        Eigen::Vector3d(0.40, 0.20, 0.15));
  Peer::qualifyMolaPolicyForTest(*outside_circle);
  Peer::addOccupiedVoxel(*outside_circle, 0.23, 0.23, 0.30);
  EXPECT_EQ(Peer::validateRoute(*outside_circle, stopped)->status,
            Service::Response::VALID);

  auto legacy_square_corner = make(0.05);
  Peer::setPlanningBody(*legacy_square_corner,
                        Eigen::Vector3d(0.40, 0.20, 0.15));
  Peer::addOccupiedVoxel(*legacy_square_corner, 0.23, 0.23, 0.30);
  EXPECT_EQ(Peer::validateRoute(*legacy_square_corner, stopped)->status,
            Service::Response::INVALID);

  // Merely selecting the MOLA backend never opts an aerial platform into the
  // ground-only circular envelope or exact-height terrain path.
  auto aerial_mola = make(0.05);
  Peer::configureAerialBackboneTest(*aerial_mola);
  Peer::setPlanningBody(*aerial_mola,
                        Eigen::Vector3d(0.40, 0.20, 0.15));
  Peer::qualifyMolaPolicyForTest(*aerial_mola);
  Peer::addOccupiedVoxel(*aerial_mola, 0.23, 0.23, 0.30);
  EXPECT_EQ(Peer::validateRoute(*aerial_mola, stopped)->status,
            Service::Response::INVALID);

  auto inside_circle = make(0.05);
  Peer::setPlanningBody(*inside_circle,
                        Eigen::Vector3d(0.40, 0.20, 0.15));
  Peer::qualifyMolaPolicyForTest(*inside_circle);
  Peer::addOccupiedVoxel(*inside_circle, 0.18, 0.0, 0.30);
  EXPECT_EQ(Peer::validateRoute(*inside_circle, stopped)->status,
            Service::Response::INVALID);

  auto moving_outside_circle = make(0.05);
  Peer::setPlanningBody(*moving_outside_circle,
                        Eigen::Vector3d(0.40, 0.20, 0.15));
  Peer::qualifyMolaPolicyForTest(*moving_outside_circle);
  Peer::addOccupiedVoxel(*moving_outside_circle, 3.23, 0.23, 0.30);
  EXPECT_EQ(Peer::validateRoute(*moving_outside_circle, request())->status,
            Service::Response::VALID);

  auto moving_inside_circle = make(0.05);
  Peer::setPlanningBody(*moving_inside_circle,
                        Eigen::Vector3d(0.40, 0.20, 0.15));
  Peer::qualifyMolaPolicyForTest(*moving_inside_circle);
  Peer::addOccupiedVoxel(*moving_inside_circle, 3.18, 0.0, 0.30);
  EXPECT_EQ(Peer::validateRoute(*moving_inside_circle, request())->status,
            Service::Response::INVALID);

  // The indexed route keeps its submitted heights even when the mapped floor
  // would project its physical start lower. Validation still checks terrain,
  // steps, swept occupancy, and geofences at those exact route heights.
  auto height_disagreement = make(0.05);
  Peer::setPlanningBody(*height_disagreement,
                        Eigen::Vector3d(0.40, 0.20, 0.15));
  Peer::setMaxStep(*height_disagreement, 0.15);
  Peer::setMaxGroundHeight(*height_disagreement, 0.075);
  Peer::qualifyMolaPolicyForTest(*height_disagreement);
  Peer::acceptOdometry(*height_disagreement, 0.0, 0.0, 0.175);
  Peer::observeGroundRectangleAt(*height_disagreement, -0.2, 0.2, -0.3, 0.3,
                                 0.0);
  Peer::observeGroundRectangleAt(*height_disagreement, 0.3, 1.3, -0.3, 0.3,
                                 0.0);
  Peer::observeFreeBodyBox(*height_disagreement,
                           Eigen::Vector3d(0.5, 0.0, 0.55),
                           Eigen::Vector3d(2.0, 1.0, 0.30));
  // The observed lower floor itself is below the submitted body's lower face,
  // but intersects the body if the route is reprojected down at its start.
  Peer::finishMapRevision(*height_disagreement);
  auto raised_start = std::make_shared<Service::Request>();
  raised_start->mission_id = "mission-test";
  raised_start->component_id = "world";
  raised_start->frame_id = "world";
  raised_start->lookahead_m = 1.0;
  for (const double x : {0.0, 0.5, 1.0}) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.z = 0.175;
    pose.orientation.w = 1.0;
    raised_start->path.push_back(pose);
  }
  Peer::clearMolaPolicyForTest(*height_disagreement);
  const auto reprojected_height_validation =
      Peer::validateRoute(*height_disagreement, raised_start);
  EXPECT_EQ(reprojected_height_validation->status, Service::Response::INVALID)
      << reprojected_height_validation->reason;
  Peer::qualifyMolaPolicyForTest(*height_disagreement);
  const auto height_validation = Peer::validateRoute(*height_disagreement,
                                                     raised_start);
  EXPECT_EQ(height_validation->status, Service::Response::VALID)
      << height_validation->reason;

  auto known_beats_unavailable = make();
  Peer::addOccupiedVoxel(*known_beats_unavailable, 0.0, 0.0, 0.30);
  Peer::setMaxStep(*known_beats_unavailable,
                   std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(Peer::validateRoute(*known_beats_unavailable, stopped)->status,
            Service::Response::INVALID);

  auto invalid_query = make();
  Peer::setPlanningBody(*invalid_query, Eigen::Vector3d(100.0, 100.0, 0.15));
  EXPECT_EQ(Peer::validateRoute(*invalid_query, stopped)->status,
            Service::Response::UNAVAILABLE);

  auto fenced_gap = make();
  Peer::setGridGeofence(*fenced_gap, 1.34, 1.36, -0.02, 0.02);
  EXPECT_EQ(Peer::validateRoute(*fenced_gap, request())->status,
            Service::Response::INVALID);

  auto dense = request();
  dense->path.clear();
  for (int i = 0; i < 60; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.05 * i;
    pose.position.z = 0.075;
    pose.orientation.w = 1.0;
    dense->path.push_back(pose);
  }
  EXPECT_EQ(Peer::validateRoute(*clear, dense)->status,
            Service::Response::VALID);

  auto excessive = request();
  excessive->path.clear();
  for (int i = 0; i < 160; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = i == 0 ? 0.0 : 0.20 + 0.015 * (i - 1);
    pose.position.z = 0.075;
    pose.orientation.w = 1.0;
    excessive->path.push_back(pose);
  }
  EXPECT_EQ(Peer::validateRoute(*clear, excessive)->status,
            Service::Response::UNAVAILABLE);
  EXPECT_NE(Peer::validateRoute(*clear, excessive)->reason.find("work bound"),
            std::string::npos);

  auto ambiguous = request();
  ambiguous->path.clear();
  for (const auto& xy : std::vector<Eigen::Vector2d>{{0.0, 0.0}, {2.0, 0.0},
                                                     {2.0, 0.10}, {0.0, 0.10},
                                                     {-1.0, 0.10}}) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = xy.x();
    pose.position.y = xy.y();
    pose.position.z = 0.075;
    pose.orientation.w = 1.0;
    ambiguous->path.push_back(pose);
  }
  EXPECT_EQ(Peer::validateRoute(*clear, ambiguous)->status,
            Service::Response::UNAVAILABLE);
}

TEST(PlannerObjective, ObservedGroundAllowsUnknownAirButNotWallOrMissingGround) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  auto make = [](const std::string& policy, double resolution = 0.05) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true),
         rclcpp::Parameter("map.resolution", resolution),
         rclcpp::Parameter("objective_body_evidence_policy", policy)});
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureBackboneTest(*node);
    Peer::acceptOdometry(*node, 0.0, 0.0, 0.075);
    Peer::observeGroundRectangle(*node, -0.3, 2.3, -0.3, 0.3);
    Peer::finishMapRevision(*node);
    return node;
  };
  auto corridor = [](double goal_x) {
    mgg::RouteCorridor route;
    route.status = mgg::PlanningStatus::kSucceeded;
    route.request.objective = mgg::ObjectiveKind::kNavigate;
    route.request.goal.pose = mgg::StateVec(goal_x, 0.0, 0.075, 0.4);
    return route;
  };

  auto strict = make("strict_volume");
  EXPECT_EQ(Peer::refine(*strict, corridor(2.0)).status,
            mgg::PlanningStatus::kBlocked);

  auto observed = make("observed_ground");
  const mgg::FeasiblePath clear = Peer::refine(*observed, corridor(2.0));
  ASSERT_EQ(clear.status, mgg::PlanningStatus::kSucceeded) << clear.reason;
  EXPECT_NEAR(clear.poses.back().x(), 2.0, 1e-9);
  EXPECT_FALSE(clear.indexed_map_validated);

  Peer::addOccupiedVoxel(*observed, 1.0, 0.0, 0.30);
  EXPECT_EQ(Peer::refine(*observed, corridor(2.0)).status,
            mgg::PlanningStatus::kBlocked);

  auto unsupported = make("observed_ground");
  EXPECT_EQ(Peer::refine(*unsupported, corridor(4.0)).status,
            mgg::PlanningStatus::kBlocked);

  // At 15 cm resolution the legacy AND rule sees a 15 cm kerb across a
  // 30 cm edge sample as a 26.6 degree ramp and permits it under a 30 degree
  // inclination cap. The objective terrain contract retains the 10 cm step
  // bound independently.
  auto stepped = make("observed_ground", 0.15);
  Peer::addRaisedFloor(*stepped, 0.15);
  EXPECT_EQ(Peer::refine(*stepped, corridor(2.0)).status,
            mgg::PlanningStatus::kBlocked);

  auto fenced = make("observed_ground");
  Peer::setGridGeofence(*fenced, -0.5, 0.5, -0.5, 0.5);
  EXPECT_EQ(Peer::refine(*fenced, corridor(2.0)).status,
            mgg::PlanningStatus::kBlocked);
}

TEST(PlannerObjective, ObservedGroundBlindStartConnectorRetainsDistanceBound) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  auto make = [](double support_x) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true),
         rclcpp::Parameter("map.resolution", 0.05),
         rclcpp::Parameter("objective_body_evidence_policy", "observed_ground")});
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureBackboneTest(*node);
    Peer::acceptOdometry(*node, 0.0, 0.0, 0.075);
    Peer::observeDestinationSupport(*node, support_x);
    return node;
  };
  auto corridor = [](double goal_x) {
    mgg::RouteCorridor route;
    route.status = mgg::PlanningStatus::kSucceeded;
    route.request.objective = mgg::ObjectiveKind::kNavigate;
    route.request.goal.pose = mgg::StateVec(goal_x, 0.0, 0.075, 0.0);
    return route;
  };

  auto within = make(2.5);
  const mgg::FeasiblePath connected = Peer::refine(*within, corridor(2.5));
  ASSERT_EQ(connected.status, mgg::PlanningStatus::kSucceeded)
      << connected.reason;
  EXPECT_NEAR(connected.poses.back().x(), 2.5, 1e-9);

  auto curb = make(2.5);
  // The kerb top remains below the raised body box, isolating the terrain
  // footprint veto from the ordinary occupied-body check.
  Peer::addMeasuredSurface(*curb, 1.25, 0.10, 0.125);
  const mgg::FeasiblePath curb_refused = Peer::refine(*curb, corridor(2.5));
  EXPECT_EQ(curb_refused.status, mgg::PlanningStatus::kBlocked);
  EXPECT_NE(curb_refused.reason.find("first footprint rejection: known rise"),
            std::string::npos)
      << curb_refused.reason;
  EXPECT_NE(curb_refused.reason.find("exceeds step limit 0.100 m"),
            std::string::npos)
      << curb_refused.reason;

  auto beyond = make(3.25);
  const mgg::FeasiblePath refused = Peer::refine(*beyond, corridor(3.25));
  EXPECT_EQ(refused.status, mgg::PlanningStatus::kBlocked);
}

TEST(PlannerObjective, PhysicalCurrentMayEscapeRejectedTerrainButCannotCrossIt) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  const auto make = [] {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true),
         rclcpp::Parameter("map.resolution", 0.05),
         rclcpp::Parameter("objective_body_evidence_policy", "observed_ground")});
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureBackboneTest(*node);
    Peer::setPlanningBody(*node, Eigen::Vector3d(0.40, 0.20, 0.15));
    Peer::acceptOdometry(*node, 0.0, 0.0, 0.075);
    Peer::observeGroundRectangle(*node, -1.5, 1.5, -1.0, 1.0);
    Peer::observeFreeBodyBox(*node, Eigen::Vector3d(0.0, 0.0, 0.40),
                            Eigen::Vector3d(3.2, 2.2, 0.20));
    // Pin the centre ray and forward connector to the flat floor explicitly.
    // The rectangle helper's repeated 0.1 additions can represent y=0 just
    // below a voxel boundary, making projection fall back to the rear probe
    // that this fixture intentionally raises.
    for (double x = 0.0; x <= 1.0 + 1e-9; x += 0.05) {
      Peer::addMeasuredSurface(*node, x, 0.0, 0.0);
    }
    // A raised terrain return lies under the rear edge of the stationary
    // footprint, while the forward edge immediately leaves it behind.
    Peer::addMeasuredSurface(*node, -0.24, 0.0, 0.125);
    Peer::finishMapRevision(*node);
    return node;
  };
  const auto corridor = [](double x, double y = 0.0) {
    mgg::RouteCorridor route;
    route.status = mgg::PlanningStatus::kSucceeded;
    route.request.objective = mgg::ObjectiveKind::kNavigate;
    route.request.goal.pose = mgg::StateVec(x, y, 0.075, 0.0);
    return route;
  };

  auto away = make();
  ASSERT_FALSE(Peer::footprintTerrainSupported(
      *away, Eigen::Vector3d(0.0, 0.0, 0.30),
      Eigen::Vector3d(0.40, 0.20, 0.15)));
  const mgg::FeasiblePath escaped = Peer::refine(*away, corridor(1.0));
  ASSERT_EQ(escaped.status, mgg::PlanningStatus::kSucceeded) << escaped.reason;
  EXPECT_NEAR(escaped.poses.back().x(), 1.0, 1e-9);

  auto across = make();
  for (double y = -1.0; y <= 1.0 + 1e-9; y += 0.05) {
    Peer::addMeasuredSurface(*across, -0.30, y, 0.125);
  }
  const mgg::FeasiblePath refused_crossing =
      Peer::refine(*across, corridor(-1.0));
  EXPECT_EQ(refused_crossing.status, mgg::PlanningStatus::kBlocked)
      << refused_crossing.reason;

  auto endpoint = make();
  Peer::addMeasuredSurface(*endpoint, 0.80, 0.0, 0.125);
  const mgg::FeasiblePath refused_endpoint =
      Peer::refine(*endpoint, corridor(0.80));
  EXPECT_EQ(refused_endpoint.status, mgg::PlanningStatus::kBlocked)
      << refused_endpoint.reason;
}

TEST(PlannerObjective,
     PlannedPhysicalCurrentFootprintExceptionSurvivesImmediateValidation) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  using Validation = mgg_msgs::srv::ValidateObjectiveRoute;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("mission_id", "physical-current-validation"),
       rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy",
                         "provisional_unknown")});
  auto node = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBackboneTest(*node);
  Peer::setPlanningBody(*node, Eigen::Vector3d(0.40, 0.20, 0.15));
  Peer::acceptOdometry(*node, 0.0, 0.0, 0.075);
  Peer::observeGroundRectangle(*node, -1.5, 1.5, -1.0, 1.0);
  Peer::observeFreeBodyBox(*node, Eigen::Vector3d(0.0, 0.0, 0.40),
                           Eigen::Vector3d(3.2, 2.2, 0.20));
  for (double x = 0.0; x <= 1.0 + 1e-9; x += 0.05) {
    Peer::addMeasuredSurface(*node, x, 0.0, 0.0);
  }
  Peer::addMeasuredSurface(*node, -0.24, 0.0, 0.125);
  Peer::finishMapRevision(*node);
  ASSERT_FALSE(Peer::footprintTerrainSupported(
      *node, Eigen::Vector3d(0.0, 0.0, 0.30),
      Eigen::Vector3d(0.40, 0.20, 0.15)));

  const auto planned =
      Peer::requestBoundNavigate(*node, mgg::StateVec(1.0, 0.0, 0.075, 0.0));
  ASSERT_EQ(planned->status,
            mgg_msgs::srv::PlanObjective::Response::SUCCEEDED)
      << planned->reason;
  ASSERT_GE(planned->path.size(), 2u);

  auto validation = std::make_shared<Validation::Request>();
  validation->mission_id = "physical-current-validation";
  validation->component_id = planned->component_id;
  validation->frame_id = "world";
  validation->lookahead_m = 2.0;
  validation->path = planned->path;
  const auto immediate = Peer::validateRoute(*node, validation);
  ASSERT_EQ(immediate->status, Validation::Response::VALID)
      << immediate->reason;
  EXPECT_EQ(immediate->map_revision, planned->map_revision);

  // The physical-pose exception applies only to the first remaining sample.
  // A lateral rise under the same route's later footprint remains a veto.
  Peer::addMeasuredSurface(*node, 0.75, 0.10, 0.125);
  const auto later_hazard = Peer::validateRoute(*node, validation);
  EXPECT_EQ(later_hazard->status, Validation::Response::INVALID)
      << later_hazard->reason;
  EXPECT_NE(later_hazard->reason.find("footprint"), std::string::npos)
      << later_hazard->reason;
  EXPECT_GT(later_hazard->map_revision, immediate->map_revision);
}

TEST(PlannerObjective, ObservedGroundVetoesKnownFootprintTerrainHazards) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  const auto make = [] {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true),
         rclcpp::Parameter("map.resolution", 0.05),
         rclcpp::Parameter("objective_body_evidence_policy", "observed_ground")});
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureBackboneTest(*node);
    return node;
  };
  const Eigen::Vector3d body(0.40, 0.20, 0.15);
  const Eigen::Vector3d driving_pose(0.0, 0.0, 0.30);

  auto flat = make();
  Peer::observeGroundRectangle(*flat, -0.3, 0.3, -0.3, 0.3);
  EXPECT_TRUE(Peer::footprintTerrainSupported(*flat, driving_pose, body));

  // This obstacle is outside the narrow body's centreline but under a corner
  // of the body at some yaw. The circumscribed footprint must see it.
  Peer::addMeasuredSurface(*flat, 0.18, 0.08, 0.125);
  EXPECT_FALSE(Peer::footprintTerrainSupported(*flat, driving_pose, body));
  EXPECT_FALSE(Peer::terrainPathSupported(
      *flat, {driving_pose, Eigen::Vector3d(0.10, 0.0, 0.30)}));

  auto square_corner = make();
  Peer::observeGroundRectangle(*square_corner, -0.3, 0.3, -0.3, 0.3);
  // The sampling lattice encloses the circumscribed footprint circle.  A
  // kerb in a lattice corner is farther from the robot than that circle and
  // cannot veto an otherwise supported pose.
  Peer::addMeasuredSurface(*square_corner, 0.25, 0.25, 0.125);
  EXPECT_TRUE(Peer::footprintTerrainSupported(*square_corner, driving_pose,
                                              body));

  auto missing_corner = make();
  Peer::observeGroundRectangle(*missing_corner, -0.05, 0.05, -0.05, 0.05);
  // Sparse lateral evidence is neutral; ordinary objective projection still
  // requires known support on the centreline.
  EXPECT_TRUE(
      Peer::footprintTerrainSupported(*missing_corner, driving_pose, body));

  auto unknown_transition = make();
  EXPECT_FALSE(Peer::terrainPathSupported(
      *unknown_transition,
      {driving_pose, Eigen::Vector3d(0.30, 0.0, 0.425)}));

  // A fully observed shallow surface remains within the configured 10 cm
  // step budget across the complete footprint.
  auto shallow = make();
  Peer::observeShallowRamp(*shallow);
  EXPECT_TRUE(Peer::footprintTerrainSupported(
      *shallow, Eigen::Vector3d(0.0, 0.0, 0.325), body));

  EXPECT_FALSE(Peer::footprintTerrainSupported(
      *shallow, driving_pose, Eigen::Vector3d(10.0, 10.0, 0.15)));
}

TEST(PlannerObjective, ConnectedMeasuredSupportAllowsStagedFootprintTerrain) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  const Eigen::Vector3d body(0.90, 0.15, 0.15);
  const Eigen::Vector3d low_pose(0.0, 0.0, 0.30);
  const auto make = [&body] {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true),
         rclcpp::Parameter("map.resolution", 0.15),
         rclcpp::Parameter("objective_body_evidence_policy", "observed_ground")});
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureBackboneTest(*node);
    Peer::setPlanningBody(*node, body);
    return node;
  };
  const auto add_column = [](mgg_ros::PlannerNode& node, double x,
                             double z) {
    // Samples lie in both map-cell rows touched by y=0.  The 15 cm phase
    // matches the measured-surface cells used by the Bistro simulation.
    Peer::addMeasuredSurface(node, x, -0.075, z);
    Peer::addMeasuredSurface(node, x, 0.075, z);
  };

  auto staged = make();
  for (const double x : {-0.525, -0.375, -0.225}) {
    add_column(*staged, x, 0.0);
  }
  add_column(*staged, -0.075, 0.068);
  for (const double x : {0.075, 0.225, 0.375, 0.525}) {
    add_column(*staged, x, 0.143);
  }
  // The complete footprint spans 14.3 cm relative to the low centre, but
  // each measured transition is inside the 10 cm step budget.
  EXPECT_TRUE(Peer::footprintTerrainSupported(*staged, low_pose, body));
  EXPECT_TRUE(Peer::terrainPathSupported(
      *staged, {Eigen::Vector3d(-0.30, 0.0, 0.30),
                Eigen::Vector3d(0.0, 0.0, 0.368),
                Eigen::Vector3d(0.30, 0.0, 0.443)}));

  auto invalid_inclination = make();
  for (const double x : {-0.525, -0.375, -0.225}) {
    add_column(*invalid_inclination, x, 0.0);
  }
  add_column(*invalid_inclination, -0.075, 0.068);
  for (const double x : {0.075, 0.225, 0.375, 0.525}) {
    add_column(*invalid_inclination, x, 0.143);
  }
  Peer::setMaxInclination(*invalid_inclination,
                          std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(Peer::footprintTerrainSupported(*invalid_inclination,
                                               low_pose, body));

  auto grade = make();
  const std::vector<double> grade_x = {
      -0.525, -0.375, -0.225, -0.075, 0.075, 0.225, 0.375, 0.525};
  for (std::size_t i = 0; i < grade_x.size(); ++i) {
    add_column(*grade, grade_x[i], 0.025 * static_cast<double>(i));
  }
  // A continuous measured grade may exceed one centre-to-edge step while
  // each adjacent cell remains below both the step and inclination limits.
  EXPECT_TRUE(Peer::footprintTerrainSupported(*grade, low_pose, body));

  auto curb = make();
  for (const double x : {-0.525, -0.375, -0.225, -0.075}) {
    add_column(*curb, x, 0.0);
  }
  for (const double x : {0.075, 0.225, 0.375, 0.525}) {
    add_column(*curb, x, 0.16);
  }
  // A fully measured single 16 cm curb has no admissible support edge and
  // retains the known-terrain veto.
  EXPECT_FALSE(Peer::footprintTerrainSupported(*curb, low_pose, body));
  EXPECT_FALSE(Peer::terrainPathSupported(
      *curb, {Eigen::Vector3d(-0.30, 0.0, 0.30),
              Eigen::Vector3d(0.30, 0.0, 0.46)}));

  auto missing_middle = make();
  for (const double x : {-0.525, -0.375}) {
    add_column(*missing_middle, x, 0.0);
  }
  for (const double x : {0.375, 0.525}) {
    add_column(*missing_middle, x, 0.143);
  }
  // Absent ground observations remain neutral for the footprint query, but
  // they cannot bridge known support to an otherwise unsupported outlier.
  EXPECT_FALSE(
      Peer::footprintTerrainSupported(*missing_middle, low_pose, body));
}

TEST(PlannerObjective, FootprintQueriesEachIntersectingMapCellExactlyOnce) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  const auto make = [] {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true),
         rclcpp::Parameter("map.resolution", 0.15),
         rclcpp::Parameter("objective_body_evidence_policy", "observed_ground")});
    auto node = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureBackboneTest(*node);
    return node;
  };
  const Eigen::Vector3d scout_body(0.662, 0.630, 0.15);
  const Eigen::Vector3d home(-0.01, -0.08, 0.30);

  auto outside = make();
  Peer::observeGroundRectangle(*outside, -0.5, 0.5, -0.5, 0.5);
  // The old radius-plus-half-diagonal ray lattice selected the cell holding
  // this Bistro curb even though that cell does not touch the physical circle.
  Peer::addMeasuredSurface(*outside, -0.15, -0.75, 0.145);
  EXPECT_TRUE(Peer::footprintTerrainSupported(*outside, home, scout_body));

  auto touching = make();
  Peer::observeGroundRectangle(*touching, -0.5, 0.5, -0.5, 0.5);
  // This cell's closed AABB intersects the physical footprint and must retain
  // the known-terrain veto.
  Peer::addMeasuredSurface(*touching, -0.15, -0.45, 0.145);
  EXPECT_FALSE(Peer::footprintTerrainSupported(*touching, home, scout_body));

  auto tangent = make();
  Peer::observeGroundRectangle(*tangent, -0.3, 0.3, -0.3, 0.3);
  // Radius zero at the x=0/y=0 grid boundary touches all adjoining closed
  // cells. A hazard in one of those cells cannot disappear by grid phase.
  Peer::addMeasuredSurface(*tangent, 0.01, 0.01, 0.145);
  EXPECT_FALSE(Peer::footprintTerrainSupported(
      *tangent, Eigen::Vector3d(0.0, 0.0, 0.30),
      Eigen::Vector3d(0.0, 0.0, 0.15)));

  const Eigen::Vector3d tangent_body(0.30, 0.0, 0.15);
  for (const double tangent_x : {-0.525, -0.075}) {
    auto nonzero_tangent = make();
    Peer::observeGroundRectangle(*nonzero_tangent, -0.7, 0.1, -0.7, 0.1);
    // At the negative-coordinate grid boundary, cells centred on either side
    // touch the nonzero-radius circle at exactly one edge.
    Peer::addMeasuredSurface(*nonzero_tangent, tangent_x, -0.30, 0.145);
    EXPECT_FALSE(Peer::footprintTerrainSupported(
        *nonzero_tangent, Eigen::Vector3d(-0.30, -0.30, 0.30),
        tangent_body));
  }
}

TEST(PlannerBackbone, CapturesHomeBeforeMotionAndConnectsOnlyMappedTerrain) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  mgg_ros::PlannerNodeTestPeer::configureBackboneTest(*planner);

  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
  ASSERT_EQ(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 1);
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 0);
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::initialAnchorSupported(*planner));
  const mgg::StateVec provisional =
      mgg_ros::PlannerNodeTestPeer::globalVertexState(*planner, 0);
  EXPECT_NEAR(provisional.x(), 0.0, 1e-9);
  EXPECT_NEAR(provisional.y(), 0.0, 1e-9);

  // Motion through unknown space cannot turn the landmark into a free edge.
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.60, 0.0, 0.075);
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 1);
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 0);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 1.20, 0.0, 0.075);
  mgg::StateVec authority_home = mgg::StateVec::Zero();
  authority_home.z() = 0.075;
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::planHome(*planner, authority_home).status,
            mgg::PlanningStatus::kUnreachable);

  // Ground support refines the anchor, but unknown body clearance still may
  // not create a global edge.
  mgg_ros::PlannerNodeTestPeer::observeGroundSupport(*planner);
  ASSERT_TRUE(mgg_ros::PlannerNodeTestPeer::initialAnchorSupported(*planner));
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 1);
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 0);

  // Once the body corridor is observed free, both retained chronological
  // breadcrumbs connect instead of replacing the travelled segment by one
  // long edge.
  mgg_ros::PlannerNodeTestPeer::observeBodyCorridor(*planner);
  mgg::StateVec observed_here = mgg::StateVec::Zero();
  observed_here.x() = 1.20;
  observed_here.z() = 0.075;
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdgeStatus(
                *planner, authority_home, observed_here),
            mgg::ProjectedEdgeStatus::kAdmissible);
  ASSERT_EQ(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 3);
  ASSERT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 2);
  const mgg::StateVec home =
      mgg_ros::PlannerNodeTestPeer::globalVertexState(*planner, 0);
  EXPECT_NEAR(home.x(), 0.0, 0.11);
  EXPECT_NEAR(home.y(), 0.0, 0.11);
  EXPECT_NEAR(home.z(), 0.30, 0.11);

  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 1.80, 0.0, 0.075);
  ASSERT_EQ(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 4);
  ASSERT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 3);

  // Authority home is a base pose, while graph vertices sit at driving
  // height.  The normal goal tolerance still selects the initial anchor.
  const mgg::RouteCorridor route =
      mgg_ros::PlannerNodeTestPeer::planHome(*planner, authority_home);
  EXPECT_EQ(route.status, mgg::PlanningStatus::kSucceeded) << route.reason;
  ASSERT_GE(route.poses.size(), 2u);
  EXPECT_NEAR(route.poses.back().x(), home.x(), 1e-9);
  EXPECT_NEAR(route.poses.back().y(), home.y(), 1e-9);
  const mgg::FeasiblePath feasible =
      mgg_ros::PlannerNodeTestPeer::refine(*planner, route);
  ASSERT_EQ(feasible.status, mgg::PlanningStatus::kSucceeded)
      << feasible.reason;
  ASSERT_FALSE(feasible.poses.empty());
  for (const auto& pose : feasible.poses) {
    EXPECT_NEAR(pose.z(), 0.075, 0.06);
  }
  EXPECT_NEAR(feasible.poses.back().z(), authority_home.z(), 0.06);

  // The Explore response boundary uses the same graph-to-base conversion.
  mgg::RouteCorridor explore = route;
  explore.request.objective = mgg::ObjectiveKind::kExplore;
  const mgg::FeasiblePath explore_response =
      mgg_ros::PlannerNodeTestPeer::refine(*planner, explore);
  ASSERT_EQ(explore_response.status, mgg::PlanningStatus::kSucceeded)
      << explore_response.reason;
  ASSERT_FALSE(explore_response.poses.empty());
  for (const auto& pose : explore_response.poses) {
    EXPECT_NEAR(pose.z(), 0.075, 0.06);
  }

}

TEST(PlannerBackbone, ExplicitGroundFailureIdentifiesCurrentOrGoal) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  using Peer = mgg_ros::PlannerNodeTestPeer;

  auto unsupported_current = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBackboneTest(*unsupported_current);
  Peer::acceptOdometry(*unsupported_current, 0.0, 0.0, 0.075);
  Peer::observeDestinationSupport(*unsupported_current, 2.0);
  const mgg::StateVec home(0.0, 0.0, 0.075, 0.0);
  auto response = Peer::requestObjective(
      *unsupported_current, mgg::ObjectiveKind::kReturnHome, home);
  ASSERT_NE(response, nullptr);
  EXPECT_NE(response->reason.find(
                "current pose rejected: no mapped ground support at "
                "(0.00, 0.00,"),
            std::string::npos)
      << response->reason;

  auto unsupported_goal = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBackboneTest(*unsupported_goal);
  Peer::acceptOdometry(*unsupported_goal, 0.0, 0.0, 0.075);
  Peer::observeGroundSupport(*unsupported_goal);
  const mgg::StateVec distant_goal(10.0, 0.0, 0.075, 0.0);
  response = Peer::requestObjective(
      *unsupported_goal, mgg::ObjectiveKind::kReturnHome, distant_goal);
  ASSERT_NE(response, nullptr);
  EXPECT_NE(response->reason.find(
                "goal rejected: no mapped ground support at "
                "(10.00, 0.00,"),
            std::string::npos)
      << response->reason;
}

TEST(PlannerObjective, NavigateReturnsBoundedWindowForThirtyMetreKnownRoad) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureLongKnownRoad(*planner);

  const mgg::StateVec exact_goal(30.0, 0.0, 0.075, 0.9);
  const auto response = Peer::requestObjective(
      *planner, mgg::ObjectiveKind::kNavigate, exact_goal);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  EXPECT_TRUE(response->partial);
  EXPECT_FALSE(response->route_id.empty());
  ASSERT_FALSE(response->global_path.empty());
  ASSERT_FALSE(response->path.empty());
  EXPECT_LE(response->path.back().position.x, 8.0 + 1e-3);
  EXPECT_NEAR(response->global_path.back().position.x, exact_goal.x(), 1e-3);
  EXPECT_NEAR(response->global_path.back().position.y, exact_goal.y(), 1e-3);
}

TEST(PlannerObjective, RollingNavigateKeepsFixedGoalAcrossGraphGrowth) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("objective_route_horizon_m", 3.0)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*planner);

  const mgg::StateVec exact_goal(12.0, 0.0, 0.075, 0.8);
  auto response = Peer::requestObjective(
      *planner, mgg::ObjectiveKind::kNavigate, exact_goal);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  ASSERT_TRUE(response->partial);
  ASSERT_FALSE(response->route_id.empty());
  ASSERT_FALSE(response->global_path.empty());
  EXPECT_NEAR(response->global_path.back().position.x, exact_goal.x(), 1e-9);
  EXPECT_NEAR(response->global_path.back().orientation.z, std::sin(0.4), 1e-9);
  const std::string route_id = response->route_id;
  const std::uint64_t captured_graph_revision = response->graph_revision;

  auto premature =
      std::make_shared<mgg_msgs::srv::RefineObjectiveRoute::Request>();
  premature->route_id = route_id;
  premature->component_id = response->component_id;
  premature->graph_revision = captured_graph_revision;
  EXPECT_EQ(Peer::refineObjectiveRoute(*planner, premature)->status,
            mgg_msgs::srv::RefineObjectiveRoute::Response::BLOCKED);

  bool complete = false;
  for (int chunk = 0; chunk < 8 && !complete; ++chunk) {
    ASSERT_FALSE(response->path.empty());
    const auto endpoint = response->path.back().position;
    // Normal odometry updates may grow the live graph, but the token remains
    // bound to the immutable graph route captured by the first request.
    Peer::acceptOdometry(*planner, endpoint.x, endpoint.y, endpoint.z);
    auto request =
        std::make_shared<mgg_msgs::srv::RefineObjectiveRoute::Request>();
    request->route_id = route_id;
    request->component_id = response->component_id;
    request->graph_revision = captured_graph_revision;
    const auto next = Peer::refineObjectiveRoute(*planner, request);
    ASSERT_EQ(next->status,
              mgg_msgs::srv::RefineObjectiveRoute::Response::SUCCEEDED)
        << next->reason;
    complete = !next->partial;
    response->path = next->path;
    response->partial = next->partial;
  }
  ASSERT_TRUE(complete);
  ASSERT_FALSE(response->path.empty());
  EXPECT_NEAR(response->path.back().position.x, exact_goal.x(), 1e-9);
  EXPECT_NEAR(response->path.back().orientation.z, std::sin(0.4), 1e-9);
}

TEST(PlannerObjective, RollingNavigateDoesNotValidateBeyondCurrentHorizon) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("objective_route_horizon_m", 3.0)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*planner);
  Peer::addGridObstacle(*planner, 0.325, true, 10.0);

  const auto response = Peer::requestObjective(
      *planner, mgg::ObjectiveKind::kNavigate,
      mgg::StateVec(12.0, 0.0, 0.075, 0.0));
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  EXPECT_TRUE(response->partial);
  ASSERT_FALSE(response->path.empty());
  EXPECT_LE(response->path.back().position.x, 3.0 + 1e-3);
}

TEST(PlannerObjective, NavigateResolvesStaleStartHeightAcrossLongShallowRamp) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("objective_ground_evidence_policy",
                         "provisional_unknown"),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("use_sim_time", true)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureLongKnownRamp(*planner);

  // Goal Z deliberately retains the starting base height. The endpoint's
  // locally flat ground is 30 cm higher, reached by a continuous 10% grade.
  const mgg::StateVec goal(3.0, 0.0, 0.075, 0.4);
  const auto response = Peer::requestBoundNavigate(*planner, goal);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  ASSERT_FALSE(response->path.empty());
  EXPECT_NEAR(response->path.back().position.x, goal.x(), 1e-3);
  EXPECT_NEAR(response->path.back().position.y, goal.y(), 1e-3);
  EXPECT_NEAR(response->path.back().position.z, 0.4025, 0.03);
  EXPECT_NEAR(response->path.back().orientation.z, std::sin(0.2), 1e-6);
  EXPECT_NEAR(response->path.back().orientation.w, std::cos(0.2), 1e-6);
}

TEST(PlannerObjective, ProvisionalUnknownContinuesFromLastKnownRampHeight) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("mission_id", "ramp-unknown-test"),
       rclcpp::Parameter("objective_ground_evidence_policy",
                         "provisional_unknown"),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_grid_timeout_ms", 2000),
       rclcpp::Parameter("use_sim_time", true)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureKnownRampThenUnknown(*planner);

  // The known road climbs gradually to a 12 cm plateau and observations end
  // at x=4 m. This six-metre objective fits within the final local horizon;
  // its unknown suffix must inherit that locally reached
  // driving plane instead of snapping back to the initial height.
  const mgg::StateVec goal(6.0, 0.0, 0.075, 0.6);
  const auto response = Peer::requestBoundNavigate(*planner, goal);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  ASSERT_FALSE(response->path.empty());
  EXPECT_NEAR(response->path.back().position.x, goal.x(), 1e-3);
  EXPECT_NEAR(response->path.back().position.y, goal.y(), 1e-3);
  EXPECT_NEAR(response->path.back().position.z, 0.20, 0.03);
  EXPECT_NEAR(response->path.back().orientation.z, std::sin(0.3), 1e-6);
  EXPECT_NEAR(response->path.back().orientation.w, std::cos(0.3), 1e-6);
  for (std::size_t i = 1; i < response->path.size(); ++i) {
    EXPECT_LE(response->path[i].position.z, 0.25 + 1e-6);
    EXPECT_LE(std::abs(response->path[i].position.z -
                       response->path[i - 1].position.z),
              0.10 + 1e-6);
  }

  auto validation =
      std::make_shared<mgg_msgs::srv::ValidateObjectiveRoute::Request>();
  validation->mission_id = "ramp-unknown-test";
  validation->component_id = "world";
  validation->frame_id = "world";
  validation->lookahead_m = 5.0;
  validation->path = response->path;
  const auto validated = Peer::validateRoute(*planner, validation);
  ASSERT_NE(validated, nullptr);
  EXPECT_EQ(validated->status,
            mgg_msgs::srv::ValidateObjectiveRoute::Response::VALID)
      << validated->reason;
}

TEST(PlannerObjective, NavigateHeightResolutionCannotTeleportOntoCeiling) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("objective_ground_evidence_policy",
                         "provisional_unknown"),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("use_sim_time", true)});
  using Peer = mgg_ros::PlannerNodeTestPeer;
  const auto make = [&options] {
    auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
    Peer::configureBackboneTest(*planner);
    Peer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
    Peer::observeGroundRectangleAt(*planner, -0.3, 0.3, -0.3, 0.3, 0.40);
    return planner;
  };

  auto stationary = make();
  const auto turn = Peer::requestBoundNavigate(
      *stationary, mgg::StateVec(0.0, 0.0, 0.0, 0.8));
  ASSERT_NE(turn, nullptr);
  ASSERT_EQ(turn->status, Service::Response::SUCCEEDED) << turn->reason;
  ASSERT_EQ(turn->path.size(), 1u);
  EXPECT_NEAR(turn->path.front().position.z, 0.075, 1e-6);
  EXPECT_NEAR(turn->path.front().orientation.z, std::sin(0.4), 1e-6);
  EXPECT_NEAR(turn->path.front().orientation.w, std::cos(0.4), 1e-6);

  auto near = make();
  const auto cannot_climb = Peer::requestBoundNavigate(
      *near, mgg::StateVec(0.01, 0.0, 0.0, 0.0));
  ASSERT_NE(cannot_climb, nullptr);
  EXPECT_EQ(cannot_climb->status, Service::Response::BLOCKED)
      << cannot_climb->reason;
  EXPECT_TRUE(cannot_climb->path.empty());
}

TEST(PlannerObjective, NavigateUsesHelpfulGraphBeforeUnknownExactSuffix) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("objective_ground_evidence_policy",
                         "provisional_unknown"),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("use_sim_time", true)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureNavigateGraphCorridor(
      *planner, {mgg::StateVec(0.0, 0.0, 0.30, 0.0),
                 mgg::StateVec(0.0, 0.60, 0.30, 0.0),
                 mgg::StateVec(1.20, 0.60, 0.30, 0.0)});

  const mgg::StateVec exact_goal(3.0, 0.0, 0.075, 0.8);
  const auto response = Peer::requestBoundNavigate(*planner, exact_goal);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  ASSERT_FALSE(response->path.empty());
  EXPECT_TRUE(std::any_of(response->path.begin(), response->path.end(),
                          [](const auto& pose) {
                            return pose.position.y > 0.50;
                          }));
  EXPECT_NEAR(response->path.back().position.x, exact_goal.x(), 1e-3);
  EXPECT_NEAR(response->path.back().position.y, exact_goal.y(), 1e-3);
}

TEST(PlannerObjective, NavigateFallsBackToDirectWhenGraphCorridorIsBlocked) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureNavigateGraphCorridor(
      *planner, {mgg::StateVec(0.0, 0.0, 0.30, 0.0),
                 mgg::StateVec(0.60, 0.80, 0.30, 0.0),
                 mgg::StateVec(1.20, 0.0, 0.30, 0.0)});
  Peer::addOccupiedVoxel(*planner, 0.60, 0.80, 0.30);

  const mgg::StateVec exact_goal(1.20, 0.0, 0.075, 0.0);
  const auto response = Peer::requestBoundNavigate(*planner, exact_goal);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  ASSERT_FALSE(response->path.empty());
  EXPECT_TRUE(std::all_of(response->path.begin(), response->path.end(),
                         [](const auto& pose) {
                           return std::abs(pose.position.y) < 0.20;
                         }));
  EXPECT_NEAR(response->path.back().position.x, exact_goal.x(), 1e-3);
  EXPECT_NEAR(response->path.back().position.y, exact_goal.y(), 1e-3);
}

TEST(PlannerObjective, FarNavigateRejectsUnknownInsteadOfReturningProxy) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("partial_route_min_progress_m", 1.0)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configurePartialNavigateScene(*planner);

  const mgg::StateVec exact_goal(30.0, 4.0, 0.075, 1.1);
  const auto response = Peer::requestBoundNavigate(*planner, exact_goal);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED)
      << response->reason;
  EXPECT_FALSE(response->partial);
  EXPECT_FALSE(response->indexed_map_validated);
  EXPECT_TRUE(response->path.empty());
  EXPECT_EQ(response->graph_revision, Peer::globalGraphRevision(*planner));
  EXPECT_GT(response->map_revision, 0u);
}

TEST(PlannerObjective, FlatBlindStartConnectsWithoutMovingPhysicalAnchor) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureBlindStartScene(*planner);
  const auto response = Peer::requestBoundNavigate(
      *planner, mgg::StateVec(1.20, 0.0, 0.075, 0.0));
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_NEAR(response->path.front().position.x, 0.0, 1e-9);
  EXPECT_NEAR(response->path.front().position.z, 0.075, 1e-9);
  EXPECT_NEAR(response->path.back().position.x, 1.20, 0.06);
}

TEST(PlannerObjective, BlindStartBuilderReachesSupportPastOrdinaryEdgeLimit) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureBlindStartScene(*planner);
  const std::string summary = Peer::rebuildBlindStartLocalGraph(*planner);
  SCOPED_TRACE(summary);
  EXPECT_GT(Peer::localGraphRevision(*planner), 41u);
  const auto response = Peer::requestBoundNavigate(
      *planner, mgg::StateVec(1.20, 0.0, 0.075, 0.0));
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
}

TEST(PlannerObjective, FinitePhysicalRootRetainsBoundedBlindStartEligibility) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true),
       rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("objective_body_evidence_policy", "observed_ground"),
       rclcpp::Parameter("objective_ground_evidence_policy",
                         "provisional_unknown")});
  using Peer = mgg_ros::PlannerNodeTestPeer;

  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBlindStartScene(*planner);
  // An isolated measured patch at the physical root must not hide the real
  // near-field floor gap before the first supported destination.
  Peer::addBlindStartKnownFloor(*planner, 0.0, 0.0);
  const std::string summary = Peer::rebuildBlindStartLocalGraph(*planner);
  EXPECT_GT(Peer::localVertices(*planner), 1u) << summary;

  auto blocked = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBlindStartScene(*blocked);
  Peer::addBlindStartKnownFloor(*blocked, 0.0, 0.0);
  Peer::addGridObstacle(*blocked, 0.325, true, 0.60);
  const std::string blocked_summary =
      Peer::rebuildBlindStartLocalGraph(*blocked);
  EXPECT_EQ(Peer::localVertices(*blocked), 1u) << blocked_summary;
}

TEST(PlannerObjective, BlindStartRefusesWallUnknownStepAndObservedDrop) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  using Peer = mgg_ros::PlannerNodeTestPeer;
  const auto goal = mgg::StateVec(1.20, 0.0, 0.075, 0.0);

  auto wall = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBlindStartScene(*wall);
  Peer::addGridObstacle(*wall, 0.325, true, 0.60);
  Peer::syncLocalGraphMapRevision(*wall);
  EXPECT_EQ(Peer::requestBoundNavigate(*wall, goal)->status,
            Service::Response::BLOCKED);

  auto unknown = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBlindStartScene(*unknown);
  ASSERT_TRUE(Peer::makeGridVoxelUnknown(*unknown, 0.60, 0.025, 0.325));
  Peer::syncLocalGraphMapRevision(*unknown);
  EXPECT_EQ(Peer::requestBoundNavigate(*unknown, goal)->status,
            Service::Response::BLOCKED);

  auto step = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBlindStartScene(*step, 0.15);
  EXPECT_EQ(Peer::requestBoundNavigate(
                *step, mgg::StateVec(1.20, 0.0, 0.225, 0.0))
                ->status,
            Service::Response::BLOCKED);

  auto drop = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureBlindStartScene(*drop);
  Peer::addBlindStartKnownFloor(*drop, 0.60, -0.15);
  EXPECT_EQ(Peer::requestBoundNavigate(*drop, goal)->status,
            Service::Response::BLOCKED);
}

TEST(PlannerObjective, BlindStartHomeReturnsToLatchedPhysicalAnchor) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureBlindStartScene(*planner);
  Peer::acceptOdometry(*planner, 1.20, 0.0, 0.075);
  ASSERT_TRUE(Peer::initialAnchorSupported(*planner));
  const auto response = Peer::requestObjective(
      *planner, mgg::ObjectiveKind::kReturnHome,
      mgg::StateVec(0.0055, 0.0, 0.075, 0.6), "kf-home");
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);
  EXPECT_NEAR(response->path.front().position.x, 1.20, 0.06);
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-9);
  EXPECT_NEAR(response->path.back().position.y, 0.0, 1e-9);
  EXPECT_NEAR(response->path.back().position.z, 0.075, 1e-9);
  EXPECT_NEAR(response->path.back().orientation.z, std::sin(0.3), 1e-6);
}

TEST(PlannerObjective, SnapshotRefreshCannotStarveIndexedResponse) {
  using Query = mgg_msgs::srv::QueryMapBatch;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("partial_route_min_progress_m", 1.0),
       rclcpp::Parameter("indexed_map_query_timeout_s", 0.25),
       rclcpp::Parameter("indexed_map_query_service",
                         "/robot_1/mapping/query_batch")});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configurePartialNavigateScene(*planner);

  mgg_msgs::msg::MappingSnapshot snapshot;
  snapshot.component_id = "local";
  snapshot.epoch = 7;
  snapshot.graph_revision = 8;
  snapshot.geometry_revision = std::string(64, 'a');
  const auto stamp = planner->now();
  snapshot.source_stamp.sec = static_cast<std::int32_t>(stamp.seconds());
  snapshot.source_stamp.nanosec =
      static_cast<std::uint32_t>(stamp.nanoseconds() % 1000000000LL);
  snapshot.component_from_navigation.rotation.w = 1.0;
  Peer::acceptSnapshot(*planner, snapshot);

  auto io = std::make_shared<rclcpp::Node>("partial_two_thread_io");
  auto query_service = io->create_service<Query>(
      "/robot_1/mapping/query_batch",
      [planner, snapshot](const Query::Request::SharedPtr request,
                          Query::Response::SharedPtr response) {
        // This callback occupies the executor's second thread. With the old
        // objective-wide mutex, refreshing the authority here blocked forever
        // behind thread one and the indexed response could only time out.
        mgg_ros::PlannerNodeTestPeer::acceptSnapshot(*planner, snapshot);
        response->status = Query::Response::OK;
        response->component_id = request->component_id;
        response->epoch = request->epoch;
        response->graph_revision = request->graph_revision;
        response->geometry_revision = request->geometry_revision;
        const auto n = request->samples.size();
        response->occupancy.assign(n, Query::Response::FREE);
        response->ground_z.assign(n, 0.0);
        response->roughness.assign(n, 0.0);
        response->clearance.assign(n, 100.0);
        response->step.assign(n, false);
        response->drop.assign(n, false);
      });
  auto client = io->create_client<Service>("plan_objective");
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions{}, 2);
  executor.add_node(planner);
  executor.add_node(io);
  std::thread spinner([&executor]() { executor.spin(); });
  const bool objective_ready = client->wait_for_service(3s);
  const auto query_deadline = std::chrono::steady_clock::now() + 3s;
  while (!Peer::queryReady(*planner) &&
         std::chrono::steady_clock::now() < query_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  const bool query_ready = Peer::queryReady(*planner);
  EXPECT_TRUE(objective_ready);
  EXPECT_TRUE(query_ready);
  if (!objective_ready || !query_ready) {
    executor.cancel();
    spinner.join();
    return;
  }

  auto request = std::make_shared<Service::Request>();
  request->objective = Service::Request::NAVIGATE;
  request->component_id = snapshot.component_id;
  request->graph_revision = Peer::globalGraphRevision(*planner);
  request->map_revision = 0;
  request->map_epoch = snapshot.epoch;
  request->mapping_graph_revision = snapshot.graph_revision;
  request->geometry_revision = snapshot.geometry_revision;
  request->map_source_stamp = snapshot.source_stamp;
  request->goal.position.x = 1.5;
  request->goal.position.y = 0.0;
  request->goal.position.z = 0.075;
  request->goal.orientation.w = 1.0;
  auto future = client->async_send_request(request);
  const bool completed = future.wait_for(3s) == std::future_status::ready;
  EXPECT_TRUE(completed);
  if (completed) {
    const auto response = future.get();
    EXPECT_NE(response, nullptr);
    if (response) {
      EXPECT_EQ(response->status, Service::Response::SUCCEEDED)
          << response->reason;
      EXPECT_FALSE(response->partial);
      EXPECT_TRUE(response->indexed_map_validated) << response->reason;
    }
  }

  executor.cancel();
  spinner.join();
}

TEST(PlannerObjective, OlderIndexedRequestCannotReplaceNewerRouteCache) {
  using Query = mgg_msgs::srv::QueryMapBatch;
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("objective_route_horizon_m", 3.0),
       rclcpp::Parameter("indexed_map_query_timeout_s", 1.0),
       rclcpp::Parameter("indexed_map_query_service",
                         "/supersession/query_batch")});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*planner);
  Peer::setFreshMappingComponent(*planner, "shared-component");
  auto io = std::make_shared<rclcpp::Node>("supersession_io");
  std::string newer_route_id;
  std::atomic<int> query_count{0};
  auto query_group = io->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  auto query_service = io->create_service<Query>(
      "/supersession/query_batch",
      [planner, &newer_route_id,
       &query_count](const Query::Request::SharedPtr request,
                     Query::Response::SharedPtr response) {
        // The first objective has released planner_mutex_ while awaiting this
        // reply. Complete a newer rolling objective (its nested query is
        // served by the executor's third thread), then let the older request
        // resume and prove it cannot overwrite the newer cache.
        if (query_count.fetch_add(1) == 0) {
          const auto newer = Peer::requestMappedObjective(
              *planner, mgg::ObjectiveKind::kNavigate,
              mgg::StateVec(15.0, 0.0, 0.075, 0.2));
          if (newer->status == Service::Response::SUCCEEDED) {
            newer_route_id = newer->route_id;
          }
        }
        response->status = Query::Response::OK;
        response->component_id = request->component_id;
        response->epoch = request->epoch;
        response->graph_revision = request->graph_revision;
        response->geometry_revision = request->geometry_revision;
        const auto n = request->samples.size();
        response->occupancy.assign(n, Query::Response::FREE);
        response->ground_z.assign(n, 0.0);
        response->roughness.assign(n, 0.0);
        response->clearance.assign(n, 100.0);
        response->step.assign(n, false);
        response->drop.assign(n, false);
      },
      rclcpp::ServicesQoS(), query_group);
  auto client = io->create_client<Service>("plan_objective");
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions{}, 3);
  executor.add_node(planner);
  executor.add_node(io);
  std::thread spinner([&executor]() { executor.spin(); });
  const bool objective_ready = client->wait_for_service(3s);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!Peer::queryReady(*planner) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  const bool query_ready = Peer::queryReady(*planner);
  EXPECT_TRUE(objective_ready);
  EXPECT_TRUE(query_ready);
  if (!objective_ready || !query_ready) {
    executor.cancel();
    spinner.join();
    return;
  }

  auto request = std::make_shared<Service::Request>();
  request->objective = Service::Request::NAVIGATE;
  const auto snapshot = Peer::mappingSnapshot(*planner);
  request->component_id = snapshot.component_id;
  request->map_epoch = snapshot.epoch;
  request->mapping_graph_revision = snapshot.graph_revision;
  request->geometry_revision = snapshot.geometry_revision;
  request->map_source_stamp = snapshot.source_stamp;
  request->goal.position.x = 12.0;
  request->goal.position.z = 0.075;
  request->goal.orientation.w = 1.0;
  auto future = client->async_send_request(request);
  const bool completed = future.wait_for(3s) == std::future_status::ready;
  EXPECT_TRUE(completed);
  if (!completed) {
    executor.cancel();
    spinner.join();
    return;
  }
  const auto older = future.get();
  EXPECT_NE(older, nullptr);
  if (!older) {
    executor.cancel();
    spinner.join();
    return;
  }
  EXPECT_EQ(older->status, Service::Response::BLOCKED) << older->reason;
  EXPECT_NE(older->reason.find("superseded"), std::string::npos);
  EXPECT_FALSE(newer_route_id.empty());
  if (newer_route_id.empty()) {
    executor.cancel();
    spinner.join();
    return;
  }
  EXPECT_EQ(Peer::cachedObjectiveRouteId(*planner), newer_route_id);

  executor.cancel();
  spinner.join();
}

TEST(PlannerObjective, NavigateBeyondMeasuredFloorDrivesPrefixSections) {
  using Query = mgg_msgs::srv::QueryMapBatch;
  using Peer = mgg_ros::PlannerNodeTestPeer;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("indexed_map_query_timeout_s", 2.0),
       rclcpp::Parameter("indexed_map_query_service",
                         "/measured_floor/query_batch")});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureLongKnownRoad(*planner);
  Peer::setFreshMappingComponent(*planner, "shared-component");
  auto io = std::make_shared<rclcpp::Node>("measured_floor_io");
  // The native map knows the whole road. The indexed authority has measured
  // only the floor this far, exactly as a cold-start low lidar leaves it.
  std::atomic<double> measured_to_x{2.0};
  std::atomic<int> query_count{0};
  auto query_group =
      io->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto query_service = io->create_service<Query>(
      "/measured_floor/query_batch",
      [&measured_to_x, &query_count](const Query::Request::SharedPtr request,
                                     Query::Response::SharedPtr response) {
        ++query_count;
        response->status = Query::Response::OK;
        response->component_id = request->component_id;
        response->epoch = request->epoch;
        response->graph_revision = request->graph_revision;
        response->geometry_revision = request->geometry_revision;
        const auto n = request->samples.size();
        response->occupancy.assign(n, Query::Response::FREE);
        response->clearance.assign(n, 100.0);
        response->step.assign(n, false);
        response->drop.assign(n, false);
        const double reach = measured_to_x.load();
        for (std::size_t i = 0; i < n; ++i) {
          const bool measured = request->samples[i].x <= reach + 1e-9;
          response->ground_z.push_back(
              measured ? 0.0 : std::numeric_limits<double>::quiet_NaN());
          response->roughness.push_back(
              measured ? 0.0 : std::numeric_limits<double>::quiet_NaN());
        }
      },
      rclcpp::ServicesQoS(), query_group);
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions{}, 3);
  executor.add_node(planner);
  executor.add_node(io);
  std::thread spinner([&executor]() { executor.spin(); });
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!Peer::queryReady(*planner) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  const bool query_ready = Peer::queryReady(*planner);
  EXPECT_TRUE(query_ready);
  if (!query_ready) {
    executor.cancel();
    spinner.join();
    return;
  }

  // The operator commits to a destination far beyond the measured floor.
  const auto first = Peer::requestMappedObjective(
      *planner, mgg::ObjectiveKind::kNavigate,
      mgg::StateVec(12.0, 0.0, 0.075, 0.0));
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first->status, Service::Response::SUCCEEDED) << first->reason;
  EXPECT_TRUE(first->partial);
  EXPECT_TRUE(first->indexed_map_validated);
  EXPECT_FALSE(first->route_id.empty());
  ASSERT_FALSE(first->path.empty());
  ASSERT_FALSE(first->global_path.empty());
  // The executable section stops on measured support past the useful-progress
  // bound, while the committed destination stays exactly where it was asked.
  const double first_reach = first->path.back().position.x;
  EXPECT_GE(first_reach, 1.0);
  EXPECT_LE(first_reach, 2.0 + 1e-6);
  EXPECT_NEAR(first_reach, 2.0, 0.30);
  EXPECT_NEAR(first->global_path.back().position.x, 12.0, 1e-9);
  EXPECT_TRUE(Peer::hasCachedObjectiveRoute(*planner));
  EXPECT_EQ(Peer::cachedObjectiveRouteId(*planner), first->route_id);
  EXPECT_NEAR(Peer::cachedObjectiveRouteEndpoint(*planner).x(), first_reach,
              1e-9);
  const std::size_t resume_index = Peer::cachedObjectiveRouteNextIndex(*planner);
  EXPECT_LT(resume_index, Peer::cachedObjectiveRouteSize(*planner));

  auto refine_request =
      std::make_shared<mgg_msgs::srv::RefineObjectiveRoute::Request>();
  refine_request->route_id = first->route_id;
  refine_request->component_id = Peer::mappingSnapshot(*planner).component_id;
  refine_request->map_epoch = Peer::mappingSnapshot(*planner).epoch;
  refine_request->mapping_graph_revision =
      Peer::mappingSnapshot(*planner).graph_revision;
  refine_request->geometry_revision =
      Peer::mappingSnapshot(*planner).geometry_revision;
  refine_request->map_source_stamp =
      Peer::mappingSnapshot(*planner).source_stamp;

  // The robot arrives. Nothing new has been measured, so the next section has no
  // prefix worth driving and keeps failing with its exact terrain reason. The
  // cached route survives, so a later attempt can still continue.
  Peer::acceptOdometry(*planner, first_reach, 0.0, 0.075);
  const auto stalled = Peer::refineObjectiveRoute(*planner, refine_request);
  ASSERT_NE(stalled, nullptr);
  EXPECT_EQ(stalled->status,
            mgg_msgs::srv::RefineObjectiveRoute::Response::BLOCKED)
      << stalled->reason;
  EXPECT_TRUE(stalled->path.empty());
  EXPECT_TRUE(Peer::hasCachedObjectiveRoute(*planner));

  // The next floor section is measured. The continuation resumes from the
  // emitted prefix endpoint and commits to the same destination.
  measured_to_x = 5.0;
  const auto resumed = Peer::refineObjectiveRoute(*planner, refine_request);
  ASSERT_NE(resumed, nullptr);
  EXPECT_EQ(resumed->status,
            mgg_msgs::srv::RefineObjectiveRoute::Response::SUCCEEDED)
      << resumed->reason;
  EXPECT_TRUE(resumed->partial);
  EXPECT_TRUE(resumed->indexed_map_validated);
  ASSERT_FALSE(resumed->path.empty());
  EXPECT_NEAR(resumed->path.front().position.x, first_reach, 0.60);
  const double second_reach = resumed->path.back().position.x;
  EXPECT_GE(second_reach, first_reach + 1.0);
  EXPECT_LE(second_reach, 5.0 + 1e-6);
  EXPECT_NEAR(second_reach, 5.0, 0.30);
  EXPECT_EQ(Peer::cachedObjectiveRouteId(*planner), first->route_id);
  EXPECT_NEAR(Peer::cachedObjectiveRouteEndpoint(*planner).x(), second_reach,
              1e-9);
  EXPECT_GE(Peer::cachedObjectiveRouteNextIndex(*planner), resume_index);
  EXPECT_LT(Peer::cachedObjectiveRouteNextIndex(*planner),
            Peer::cachedObjectiveRouteSize(*planner));

  executor.cancel();
  spinner.join();
}

TEST(PlannerObjective, PartialProxyCannotCrossObservedWall) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05),
       rclcpp::Parameter("partial_route_min_progress_m", 1.0)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configurePartialNavigateScene(*planner, /*blocked_by_wall=*/true);

  const auto response = Peer::requestBoundNavigate(
      *planner, mgg::StateVec(30.0, 0.0, 0.075, 0.0));
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_FALSE(response->partial);
  EXPECT_TRUE(response->path.empty());
}

TEST(PlannerObjective, PartialProxyHonorsPlatformStepCapInOctomap) {
  struct Scenario {
    double step_height;
    double step_cap;
    bool accepted;
  };
  const std::vector<Scenario> scenarios{
      {0.05, 0.10, true},   // Bunker/Scout: sub-cap curb.
      {0.15, 0.10, false},  // Bunker/Scout: sharp curb above cap.
      {0.20, 0.30, true},   // Spot: same terrain remains below its cap.
  };
  for (const Scenario& scenario : scenarios) {
    SCOPED_TRACE("height=" + std::to_string(scenario.step_height) +
                 " cap=" + std::to_string(scenario.step_cap));
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("map.resolution", 0.05),
         rclcpp::Parameter("partial_route_min_progress_m", 1.0)});
    auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
    using Peer = mgg_ros::PlannerNodeTestPeer;
    Peer::configurePartialNavigateScene(
        *planner, /*blocked_by_wall=*/false, scenario.step_height,
        scenario.step_cap);

    const auto response = Peer::requestBoundNavigate(
        *planner, mgg::StateVec(1.50, 0.0, 0.075 + scenario.step_height, 0.0));
    ASSERT_NE(response, nullptr);
    if (scenario.accepted) {
      EXPECT_EQ(response->status, Service::Response::SUCCEEDED)
          << response->reason;
      EXPECT_FALSE(response->partial);
      EXPECT_FALSE(response->path.empty());
    } else {
      EXPECT_EQ(response->status, Service::Response::BLOCKED)
          << response->reason;
      EXPECT_FALSE(response->partial);
      EXPECT_TRUE(response->path.empty());
    }

    // Resolving a stale start-height goal onto known raised terrain does not
    // waive the edge step limit. The sharp Scout curb remains blocked, while
    // terrain within each platform's configured limit retains its result.
    const auto stale_height_response = Peer::requestBoundNavigate(
        *planner, mgg::StateVec(1.50, 0.0, 0.075, 0.0));
    ASSERT_NE(stale_height_response, nullptr);
    EXPECT_EQ(stale_height_response->status,
              scenario.accepted ? Service::Response::SUCCEEDED
                                : Service::Response::BLOCKED)
        << stale_height_response->reason;
  }
}

TEST(PlannerBackbone, DelayedSupportBackfillsBentTrajectoryBeyondParentRadius) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureBackboneTest(*planner);

  // Travel around an off-grid right angle before any terrain is mapped. The
  // endpoint is farther from home than the old 5*spacing parent radius.
  Peer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
  for (int i = 1; i <= 11; ++i) {
    Peer::acceptOdometry(*planner, 0.0, i * 0.25, 0.075);
  }
  for (int i = 1; i <= 12; ++i) {
    Peer::acceptOdometry(*planner, i * 0.25, 2.75, 0.075);
  }
  ASSERT_EQ(Peer::globalVertices(*planner), 1);
  ASSERT_GE(Peer::pendingBreadcrumbs(*planner), 12u);
  EXPECT_GT(Peer::pendingBreadcrumbLength(*planner), 5.0);

  // Observe only the two travelled legs. An obstacle makes the direct
  // home-to-current diagonal invalid, so success requires retained history.
  Peer::observeGroundRectangle(*planner, -0.3, 3.3, -0.3, 3.1);
  Peer::observeFreeBodyBox(*planner, {0.0, 1.375, 0.40},
                           {0.6, 3.35, 0.40});
  Peer::observeFreeBodyBox(*planner, {1.5, 2.75, 0.40}, {3.6, 0.6, 0.40});
  Peer::addOccupiedVoxel(*planner, 1.5, 1.375, 0.30);
  Peer::finishMapRevision(*planner);

  mgg::StateVec home(0.0, 0.0, 0.075, 0.0);
  mgg::StateVec current(3.0, 2.75, 0.075, 0.0);
  EXPECT_NE(Peer::globalEdgeStatus(*planner, home, current),
            mgg::ProjectedEdgeStatus::kAdmissible);
  EXPECT_EQ(Peer::pendingBreadcrumbs(*planner), 0u);
  EXPECT_FALSE(Peer::backboneHistoryLost(*planner));
  EXPECT_GE(Peer::globalVertices(*planner), 12);
  EXPECT_EQ(Peer::globalEdges(*planner), Peer::globalVertices(*planner) - 1);
  EXPECT_TRUE(Peer::allGlobalEdgesAtMost(*planner, 0.75));
  EXPECT_TRUE(Peer::hasGlobalVertexNear(*planner, 0.0, 2.75, 0.05));

  const mgg::RouteCorridor route = Peer::planHome(*planner, home);
  EXPECT_EQ(route.status, mgg::PlanningStatus::kSucceeded) << route.reason;
  EXPECT_GE(route.poses.size(), 12u);
}

TEST(PlannerBackbone, BlockedHeadWaitsForNewMapAndIsNeverSkipped) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureBackboneTest(*planner);

  Peer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
  for (int i = 1; i <= 8; ++i) {
    Peer::acceptOdometry(*planner, i * 0.25, 0.0, 0.075);
  }
  ASSERT_EQ(Peer::pendingBreadcrumbs(*planner), 4u);

  // Ground can support the root and every queued sample, while body clearance
  // remains unknown. The first head must hold back all later known positions.
  Peer::observeGroundSupport(*planner);
  ASSERT_TRUE(Peer::initialAnchorSupported(*planner));
  EXPECT_EQ(Peer::globalVertices(*planner), 1);
  EXPECT_EQ(Peer::pendingBreadcrumbs(*planner), 4u);

  // More odometry on the same map revision cannot consume or skip that head.
  Peer::acceptOdometry(*planner, 2.10, 0.0, 0.075);
  EXPECT_EQ(Peer::globalVertices(*planner), 1);
  EXPECT_EQ(Peer::pendingBreadcrumbs(*planner), 4u);

  Peer::observeBodyCorridor(*planner);
  EXPECT_EQ(Peer::pendingBreadcrumbs(*planner), 0u);
  EXPECT_EQ(Peer::globalVertices(*planner), 5);
  EXPECT_EQ(Peer::globalEdges(*planner), 4);
  EXPECT_TRUE(Peer::allGlobalEdgesAtMost(*planner, 0.75));
}

TEST(PlannerBackbone, RepeatedOutAndBackReusesOwnedTrajectoryVertices) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureBackboneTest(*planner);

  Peer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
  Peer::observeGroundSupport(*planner);
  Peer::observeBodyCorridor(*planner);
  for (int i = 1; i <= 8; ++i) {
    Peer::acceptOdometry(*planner, i * 0.25, 0.0, 0.075);
  }
  ASSERT_EQ(Peer::globalVertices(*planner), 5);
  ASSERT_EQ(Peer::globalEdges(*planner), 4);

  for (int i = 7; i >= 0; --i) {
    Peer::acceptOdometry(*planner, i * 0.25, 0.0, 0.075);
  }
  EXPECT_EQ(Peer::pendingBreadcrumbs(*planner), 0u);
  EXPECT_EQ(Peer::globalVertices(*planner), 5);
  EXPECT_EQ(Peer::globalEdges(*planner), 4);
  EXPECT_FALSE(Peer::backboneHistoryLost(*planner));
}

TEST(PlannerBackbone, StationaryNoiseDoesNotFillBoundedQueueAndOverflowFailsHome) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureBackboneTest(*planner);
  Peer::setBackboneLimits(*planner, 2, 100.0, 32);

  Peer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
  for (int i = 0; i < 1000; ++i) {
    Peer::acceptOdometry(*planner, i % 2 == 0 ? 0.01 : -0.01, 0.0, 0.075);
  }
  EXPECT_EQ(Peer::pendingBreadcrumbs(*planner), 0u);
  EXPECT_FALSE(Peer::backboneHistoryLost(*planner));

  Peer::acceptOdometry(*planner, 0.50, 0.0, 0.075);
  Peer::acceptOdometry(*planner, 1.00, 0.0, 0.075);
  Peer::acceptOdometry(*planner, 1.50, 0.0, 0.075);
  ASSERT_TRUE(Peer::backboneHistoryLost(*planner));
  EXPECT_EQ(Peer::pendingBreadcrumbs(*planner), 2u);

  const mgg::StateVec home(0.0, 0.0, 0.075, 0.0);
  const auto response =
      Peer::requestObjective(*planner, mgg::ObjectiveKind::kReturnHome, home);
  EXPECT_EQ(response->status, static_cast<std::uint8_t>(
                                  mgg::PlanningStatus::kBlocked));
  EXPECT_NE(response->reason.find("global trajectory history was lost"),
            std::string::npos);
}

TEST(PlannerBackbone, NumericNoProgressLatchesLostHistoryOnce) {
  rclcpp::NodeOptions options;
  using Peer = mgg_ros::PlannerNodeTestPeer;

  auto boundary = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureAerialBackboneTest(*boundary);
  Peer::setBackboneSpacing(*boundary, 0.01);
  Peer::acceptOdometry(*boundary, 0.0, 0.0, 0.0);
  Peer::acceptOdometry(*boundary, 0.0099999995, 0.0, 0.0);
  EXPECT_FALSE(Peer::backboneHistoryLost(*boundary));
  EXPECT_EQ(Peer::pendingBreadcrumbs(*boundary), 0u);
  Peer::acceptOdometry(*boundary, 0.01, 0.0, 0.0);
  EXPECT_FALSE(Peer::backboneHistoryLost(*boundary));
  EXPECT_EQ(Peer::pendingBreadcrumbs(*boundary), 1u);

  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureAerialBackboneTest(*planner);

  // At this magnitude, adding the 0.5 m spacing rounds back to the anchor.
  // The sampler must stop instead of filling the bounded queue with copies.
  Peer::acceptOdometry(*planner, 1e20, 0.0, 0.0);
  Peer::acceptOdometry(*planner, 1e20 + 1e10, 0.0, 0.0);
  ASSERT_TRUE(Peer::backboneHistoryLost(*planner));
  EXPECT_EQ(Peer::pendingBreadcrumbs(*planner), 0u);

  // A later extreme displacement cannot append or emit another loss state.
  Peer::acceptOdometry(*planner, -1e154, 0.0, 0.0);
  EXPECT_EQ(Peer::pendingBreadcrumbs(*planner), 0u);
  const mgg::StateVec home(1e20, 0.0, 0.0, 0.0);
  const auto response =
      Peer::requestObjective(*planner, mgg::ObjectiveKind::kReturnHome, home);
  EXPECT_EQ(response->status, static_cast<std::uint8_t>(
                                  mgg::PlanningStatus::kBlocked));
  EXPECT_NE(response->reason.find("global trajectory history was lost"),
            std::string::npos);

  auto extreme = std::make_shared<mgg_ros::PlannerNode>(options);
  Peer::configureAerialBackboneTest(*extreme);
  Peer::acceptOdometry(*extreme, -1e308, 0.0, 0.0);
  Peer::acceptOdometry(*extreme, 1e308, 0.0, 0.0);
  ASSERT_TRUE(Peer::backboneHistoryLost(*extreme));
  EXPECT_EQ(Peer::pendingBreadcrumbs(*extreme), 0u);
  EXPECT_NE(Peer::backboneHistoryLostReason(*extreme).find("finite"),
            std::string::npos);
}

TEST(PlannerBackbone, LegacyExploreRetainsHangingRootBootstrapPolicy) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.resolution", 0.05)});
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  mgg_ros::PlannerNodeTestPeer::configureBackboneTest(*planner);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
  ASSERT_FALSE(mgg_ros::PlannerNodeTestPeer::initialAnchorSupported(*planner));
  mgg_ros::PlannerNodeTestPeer::observeDestinationSupport(*planner, 0.50);

  // Response-boundary fixture for output already admitted by local graph
  // expansion, whose established bootstrap policy permits a hanging root.
  mgg::RouteCorridor explore;
  explore.status = mgg::PlanningStatus::kSucceeded;
  explore.request.objective = mgg::ObjectiveKind::kExplore;
  explore.poses.push_back(mgg::StateVec(0.50, 0.0, 0.30, 0.0));
  const mgg::FeasiblePath response =
      mgg_ros::PlannerNodeTestPeer::refine(*planner, explore);
  ASSERT_EQ(response.status, mgg::PlanningStatus::kSucceeded)
      << response.reason;
  ASSERT_EQ(response.poses.size(), 1u);
  EXPECT_NEAR(response.poses.front().z(), 0.075, 1e-9);

  // Explicit objectives do not inherit the hanging-root exception.
  mgg::RouteCorridor home = explore;
  home.request.objective = mgg::ObjectiveKind::kReturnHome;
  home.request.goal.pose = mgg::StateVec(0.50, 0.0, 0.075, 0.0);
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::refine(*planner, home).status,
            mgg::PlanningStatus::kBlocked);
}

class ObjectiveService : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("map.resolution", 0.05)});
    planner = std::make_shared<mgg_ros::PlannerNode>(options);
    caller = std::make_shared<rclcpp::Node>("objective_service_test_client");
    client = caller->create_client<Service>("plan_objective");
    legacy_client = caller->create_client<LegacyService>("mggplanner");
    executor.add_node(planner);
    spinner = std::thread([this]() { executor.spin(); });
    ASSERT_TRUE(client->wait_for_service(3s));
    ASSERT_TRUE(legacy_client->wait_for_service(3s));
  }

  void TearDown() override {
    executor.cancel();
    spinner.join();
  }

  Service::Response::SharedPtr call(Service::Request::SharedPtr request) {
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(caller, future, 3s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

  LegacyService::Response::SharedPtr callLegacy(
      LegacyService::Request::SharedPtr request) {
    auto future = legacy_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(caller, future, 3s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<mgg_ros::PlannerNode> planner;
  std::shared_ptr<rclcpp::Node> caller;
  rclcpp::Client<Service>::SharedPtr client;
  rclcpp::Client<LegacyService>::SharedPtr legacy_client;
  std::thread spinner;
};

TEST_F(ObjectiveService, ValidatesBeforeTouchingUnavailableMap) {
  auto invalid = std::make_shared<Service::Request>();
  invalid->objective = 255;
  auto response = call(invalid);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::UNSUPPORTED_OBJECTIVE);

  auto wrong_component = std::make_shared<Service::Request>();
  wrong_component->objective = Service::Request::NAVIGATE;
  wrong_component->component_id = "another-component";
  wrong_component->goal.orientation.w = 1.0;
  response = call(wrong_component);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::STALE_REVISION);

  auto valid = std::make_shared<Service::Request>();
  valid->objective = Service::Request::NAVIGATE;
  valid->goal.orientation.w = 1.0;
  response = call(valid);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED);
}

TEST_F(ObjectiveService,
       ColdStartLegacyExploreThenPinnedObjectiveKeepsBaseHeight) {
  mgg_ros::PlannerNodeTestPeer::configureExploreServiceScene(
      *planner, /*support_root=*/false);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.0, 0.0, 0.20);
  ASSERT_FALSE(mgg_ros::PlannerNodeTestPeer::initialAnchorSupported(*planner));

  auto legacy_request = std::make_shared<LegacyService::Request>();
  legacy_request->bound_mode = LegacyService::Request::EXACT_BOUND;
  const auto legacy = callLegacy(legacy_request);
  ASSERT_NE(legacy, nullptr);
  ASSERT_EQ(legacy->status, LegacyService::Response::FORWARD);
  ASSERT_GE(legacy->path.size(), 2u);

  auto pinned_request = std::make_shared<Service::Request>();
  pinned_request->objective = Service::Request::EXPLORE;
  pinned_request->graph_revision =
      mgg_ros::PlannerNodeTestPeer::localGraphRevision(*planner);
  pinned_request->map_revision =
      mgg_ros::PlannerNodeTestPeer::localGraphMapRevision(*planner);
  const auto pinned = call(pinned_request);
  ASSERT_NE(pinned, nullptr);
  ASSERT_EQ(pinned->status, Service::Response::SUCCEEDED) << pinned->reason;
  ASSERT_EQ(pinned->path.size(), legacy->path.size());
  for (std::size_t i = 0; i < legacy->path.size(); ++i) {
    EXPECT_NEAR(pinned->path[i].position.x, legacy->path[i].position.x, 1e-9);
    EXPECT_NEAR(pinned->path[i].position.y, legacy->path[i].position.y, 1e-9);
    EXPECT_NEAR(pinned->path[i].position.z, legacy->path[i].position.z, 1e-9);
  }
  // A second subtraction would put the supported destination below ground.
  EXPECT_GT(pinned->path.back().position.z, -0.05);
}

TEST_F(ObjectiveService, NewObstacleAtHomeWaypointUsesLocalEndpointDetour) {
  mgg_ros::PlannerNodeTestPeer::configureGridServiceScene(*planner);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.0, 0.0, 0.075);
  mgg::StateVec home_state = mgg::StateVec::Zero();
  home_state.z() = 0.075;
  mgg::StateVec first_state = home_state;
  first_state.x() = 0.60;
  EXPECT_EQ(mgg_ros::PlannerNodeTestPeer::globalEdgeStatus(
                *planner, home_state, first_state),
            mgg::ProjectedEdgeStatus::kAdmissible);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.60, 0.0, 0.075);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 1.20, 0.0, 0.075);
  ASSERT_GE(mgg_ros::PlannerNodeTestPeer::globalVertices(*planner), 3);
  ASSERT_GE(mgg_ros::PlannerNodeTestPeer::globalEdges(*planner), 2);

  auto home = std::make_shared<Service::Request>();
  home->objective = Service::Request::RETURN_HOME;
  home->goal.position.z = 0.075;
  home->goal.orientation.w = 1.0;
  home->graph_revision =
      mgg_ros::PlannerNodeTestPeer::globalGraphRevision(*planner);
  SCOPED_TRACE(mgg_ros::PlannerNodeTestPeer::globalGraphDescription(*planner));
  auto response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  ASSERT_GE(response->path.size(), 2u);

  mgg_ros::PlannerNodeTestPeer::addCorridorObstacle(*planner, 0.60);
  response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED)
      << response->reason;
  ASSERT_FALSE(response->path.empty());
  EXPECT_TRUE(std::any_of(response->path.begin(), response->path.end(),
                          [](const geometry_msgs::msg::Pose& pose) {
                            return std::abs(pose.position.y) > 0.1;
                          }));
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-9);
}

TEST_F(ObjectiveService, GridHomeDetoursOnObservedGroundAndBlocksWall) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  // Home must use the explicit-objective envelope.  The legacy generic
  // margin is deliberately too narrow to get around the observed blocker.
  Peer::configureGridServiceScene(*planner, 0.0, 0.0);
  Peer::setProvisionalUnknownGround(*planner, true);
  for (const double x : {0.0, 0.6, 1.2}) {
    Peer::acceptOdometry(*planner, x, 0.0, 0.075);
  }
  ASSERT_GE(Peer::globalVertices(*planner), 3);
  auto home = std::make_shared<Service::Request>();
  home->objective = Service::Request::RETURN_HOME;
  home->goal.position.z = 0.075;
  const double yaw = 0.7;
  home->goal.orientation.z = std::sin(yaw / 2);
  home->goal.orientation.w = std::cos(yaw / 2);
  home->graph_revision = Peer::globalGraphRevision(*planner);
  auto response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;

  // Block the edge between graph vertices, leaving both endpoints clear.
  // New backbone samples are at 0.5 m spacing, so place the blocker midway
  // between 0.5 and 1.0 rather than on either supported endpoint.
  Peer::addGridObstacle(*planner, 0.325, false, 0.75);
  response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  ASSERT_GT(response->path.size(), 3u);
  bool detoured = false;
  for (const auto& pose : response->path) {
    detoured = detoured || std::abs(pose.position.y) > 0.15;
    EXPECT_NEAR(pose.position.z, 0.1, 1e-6);
  }
  EXPECT_TRUE(detoured);
  EXPECT_NEAR(response->path.front().position.x, 1.2, 1e-6);
  EXPECT_NEAR(response->path.back().position.x, 0.0, 1e-6);
  EXPECT_NEAR(response->path.back().orientation.z, std::sin(yaw / 2), 1e-6);
  EXPECT_NEAR(response->path.back().orientation.w, std::cos(yaw / 2), 1e-6);

  // The wall spans the observed floor and detour window; unknown space beyond
  // it must not be invented as a route around the ends.
  Peer::addGridObstacle(*planner, 0.325, true, 0.75);
  response = call(home);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());
  EXPECT_EQ(response->reason.find("primary direct:"), std::string::npos)
      << response->reason;
  EXPECT_EQ(response->reason.find("local endpoint fallback"),
            std::string::npos)
      << response->reason;

}

TEST_F(ObjectiveService, GridBodyOffsetPreservesHeightAndChecksRaisedObstacle) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureGridServiceScene(*planner, 0.30, 0.0);
  Peer::setObjectiveGridWindow(*planner, 0.0, 0.0);
  for (const double x : {0.0, 0.6, 1.2}) {
    Peer::acceptOdometry(*planner, x, 0.0, 0.075);
  }
  auto home = std::make_shared<Service::Request>();
  home->objective = Service::Request::RETURN_HOME;
  home->goal.position.z = 0.075;
  home->goal.orientation.w = 1.0;
  home->graph_revision = Peer::globalGraphRevision(*planner);
  auto response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;
  ASSERT_GT(response->path.size(), 3u);
  for (const auto& pose : response->path) EXPECT_NEAR(pose.position.z, 0.1, 1e-6);

  // Above the unoffset box and its ground probes, but inside the real box
  // centered 30cm higher. The zero-margin search cannot sidestep this obstacle.
  Peer::addGridObstacle(*planner, 0.675, false);
  response = call(home);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());
}

TEST_F(ObjectiveService, GridRejectsSingleUnknownBodyVoxelWithZeroOffset) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureGridServiceScene(*planner, 0.0, 0.0);
  Peer::setObjectiveGridWindow(*planner, 0.0, 0.0);
  for (const double x : {0.0, 0.6, 1.2}) {
    Peer::acceptOdometry(*planner, x, 0.0, 0.075);
  }
  auto home = std::make_shared<Service::Request>();
  home->objective = Service::Request::RETURN_HOME;
  home->goal.position.z = 0.075;
  home->goal.orientation.w = 1.0;
  home->graph_revision = Peer::globalGraphRevision(*planner);
  auto response = call(home);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;

  // This is one body-volume key between graph vertices. The legacy 25%
  // tolerance accepts it; explicit refinement must treat any unknown as
  // blocked even when center_offset is zero.
  ASSERT_TRUE(Peer::makeGridVoxelUnknown(*planner, 0.9, 0.025, 0.325));
  response = call(home);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());
  EXPECT_NE(response->reason.find("route corridor waypoint[0] rejected"),
            std::string::npos);
  EXPECT_NE(response->reason.find("body includes unknown space at"),
            std::string::npos);
}

TEST_F(ObjectiveService, GridGeofenceRejectsStationaryAndCrossingRoutes) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureGridServiceScene(*planner);
  for (const double x : {0.0, 0.6, 1.2}) {
    Peer::acceptOdometry(*planner, x, 0.0, 0.075);
  }
  auto goal = std::make_shared<Service::Request>();
  goal->objective = Service::Request::RETURN_HOME;
  goal->goal.position.x = 1.2;
  goal->goal.position.z = 0.075;
  goal->goal.orientation.w = 1.0;
  goal->graph_revision = Peer::globalGraphRevision(*planner);
  auto response = call(goal);
  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->status, Service::Response::SUCCEEDED) << response->reason;

  Peer::setGridGeofence(*planner, 1.1, 1.3, -0.15, 0.15);
  response = call(goal);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());
  EXPECT_NE(response->reason.find("current pose rejected: geofence violation at"),
            std::string::npos);

  Peer::setGridGeofence(*planner, 0.85, 0.95, -1.3, 1.3);
  goal->goal.position.x = 0.0;
  response = call(goal);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED) << response->reason;
  EXPECT_TRUE(response->path.empty());
}

TEST_F(ObjectiveService, AuthorityBindingDoesNotRequireIndexedQuery) {
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::hasQueryClient(*planner));
  mgg_msgs::msg::MappingSnapshot snapshot;
  snapshot.component_id = "component-a";
  snapshot.epoch = 7;
  snapshot.graph_revision = 8;
  snapshot.geometry_revision = std::string(64, 'a');
  snapshot.source_stamp.sec = 3;
  snapshot.component_from_navigation.rotation.w = 1.0;
  mgg_ros::PlannerNodeTestPeer::acceptSnapshot(*planner, snapshot);

  const auto request = [&snapshot]() {
    auto value = std::make_shared<Service::Request>();
    value->objective = Service::Request::RETURN_HOME;
    value->component_id = snapshot.component_id;
    value->goal.orientation.w = 1.0;
    value->map_epoch = snapshot.epoch;
    value->mapping_graph_revision = snapshot.graph_revision;
    value->geometry_revision = snapshot.geometry_revision;
    value->map_source_stamp = snapshot.source_stamp;
    return value;
  };

  auto response = call(request());
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::BLOCKED);

  auto wrong_component = request();
  wrong_component->component_id = "component-b";
  response = call(wrong_component);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::STALE_REVISION);

  auto wrong_graph = request();
  ++wrong_graph->mapping_graph_revision;
  response = call(wrong_graph);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::STALE_REVISION);

  auto wrong_geometry = request();
  wrong_geometry->geometry_revision = std::string(64, 'b');
  response = call(wrong_geometry);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::STALE_REVISION);

  mgg_ros::PlannerNodeTestPeer::expireSnapshot(*planner);
  response = call(request());
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(response->status, Service::Response::STALE_REVISION);
}

TEST(IndexedObjectiveService, MissingSnapshotFailsClosedWithoutNestedSpin) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("indexed_map_query_service",
                                    "/robot_1/mapping/query_batch");
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  auto caller = std::make_shared<rclcpp::Node>("indexed_objective_test_client");
  auto client = caller->create_client<Service>("plan_objective");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(planner);
  std::thread spinner([&executor]() { executor.spin(); });
  const bool ready = client->wait_for_service(3s);
  EXPECT_TRUE(ready);
  if (!ready) {
    executor.cancel();
    spinner.join();
    return;
  }

  auto request = std::make_shared<Service::Request>();
  request->objective = Service::Request::NAVIGATE;
  request->component_id = "component-a";
  request->goal.orientation.w = 1.0;
  request->map_epoch = 1;
  request->mapping_graph_revision = 2;
  request->geometry_revision = std::string(64, 'a');
  request->map_source_stamp.sec = 3;
  auto future = client->async_send_request(request);
  const auto result = rclcpp::spin_until_future_complete(caller, future, 3s);
  EXPECT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  if (result == rclcpp::FutureReturnCode::SUCCESS) {
    const auto response = future.get();
    EXPECT_NE(response, nullptr);
    if (response) {
      EXPECT_EQ(response->status, Service::Response::STALE_REVISION);
      EXPECT_FALSE(response->indexed_map_validated);
    }
  }

  executor.cancel();
  spinner.join();
}

TEST(IndexedObjectiveService, BatchedQueryIsBoundedAndUsesComponentFrame) {
  using Query = mgg_msgs::srv::QueryMapBatch;
  rclcpp::NodeOptions options;
  options.append_parameter_override("indexed_map_query_service",
                                    "/robot_1/mapping/query_batch");
  options.append_parameter_override("indexed_map_query_timeout_s", 0.05);
  options.append_parameter_override("indexed_map_sample_spacing_m", 0.30);
  auto planner = std::make_shared<mgg_ros::PlannerNode>(options);
  mgg_ros::PlannerNodeTestPeer::configureBackboneTest(*planner);
  auto server = std::make_shared<rclcpp::Node>("indexed_query_test_server");
  std::atomic<bool> delay{false};
  std::atomic<std::uint8_t> response_status{Query::Response::OK};
  std::atomic<double> ground_offset{0.0};
  std::atomic<double> leading_root_ground_offset{0.0};
  std::atomic<double> ground_from_sample{0.30};
  std::atomic<double> received_body_x{0.0};
  std::atomic<double> received_body_y{0.0};
  std::atomic<double> received_body_z{0.0};
  std::atomic<double> received_max_step{0.0};
  std::atomic<double> received_max_drop{0.0};
  std::atomic<std::uint8_t> sample_occupancy{Query::Response::FREE};
  std::atomic<double> sample_clearance{100.0};
  std::atomic<std::size_t> blind_ground_prefix{0};
  std::atomic<std::size_t> blind_clearance_prefix{0};
  std::atomic<std::size_t> blind_ground_sample{
      std::numeric_limits<std::size_t>::max()};
  std::atomic<std::size_t> step_sample{std::numeric_limits<std::size_t>::max()};
  std::atomic<std::size_t> drop_sample{std::numeric_limits<std::size_t>::max()};
  std::atomic<double> drop_at_or_after_x{
      std::numeric_limits<double>::infinity()};
  std::atomic<std::size_t> occupied_from_sample{
      std::numeric_limits<std::size_t>::max()};
  enum TerrainCase : int {
    kOrdinaryTerrain = 0,
    kTwoBlindIntervals,
    kOverlongInternalGap,
    kTerminalGap,
    kLateTerminalGap,
    kFiniteRootThenBlind,
    kOccupiedInsideGap,
    kLowClearanceInsideGap,
    kRoughClosingSupport,
    kStepInsideGap,
    kDropInsideGap,
    kGroundOnlyMissing,
    kRoughnessOnlyMissing,
    kStepAcrossGap,
    kClosingHeightMismatch,
  };
  std::atomic<int> terrain_case{kOrdinaryTerrain};
  std::atomic<double> refinement_dip{0.0};
  std::atomic<double> compatible_refinement_dip{0.0};
  std::atomic<double> refinement_from_x{0.0};
  std::atomic<std::size_t> query_count{0};
  std::atomic<std::size_t> first_query_sample_count{0};
  std::atomic<bool> occupy_refined_body{false};
  std::atomic<bool> occupy_if_early_sample_was_lowered{false};
  std::atomic<bool> stale_refined_query{false};
  std::atomic<bool> replace_query_authority{false};
  std::atomic<bool> received_stop_at_unknown{true};
  std::vector<geometry_msgs::msg::Point> received;
  auto service = server->create_service<Query>(
      "/robot_1/mapping/query_batch",
      [&delay, &response_status, &ground_offset, &leading_root_ground_offset,
       &ground_from_sample,
       &received_body_x, &received_body_y, &received_body_z,
       &received_max_step, &received_max_drop, &sample_occupancy,
       &sample_clearance, &blind_ground_prefix, &blind_clearance_prefix,
       &blind_ground_sample, &step_sample, &drop_sample,
       &drop_at_or_after_x, &occupied_from_sample,
       &terrain_case, &refinement_dip, &compatible_refinement_dip,
       &refinement_from_x, &query_count, &first_query_sample_count,
       &occupy_refined_body, &occupy_if_early_sample_was_lowered,
       &stale_refined_query,
       &replace_query_authority, planner,
       &received_stop_at_unknown, &received](
          const Query::Request::SharedPtr request,
          Query::Response::SharedPtr response) {
        const std::size_t query_ordinal = ++query_count;
        if (query_ordinal == 1) {
          first_query_sample_count = request->samples.size();
        }
        received = request->samples;
        if (delay.load()) std::this_thread::sleep_for(200ms);
        response->status = response_status.load();
        response->component_id = request->component_id;
        response->epoch = request->epoch;
        response->graph_revision = request->graph_revision;
        response->geometry_revision = request->geometry_revision;
        const auto n = request->samples.size();
        received_body_x = request->body_size.x;
        received_body_y = request->body_size.y;
        received_body_z = request->body_size.z;
        received_max_step = request->max_step_m;
        received_max_drop = request->max_drop_m;
        received_stop_at_unknown = request->stop_at_unknown;
        response->occupancy.assign(n, sample_occupancy.load());
        response->ground_z.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
          const auto& sample = request->samples[i];
          const bool at_route_start_xy =
              n > 0 &&
              std::abs(sample.x - request->samples.front().x) <= 1e-9 &&
              std::abs(sample.y - request->samples.front().y) <= 1e-9;
          const int active_case = terrain_case.load();
          const bool two_interval_blind =
              active_case == kTwoBlindIntervals &&
              (i < 3 || (i >= 5 && i < 8));
          const bool overlong_blind =
              active_case == kOverlongInternalGap && i >= 3 && i < 13;
          const bool terminal_blind =
              (active_case == kTerminalGap && i >= 3) ||
              (active_case == kLateTerminalGap && i >= 8);
          const bool finite_root_then_blind =
              active_case == kFiniteRootThenBlind && i >= 2;
          const bool hazard_gap =
              active_case >= kOccupiedInsideGap &&
              active_case <= kDropInsideGap && i >= 3 && i < 5;
          const bool mismatch_gap =
              (active_case == kStepAcrossGap ||
               active_case == kClosingHeightMismatch) &&
              i >= 3 && i < 5;
          const bool scenario_ground_missing =
              two_interval_blind || overlong_blind || terminal_blind ||
              finite_root_then_blind || hazard_gap || mismatch_gap ||
              (active_case == kGroundOnlyMissing && i == 4);
          response->ground_z.push_back(
              (i < blind_ground_prefix.load() ||
               i == blind_ground_sample.load() || scenario_ground_missing)
              ? std::numeric_limits<double>::quiet_NaN()
              : sample.z - ground_from_sample.load() + ground_offset.load() +
                    (at_route_start_xy
                         ? leading_root_ground_offset.load()
                         : 0.0));
        }
        if (std::abs(refinement_dip.load()) > 0.0 && n > 0) {
          const double root_x = request->samples.front().x;
          for (std::size_t i = 0; i < n; ++i) {
            if (std::isfinite(response->ground_z[i])) {
              const bool needs_refinement =
                  request->samples[i].x >
                  root_x + refinement_from_x.load() + 1e-9;
              response->ground_z[i] =
                  -0.10 - (needs_refinement
                               ? refinement_dip.load()
                               : compatible_refinement_dip.load());
            }
          }
        }
        response->roughness.assign(n, 0.0);
        for (std::size_t i = 0; i < std::min(n, blind_ground_prefix.load()); ++i) {
          response->roughness[i] = std::numeric_limits<double>::quiet_NaN();
        }
        if (blind_ground_sample.load() < n) {
          response->roughness[blind_ground_sample.load()] =
              std::numeric_limits<double>::quiet_NaN();
        }
        const int active_case = terrain_case.load();
        const auto set_missing_roughness = [&response, n](std::size_t begin,
                                                          std::size_t end) {
          for (std::size_t i = begin; i < std::min(n, end); ++i) {
            response->roughness[i] =
                std::numeric_limits<double>::quiet_NaN();
          }
        };
        if (active_case == kTwoBlindIntervals) {
          set_missing_roughness(0, 3);
          set_missing_roughness(5, 8);
        } else if (active_case == kOverlongInternalGap) {
          set_missing_roughness(3, 13);
        } else if (active_case == kTerminalGap ||
                   active_case == kLateTerminalGap) {
          set_missing_roughness(
              active_case == kTerminalGap ? 3 : 8, n);
        } else if (active_case == kFiniteRootThenBlind) {
          set_missing_roughness(2, n);
        } else if ((active_case >= kOccupiedInsideGap &&
                    active_case <= kDropInsideGap) ||
                   active_case == kStepAcrossGap ||
                   active_case == kClosingHeightMismatch) {
          set_missing_roughness(3, 5);
        } else if (active_case == kRoughnessOnlyMissing && n > 4) {
          response->roughness[4] =
              std::numeric_limits<double>::quiet_NaN();
        }
        response->clearance.assign(n, sample_clearance.load());
        for (std::size_t i = 0;
             i < std::min(n, blind_clearance_prefix.load()); ++i) {
          response->clearance[i] =
              std::numeric_limits<double>::quiet_NaN();
        }
        response->step.assign(n, false);
        response->drop.assign(n, false);
        if (step_sample.load() < n) response->step[step_sample.load()] = true;
        if (drop_sample.load() < n) {
          response->drop[drop_sample.load()] = true;
        }
        for (std::size_t i = 0; i < n; ++i) {
          if (request->samples[i].x >=
              request->samples.front().x + drop_at_or_after_x.load()) {
            response->drop[i] = true;
          }
        }
        for (std::size_t i = occupied_from_sample.load(); i < n; ++i) {
          response->occupancy[i] = Query::Response::OCCUPIED;
        }
        if (n > 5 && active_case == kOccupiedInsideGap) {
          response->occupancy[4] = Query::Response::OCCUPIED;
        } else if (n > 5 && active_case == kLowClearanceInsideGap) {
          response->clearance[4] = 0.01;
        } else if (n > 5 && active_case == kRoughClosingSupport) {
          response->roughness[5] = 0.20;
        } else if (n > 5 && active_case == kStepInsideGap) {
          response->step[4] = true;
        } else if (n > 5 && active_case == kDropInsideGap) {
          response->drop[4] = true;
        } else if (n > 5 && active_case == kClosingHeightMismatch) {
          response->ground_z[5] += 0.20;
        }
        if (occupy_refined_body.load() && query_ordinal == 2 && n > 2) {
          response->occupancy[2] = Query::Response::OCCUPIED;
        }
        if (occupy_if_early_sample_was_lowered.load() &&
            query_ordinal == 2) {
          const double root_x = request->samples.front().x;
          for (std::size_t i = 0; i < n; ++i) {
            const double progress = request->samples[i].x - root_x;
            if (progress > 0.20 && progress < 0.40 &&
                request->samples[i].z <
                    request->samples.front().z - 0.05) {
              response->occupancy[i] = Query::Response::OCCUPIED;
            }
          }
        }
        if (stale_refined_query.load() && query_ordinal == 2) {
          ++response->graph_revision;
        }
        if (replace_query_authority.load()) {
          mgg_msgs::msg::MappingSnapshot changed;
          changed.component_id = request->component_id;
          changed.epoch = request->epoch;
          changed.graph_revision = request->graph_revision + 1;
          changed.geometry_revision = request->geometry_revision;
          changed.source_stamp = request->source_stamp;
          changed.component_from_navigation.rotation.w = 1.0;
          mgg_ros::PlannerNodeTestPeer::acceptSnapshot(*planner, changed);
        }
      });
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions{}, 3);
  executor.add_node(planner);
  executor.add_node(server);
  std::thread spinner([&executor]() { executor.spin(); });
  const auto discovery_deadline = std::chrono::steady_clock::now() + 3s;
  while (!mgg_ros::PlannerNodeTestPeer::queryReady(*planner) &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::queryReady(*planner));

  mgg_msgs::msg::MappingSnapshot snapshot;
  snapshot.component_id = "component-a";
  snapshot.epoch = 7;
  snapshot.graph_revision = 8;
  snapshot.geometry_revision = std::string(64, 'a');
  const auto stamp = planner->now();
  snapshot.source_stamp.sec = static_cast<std::int32_t>(stamp.seconds());
  snapshot.source_stamp.nanosec =
      static_cast<std::uint32_t>(stamp.nanoseconds() % 1000000000LL);
  snapshot.component_from_navigation.translation.x = 10.0;
  snapshot.component_from_navigation.rotation.w = 1.0;
  mgg_ros::PlannerNodeTestPeer::acceptSnapshot(*planner, snapshot);

  mgg::FeasiblePath path;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.component_id = snapshot.component_id;
  path.map_epoch = snapshot.epoch;
  path.mapping_graph_revision = snapshot.graph_revision;
  path.geometry_revision = snapshot.geometry_revision;
  path.map_source_stamp_sec = snapshot.source_stamp.sec;
  path.map_source_stamp_nanosec = snapshot.source_stamp.nanosec;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  mgg_ros::PlannerNodeTestPeer::setIndexedTerrainLimits(*planner, 0.10, 0.52);
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_TRUE(path.indexed_map_validated);
  EXPECT_TRUE(received_stop_at_unknown.load());
  EXPECT_NEAR(received_max_step.load(), 0.10, 1e-9);
  EXPECT_NEAR(received_max_drop.load(), 0.10, 1e-9);
  EXPECT_GE(received.size(), 2u);
  if (received.size() >= 2) {
    EXPECT_NEAR(received.front().x, 10.0, 1e-6);
    EXPECT_NEAR(received.back().x, 11.0, 1e-6);
    EXPECT_NEAR(received.front().z, 0.225, 1e-6);
  }

  // A snapshot that was fresh when planning began remains valid for that
  // bounded in-flight query even if graph construction consumed the remaining
  // receipt TTL. Exact authority identity is still checked after the query.
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(1.0, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(
      mgg_ros::PlannerNodeTestPeer::queryAfterCapturedSnapshotReceiptAges(
          *planner, path));
  EXPECT_TRUE(path.indexed_map_validated);

  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(1.0, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(
      mgg_ros::PlannerNodeTestPeer::queryAfterCapturedSnapshotIdentityChanges(
          *planner, path));
  EXPECT_EQ(path.status, mgg::PlanningStatus::kStaleRevision);
  EXPECT_NE(path.reason.find("authority changed during query"),
            std::string::npos);

  mgg_ros::PlannerNodeTestPeer::expireSnapshot(*planner);
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(1.0, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_EQ(path.status, mgg::PlanningStatus::kStaleRevision);
  EXPECT_NE(path.reason.find("authority is unavailable or expired"),
            std::string::npos);
  mgg_ros::PlannerNodeTestPeer::acceptSnapshot(*planner, snapshot);

  // Strict hardware policy still rejects unknown body occupancy and missing
  // overhead clearance even when the exact terrain arrays are complete.
  sample_occupancy = Query::Response::UNKNOWN;
  sample_clearance = std::numeric_limits<double>::quiet_NaN();
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(1.0, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_TRUE(received_stop_at_unknown.load());
  EXPECT_FALSE(path.indexed_map_validated);

  // The qualified simulation policy admits that same sparse body evidence,
  // while the exact query still owns ground, roughness, known clearance,
  // step/drop and occupied-cell vetoes before dispatch.
  mgg_ros::PlannerNodeTestPeer::setObservedGroundBodyEvidence(*planner, true);
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_FALSE(received_stop_at_unknown.load());
  EXPECT_TRUE(path.indexed_map_validated);

  // Body-only relaxation never relaxes terrain evidence by itself.
  blind_ground_prefix = std::numeric_limits<std::size_t>::max();
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_FALSE(path.indexed_map_validated);
  EXPECT_NE(path.reason.find("ground or roughness is unavailable"),
            std::string::npos);

  // The separately qualified provisional policy admits only bounded,
  // finite-supported sensor-blind connectors.
  mgg_ros::PlannerNodeTestPeer::setProvisionalUnknownGround(*planner, true);
  sample_occupancy = Query::Response::FREE;
  sample_clearance = 100.0;
  blind_ground_prefix = 0;

  // A MOLA root floor may be within the platform step cap but outside the
  // tighter fitted-height tolerance. Explore preserves the exact physical
  // start, including its duplicate root sample. A known obstacle still vetoes
  // it, and a different pure-Z pose receives the ordinary height rejection.
  mgg_ros::PlannerNodeTestPeer::qualifyMolaPolicyForTest(*planner);
  mgg_ros::PlannerNodeTestPeer::setMaxStep(*planner, 0.15);
  leading_root_ground_offset = -0.12;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  ASSERT_TRUE(mgg_ros::PlannerNodeTestPeer::query(
      *planner, path, /*allow_explore_height_refinement=*/true))
      << path.reason;
  ASSERT_TRUE(path.indexed_map_validated);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.front().z(), 0.0, 1e-9);

  sample_occupancy = Query::Response::OCCUPIED;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(
      *planner, path, /*allow_explore_height_refinement=*/true));
  EXPECT_NE(path.reason.find("occupied or unknown"), std::string::npos);
  sample_occupancy = Query::Response::FREE;

  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.05, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(
      *planner, path, /*allow_explore_height_refinement=*/true));
  EXPECT_NE(path.reason.find("does not support the emitted body height"),
            std::string::npos);
  leading_root_ground_offset = 0.0;
  mgg_ros::PlannerNodeTestPeer::setMaxStep(*planner, 0.10);

  // A fully measured stationary route never consumes the provisional policy.
  // It remains valid for generic Navigate/Home callers even though Explore
  // separately requires progress before dispatch.
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_TRUE(path.indexed_map_validated);

  // Fully observed slopes retain the ordinary adjacent-sample inclination
  // rule and strict callers retain 3-D sample spacing; the cross-gap step cap
  // must not apply when no gap was used.
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.30, 0.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_TRUE(path.indexed_map_validated);
  EXPECT_EQ(query_count.load(), 1u);
  EXPECT_EQ(received.size(), 7u);

  // A vertical, zero-XY route still receives the ordinary known-step veto.
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(0.0, 0.0, 0.30, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(
      *planner, path, /*allow_explore_height_refinement=*/true));
  EXPECT_NE(path.reason.find("step or inclination limits"), std::string::npos);

  blind_ground_prefix = 2;
  blind_clearance_prefix = 2;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_TRUE(path.indexed_map_validated);

  // Explore includes its local root in the path, after queryIndexedMap has
  // already prepended current_state. Finite duplicate root samples are exact
  // checked but cannot close the blind prefix until positive XY progress.
  blind_ground_prefix = 0;
  blind_clearance_prefix = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  blind_ground_sample = 2;
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_TRUE(path.indexed_map_validated);

  // A later gap is also admissible only when finite support brackets it within
  // the same connector bound.
  path.status = mgg::PlanningStatus::kSucceeded;
  blind_ground_sample = 3;
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_TRUE(path.indexed_map_validated);
  blind_ground_sample = std::numeric_limits<std::size_t>::max();
  blind_ground_prefix = 2;
  blind_clearance_prefix = 2;

  // Faithful sparse-lidar shape: a blind start, finite terrain, a second
  // bounded missing interval, and a finite endpoint. UNKNOWN body volume and
  // NaN clearance remain qualified only by the simulation policy.
  blind_ground_prefix = 0;
  blind_clearance_prefix = 0;
  terrain_case = kTwoBlindIntervals;
  sample_occupancy = Query::Response::UNKNOWN;
  sample_clearance = std::numeric_limits<double>::quiet_NaN();
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(2.4, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(
      *planner, path, /*allow_explore_height_refinement=*/true));
  EXPECT_TRUE(path.indexed_map_validated);
  ASSERT_EQ(received.size(), 10u);

  terrain_case = kOverlongInternalGap;
  sample_occupancy = Query::Response::FREE;
  sample_clearance = 100.0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(4.2, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_NE(path.reason.find("connector exceeds its bound"), std::string::npos);

  terrain_case = kTerminalGap;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_NE(path.reason.find("ends without measured terrain support"),
            std::string::npos);

  // Qualified Navigate/Home keeps its exact local endpoint when the indexed
  // authority reports a bounded unknown tail. The strict/default call above
  // rejects the same terrain evidence.
  sample_occupancy = Query::Response::UNKNOWN;
  sample_clearance = std::numeric_limits<double>::quiet_NaN();
  path.status = mgg::PlanningStatus::kSucceeded;
  path.partial = false;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.3),
                mgg::StateVec(1.2, 0.0, 0.0, -0.4)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::queryExplicit(*planner, path))
      << path.reason;
  EXPECT_TRUE(path.indexed_map_validated);
  EXPECT_FALSE(path.partial);
  EXPECT_FALSE(path.poses.empty());
  if (!path.poses.empty()) {
    EXPECT_NEAR(path.poses.back().x(), 1.2, 1e-9);
    EXPECT_NEAR(path.poses.back().y(), 0.0, 1e-9);
    EXPECT_NEAR(path.poses.back()[3], -0.4, 1e-9);
  }
  sample_occupancy = Query::Response::FREE;
  sample_clearance = 100.0;

  terrain_case = kFiniteRootThenBlind;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_NE(path.reason.find("ends without measured terrain support"),
            std::string::npos);

  // Physical provenance is sufficient for a wholly unknown bounded local
  // window. The existing connector limit remains hard.
  sample_occupancy = Query::Response::UNKNOWN;
  sample_clearance = std::numeric_limits<double>::quiet_NaN();
  path.status = mgg::PlanningStatus::kSucceeded;
  path.partial = false;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::queryExplicit(*planner, path))
      << path.reason;
  EXPECT_TRUE(path.indexed_map_validated);
  EXPECT_FALSE(path.partial);

  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(3.6, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::queryExplicit(*planner, path));
  EXPECT_NE(path.reason.find("connector exceeds its bound"), std::string::npos);
  sample_occupancy = Query::Response::FREE;
  sample_clearance = 100.0;

  const auto expect_gap_hazard = [&](int active_case,
                                     const char* expected_reason) {
    terrain_case = active_case;
    path.status = mgg::PlanningStatus::kSucceeded;
    path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                  mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
    EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
    EXPECT_FALSE(path.indexed_map_validated);
    EXPECT_NE(path.reason.find(expected_reason), std::string::npos)
        << "case " << active_case << ": " << path.reason;
    path.status = mgg::PlanningStatus::kSucceeded;
    path.partial = false;
    path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                  mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
    EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::queryExplicit(*planner, path));
    EXPECT_FALSE(path.indexed_map_validated);
    EXPECT_NE(path.reason.find(expected_reason), std::string::npos)
        << "explicit case " << active_case << ": " << path.reason;
  };
  expect_gap_hazard(kOccupiedInsideGap, "occupied or unknown");
  expect_gap_hazard(kLowClearanceInsideGap,
                    "clearance is below body height");
  expect_gap_hazard(kRoughClosingSupport, "roughness exceeds its limit");
  expect_gap_hazard(kStepInsideGap, "step reported");
  expect_gap_hazard(kDropInsideGap, "drop reported");
  expect_gap_hazard(kGroundOnlyMissing,
                    "ground or roughness is unavailable");
  expect_gap_hazard(kRoughnessOnlyMissing,
                    "ground or roughness is unavailable");
  terrain_case = kStepAcrossGap;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.30, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_NE(path.reason.find("step or inclination limits"), std::string::npos);
  expect_gap_hazard(kClosingHeightMismatch, "ground does not support");
  terrain_case = kOrdinaryTerrain;

  step_sample = 1;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_NE(path.reason.find("step reported"), std::string::npos);
  step_sample = std::numeric_limits<std::size_t>::max();

  blind_ground_prefix = std::numeric_limits<std::size_t>::max();
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));

  blind_ground_prefix = 2;
  drop_sample = 1;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_NE(path.reason.find("drop reported"), std::string::npos);
  drop_sample = std::numeric_limits<std::size_t>::max();
  blind_ground_prefix = 0;
  blind_clearance_prefix = 0;

  blind_ground_sample = 2;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_TRUE(path.indexed_map_validated);
  blind_ground_sample = std::numeric_limits<std::size_t>::max();

  // The same distance bound as the native physical-start connector prevents
  // a distant observation from licensing an arbitrarily long blind prefix.
  blind_ground_prefix = 12;
  blind_clearance_prefix = 12;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(4.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  blind_ground_prefix = 0;
  blind_clearance_prefix = 0;

  sample_occupancy = Query::Response::OCCUPIED;
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_FALSE(path.indexed_map_validated);
  sample_occupancy = Query::Response::UNKNOWN;
  sample_clearance = 0.01;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_FALSE(path.indexed_map_validated);
  EXPECT_NE(path.reason.find("clearance is below body height"),
            std::string::npos);

  sample_occupancy = Query::Response::FREE;
  sample_clearance = 100.0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));

  ground_offset = 0.101;
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_FALSE(path.indexed_map_validated);
  EXPECT_NE(path.reason.find("ground does not support"), std::string::npos);

  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(0.20, 0.0, 0.15, 0.0));
  ground_offset = 0.0;
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_FALSE(path.indexed_map_validated);
  EXPECT_NE(path.reason.find("step or inclination"), std::string::npos);

  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  ground_offset = 0.0;
  response_status = Query::Response::UNAVAILABLE;
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_FALSE(path.indexed_map_validated);
  EXPECT_EQ(path.status, mgg::PlanningStatus::kBlocked);

  mgg_ros::PlannerNodeTestPeer::configureIndexedRobotGeometry(
      *planner, Eigen::Vector3d(0.80, 0.50, 1.00),
      Eigen::Vector3d(0.05, 0.05, 0.05),
      Eigen::Vector3d(0.0, 0.0, 0.02), 0.975);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.0, 0.0, 0.50);
  snapshot.component_from_navigation.rotation.z = std::sin(M_PI / 4.0);
  snapshot.component_from_navigation.rotation.w = std::cos(M_PI / 4.0);
  mgg_ros::PlannerNodeTestPeer::acceptSnapshot(*planner, snapshot);
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.50, 0.0));
  response_status = Query::Response::OK;
  ground_from_sample = 0.995;
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_TRUE(path.indexed_map_validated);
  EXPECT_NEAR(received_body_x.load(), 0.55, 1e-6);
  EXPECT_NEAR(received_body_y.load(), 0.85, 1e-6);
  EXPECT_NEAR(received_body_z.load(), 1.05, 1e-6);
  EXPECT_FALSE(received.empty());
  if (!received.empty()) {
    EXPECT_NEAR(received.front().z, 0.995, 1e-6);
  }

  mgg_ros::PlannerNodeTestPeer::configureIndexedRobotGeometry(
      *planner, Eigen::Vector3d(0.80, 0.50, 1.00),
      Eigen::Vector3d(0.05, 0.05, 0.05),
      Eigen::Vector3d(0.10, 0.0, 0.02), 0.975);
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.50, 0.0));
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_FALSE(path.indexed_map_validated);
  EXPECT_NE(path.reason.find("gravity alignment is invalid"), std::string::npos);

  // MOLA's native projection may borrow one lateral floor ray and attach its
  // height to the route centre. The indexed authority fits the centre itself.
  // Explore may correct that bounded discrepancy exactly once, but only after
  // re-querying the body at the emitted corrected height.
  snapshot.component_from_navigation.rotation.z = 0.0;
  snapshot.component_from_navigation.rotation.w = 1.0;
  mgg_ros::PlannerNodeTestPeer::acceptSnapshot(*planner, snapshot);
  mgg_ros::PlannerNodeTestPeer::configureIndexedRobotGeometry(
      *planner, Eigen::Vector3d(0.40, 0.40, 0.20),
      Eigen::Vector3d(0.05, 0.05, 0.05),
      Eigen::Vector3d(0.0, 0.0, 0.05), 0.30);
  mgg_ros::PlannerNodeTestPeer::setIndexedTerrainLimits(*planner, 0.15, 0.52);
  mgg_ros::PlannerNodeTestPeer::acceptOdometry(*planner, 0.0, 0.0, 0.0);
  ground_from_sample = 0.35;
  ground_offset = 0.0;
  refinement_dip = 0.0;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 3.0),
                mgg::StateVec(1.2, 0.0, 0.0, -3.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true))
      << path.reason;
  EXPECT_EQ(query_count.load(), 1u);

  refinement_dip = 0.11;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 3.0),
                mgg::StateVec(1.2, 0.0, 0.0, -3.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_EQ(query_count.load(), 1u);

  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  path.speed_limits = {0.2, 0.2};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_EQ(query_count.load(), 1u);
  path.speed_limits.clear();

  query_count = 0;
  // A translated exact one-spacing interval must remain three samples (the
  // physical start, the explicit root, and the endpoint) on
  // both the initial query and the post-refinement recheck. Binary subtraction
  // makes (100.0 + 0.3) - 100.0 slightly larger than 0.3 on common platforms.
  mgg_ros::PlannerNodeTestPeer::setCurrentStateWithoutExtendingBackbone(
      *planner, 100.0, 0.0, 0.0);
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(100.0, 0.0, 0.0, 3.0),
                mgg::StateVec(100.3, 0.0, 0.0, -3.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true))
      << path.reason;
  EXPECT_TRUE(path.indexed_map_validated);
  EXPECT_EQ(query_count.load(), 2u);
  EXPECT_EQ(first_query_sample_count.load(), 3u);
  EXPECT_EQ(received.size(), 3u);
  EXPECT_EQ(received.size(), first_query_sample_count.load());
  EXPECT_NEAR(received.front().x, 110.0, 1e-9);
  EXPECT_NEAR(received[1].x, received.front().x, 1e-9);
  EXPECT_NEAR(received[2].x, received.front().x + 0.30, 1e-9);
  ASSERT_GE(path.poses.size(), 2u);
  EXPECT_NEAR(path.poses.front().x(), 100.0, 1e-9);
  EXPECT_NEAR(path.poses.front().z(), 0.0, 1e-9);
  EXPECT_NEAR(path.poses.front()[3], 3.0, 1e-9);
  EXPECT_NEAR(path.poses.back().x(), 100.3, 1e-9);
  EXPECT_NEAR(path.poses.back().y(), 0.0, 1e-9);
  EXPECT_NEAR(path.poses.back().z(), -0.010, 2e-6);
  EXPECT_NEAR(path.poses.back()[3], -3.0, 1e-9);
  mgg_ros::PlannerNodeTestPeer::setCurrentStateWithoutExtendingBackbone(
      *planner, 0.0, 0.0, 0.0);

  refinement_dip = -0.11;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true))
      << path.reason;
  EXPECT_EQ(query_count.load(), 2u);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().z(), 0.010, 2e-6);
  refinement_dip = 0.11;

  // Explicit objectives may use the same bounded terrain-height correction,
  // but must retain the requested endpoint and may not become an Explore
  // prefix when indexed evidence is sparse.
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.partial = false;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.25),
                mgg::StateVec(1.2, 0.0, 0.0, -0.35)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::queryExplicit(*planner, path))
      << path.reason;
  EXPECT_EQ(query_count.load(), 2u);
  EXPECT_FALSE(path.partial);
  EXPECT_FALSE(path.poses.empty());
  if (!path.poses.empty()) {
    EXPECT_NEAR(path.poses.back().x(), 1.2, 1e-9);
    EXPECT_NEAR(path.poses.back().y(), 0.0, 1e-9);
    EXPECT_NEAR(path.poses.back()[3], -0.35, 1e-9);
  }

  // A later fitted sample must not rewrite earlier poses whose emitted
  // heights already agreed with the indexed terrain. The second response
  // simulates a voxel that would collide only if that early sample were
  // incorrectly lowered with the later correction.
  refinement_from_x = 0.60;
  compatible_refinement_dip = 0.08;
  occupy_if_early_sample_was_lowered = true;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true))
      << path.reason;
  EXPECT_EQ(query_count.load(), 2u);
  ASSERT_FALSE(received.empty());
  const double mixed_root_x = received.front().x;
  const double mixed_root_z = received.front().z;
  bool saw_unchanged_early_sample = false;
  for (const auto& sample : received) {
    const double progress = sample.x - mixed_root_x;
    if (progress > 0.20 && progress < 0.40) {
      EXPECT_NEAR(sample.z, mixed_root_z, 1e-9);
      saw_unchanged_early_sample = true;
    }
  }
  EXPECT_TRUE(saw_unchanged_early_sample);
  occupy_if_early_sample_was_lowered = false;

  // The production failure combined a blind lidar prefix, a fitted-height
  // correction, and a later sparse interval. The corrected dense route must
  // preserve its explicit physical-root pose, interpolate the bounded gap,
  // and validate the new body heights with exactly one additional query.
  terrain_case = kTwoBlindIntervals;
  refinement_from_x = 1.50;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 2.8),
                mgg::StateVec(3.0, 0.0, 0.0, -2.8)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_EQ(query_count.load(), 2u);
  ASSERT_GE(path.poses.size(), 10u);
  EXPECT_NEAR(path.poses.front().x(), 0.0, 1e-9);
  EXPECT_NEAR(path.poses.front().y(), 0.0, 1e-9);
  EXPECT_NEAR(path.poses.front().z(), 0.0, 1e-9);
  EXPECT_NEAR(path.poses.front()[3], 2.8, 1e-9);
  EXPECT_NEAR(path.poses.back().x(), 3.0, 1e-9);
  EXPECT_NEAR(path.poses.back().z(), -0.010, 2e-6);
  EXPECT_NEAR(path.poses.back()[3], -2.8, 1e-9);
  bool saw_unchanged_gap_anchor = false;
  for (const auto& pose : path.poses) {
    EXPECT_TRUE(pose.allFinite());
    if (std::abs(pose.x() - 0.9) <= 1e-9) {
      EXPECT_NEAR(pose.z(), 0.0, 1e-9);
      saw_unchanged_gap_anchor = true;
    }
  }
  EXPECT_TRUE(saw_unchanged_gap_anchor);
  compatible_refinement_dip = 0.0;
  refinement_from_x = 0.0;

  terrain_case = kTerminalGap;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(3.0, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_EQ(query_count.load(), 1u);
  EXPECT_NE(path.reason.find("ends without measured terrain support"),
            std::string::npos);
  terrain_case = kOrdinaryTerrain;

  // A later known hazard may shorten qualified Explore to the last exact,
  // fully checked measured-ground sample. Strict callers still reject it,
  // and a hazard before the configured useful-progress bound cannot move the
  // robot merely to manufacture map coverage.
  refinement_dip = 0.0;
  drop_sample = 6;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(3.0, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_EQ(query_count.load(), 1u);

  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(3.0, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_TRUE(path.partial);
  EXPECT_TRUE(path.indexed_map_validated);
  EXPECT_EQ(query_count.load(), 1u);
  ASSERT_FALSE(path.poses.empty());
  ASSERT_GT(received.size(), drop_sample.load());
  EXPECT_NEAR(path.poses.back().x(), 1.2, 1e-9);
  EXPECT_LT(path.poses.back().x(), received[drop_sample.load()].x);
  for (const auto& pose : path.poses) EXPECT_TRUE(pose.allFinite());

  drop_sample = 4;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(3.0, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_EQ(query_count.load(), 1u);

  // Cumulative travel cannot qualify a loop whose every possible endpoint
  // remains within the useful-progress radius of the physical start.
  drop_sample = 5;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(0.6, 0.0, 0.0, 0.0),
                mgg::StateVec(0.0, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_EQ(query_count.load(), 1u);

  // Even an otherwise reusable exact prefix fails closed if the mapping
  // authority changes before publication.
  drop_sample = 6;
  replace_query_authority = true;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(3.0, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_EQ(query_count.load(), 1u);
  EXPECT_EQ(path.status, mgg::PlanningStatus::kStaleRevision);
  replace_query_authority = false;
  mgg_ros::PlannerNodeTestPeer::acceptSnapshot(*planner, snapshot);
  drop_sample = std::numeric_limits<std::size_t>::max();

  terrain_case = kLateTerminalGap;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(3.0, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_TRUE(path.partial);
  EXPECT_EQ(query_count.load(), 1u);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 1.8, 1e-9);

  // A committed Navigate/Home section may outrun the measured floor. Its
  // validated prefix is drivable, so the section is shortened to the last fully
  // checked measured sample and reported partial; the strict Navigate/Home
  // caller still rejects exactly the same terrain evidence outright.
  const auto committed_section = [&](double reach) {
    path.status = mgg::PlanningStatus::kSucceeded;
    path.partial = false;
    path.indexed_map_validated = false;
    path.reason.clear();
    path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                  mgg::StateVec(reach, 0.0, 0.0, 0.0)};
  };
  bool truncated = false;
  query_count = 0;
  committed_section(6.0);
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::queryExplicit(*planner, path));
  EXPECT_NE(path.reason.find("connector exceeds its bound"), std::string::npos);

  query_count = 0;
  committed_section(6.0);
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::queryContinuable(*planner, path,
                                                             &truncated))
      << path.reason;
  EXPECT_TRUE(truncated);
  EXPECT_TRUE(path.partial);
  EXPECT_TRUE(path.indexed_map_validated);
  EXPECT_TRUE(path.reason.empty());
  EXPECT_EQ(query_count.load(), 1u);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 1.8, 1e-9);
  for (const auto& pose : path.poses) EXPECT_TRUE(pose.allFinite());

  // The provisional-ground policy is not a precondition. Any ground platform
  // may drive the part of its section that measured terrain already supports.
  truncated = false;
  query_count = 0;
  committed_section(6.0);
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::queryContinuable(
      *planner, path, &truncated, /*qualified_ground=*/false))
      << path.reason;
  EXPECT_TRUE(truncated);
  EXPECT_TRUE(path.partial);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 1.8, 1e-9);
  terrain_case = kOrdinaryTerrain;

  // Known impassable terrain is not a horizon. An occupied tail fails the whole
  // committed section closed, while qualified Explore keeps its own policy of
  // shortening to the last checked sample after any rejection.
  occupied_from_sample = 8;
  truncated = true;
  query_count = 0;
  committed_section(6.0);
  EXPECT_FALSE(
      mgg_ros::PlannerNodeTestPeer::queryContinuable(*planner, path,
                                                     &truncated));
  EXPECT_FALSE(truncated);
  EXPECT_FALSE(path.partial);
  EXPECT_FALSE(path.indexed_map_validated);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_NE(path.reason.find("occupied or unknown"), std::string::npos);

  query_count = 0;
  committed_section(6.0);
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true))
      << path.reason;
  EXPECT_TRUE(path.partial);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 1.8, 1e-9);
  occupied_from_sample = std::numeric_limits<std::size_t>::max();

  // A measured floor that ends before the useful-progress bound leaves no
  // prefix worth driving, so the exact terrain rejection stands.
  terrain_case = kTerminalGap;
  truncated = true;
  query_count = 0;
  committed_section(6.0);
  EXPECT_FALSE(
      mgg_ros::PlannerNodeTestPeer::queryContinuable(*planner, path,
                                                     &truncated));
  EXPECT_FALSE(truncated);
  EXPECT_TRUE(path.poses.empty());
  EXPECT_NE(path.reason.find("connector exceeds its bound"), std::string::npos);
  terrain_case = kOrdinaryTerrain;

  // A prefix containing corrected fitted ground is not complete after the
  // first response. It must consume the sole refinement retry and validate
  // the corrected body before publication.
  refinement_dip = 0.11;
  drop_at_or_after_x = 1.5;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(3.0, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true))
      << path.reason;
  EXPECT_TRUE(path.partial);
  EXPECT_EQ(query_count.load(), 2u);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.back().x(), 1.2, 1e-9);
  EXPECT_NEAR(path.poses.back().z(), -0.010, 2e-6);
  drop_at_or_after_x = std::numeric_limits<double>::infinity();
  drop_sample = std::numeric_limits<std::size_t>::max();

  refinement_dip = 0.16;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_EQ(query_count.load(), 1u);

  refinement_dip = 0.11;
  occupy_refined_body = true;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_EQ(query_count.load(), 2u);
  EXPECT_NE(path.reason.find("occupied or unknown"), std::string::npos);
  occupy_refined_body = false;

  stale_refined_query = true;
  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(0.0, 0.0, 0.0, 0.0),
                mgg::StateVec(1.2, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_EQ(query_count.load(), 2u);
  EXPECT_EQ(path.status, mgg::PlanningStatus::kStaleRevision);
  stale_refined_query = false;
  refinement_dip = 0.0;

  query_count = 0;
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses = {mgg::StateVec(2000.0, 0.0, 0.0, 0.0)};
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path, true));
  EXPECT_EQ(query_count.load(), 0u);
  EXPECT_NE(path.reason.find("sample budget exceeded"), std::string::npos);

  mgg_ros::PlannerNodeTestPeer::configureIndexedRobotGeometry(
      *planner, Eigen::Vector3d(0.80, 0.50, 1.00),
      Eigen::Vector3d(0.05, 0.05, 0.05),
      Eigen::Vector3d(0.0, 0.0, 0.02), 0.975);
  path.status = mgg::PlanningStatus::kSucceeded;
  path.poses.push_back(mgg::StateVec(1.0, 0.0, 0.0, 0.0));
  response_status = Query::Response::OK;
  delay = true;
  EXPECT_FALSE(mgg_ros::PlannerNodeTestPeer::query(*planner, path));
  EXPECT_FALSE(path.indexed_map_validated);
  EXPECT_EQ(path.status, mgg::PlanningStatus::kBlocked);
  executor.cancel();
  spinner.join();
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
