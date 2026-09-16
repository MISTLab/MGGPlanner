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
                   const std::string& landmark_id = "") {
    auto request = std::make_shared<mgg_msgs::srv::PlanObjective::Request>();
    request->objective = static_cast<std::uint8_t>(objective);
    request->component_id = node.component_id_;
    request->goal.position.x = goal.x();
    request->goal.position.y = goal.y();
    request->goal.position.z = goal.z();
    request->goal.orientation.z = std::sin(goal[3] / 2.0);
    request->goal.orientation.w = std::cos(goal[3] / 2.0);
    request->goal_landmark_id = landmark_id;
    auto response =
        std::make_shared<mgg_msgs::srv::PlanObjective::Response>();
    node.onObjectiveRequest(request, response);
    return response;
  }

  static mgg::StateVec globalVertexState(const PlannerNode& node, int id) {
    return node.global_graph_->getVertex(id)->state;
  }

  static bool initialAnchorSupported(const PlannerNode& node) {
    return node.initial_anchor_supported_;
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

  static bool query(PlannerNode& node, mgg::FeasiblePath& path) {
    PlannerNode::IndexedQueryContext context;
    {
      const std::lock_guard<std::recursive_mutex> lock(node.planner_mutex_);
      context = node.indexedQueryContext();
    }
    return node.queryIndexedMap(path, context);
  }

  static bool queryReady(const PlannerNode& node) {
    return node.indexed_map_client_ && node.indexed_map_client_->service_is_ready();
  }

  static bool hasQueryClient(const PlannerNode& node) {
    return static_cast<bool>(node.indexed_map_client_);
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

TEST(PlannerConfiguration, MolaBackendRequiresExactRouteValidator) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("map.backend", "mola_snapshot"),
       rclcpp::Parameter("map.mola.peer_root", "/tmp")});
  EXPECT_THROW(
      { auto planner = std::make_shared<mgg_ros::PlannerNode>(options); },
      std::invalid_argument);
}

TEST(PlannerConfiguration, ObservedGroundPolicyIsExplicitAndSimulationCloudOnly) {
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
  EXPECT_THROW(
      { auto planner = std::make_shared<mgg_ros::PlannerNode>(mola); },
      std::invalid_argument);
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
  EXPECT_EQ(Peer::refine(*curb, corridor(2.5)).status,
            mgg::PlanningStatus::kBlocked);

  auto beyond = make(3.25);
  const mgg::FeasiblePath refused = Peer::refine(*beyond, corridor(3.25));
  EXPECT_EQ(refused.status, mgg::PlanningStatus::kBlocked);
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
  Peer::addMeasuredSurface(*flat, 0.20, 0.10, 0.125);
  EXPECT_FALSE(Peer::footprintTerrainSupported(*flat, driving_pose, body));

  auto missing_corner = make();
  Peer::observeGroundRectangle(*missing_corner, -0.05, 0.05, -0.05, 0.05);
  // Sparse lateral evidence is neutral; ordinary objective projection still
  // requires known support on the centreline.
  EXPECT_TRUE(
      Peer::footprintTerrainSupported(*missing_corner, driving_pose, body));

  // A fully observed shallow surface remains within the configured 10 cm
  // step budget across the complete footprint.
  auto shallow = make();
  Peer::observeShallowRamp(*shallow);
  EXPECT_TRUE(Peer::footprintTerrainSupported(
      *shallow, Eigen::Vector3d(0.0, 0.0, 0.325), body));

  EXPECT_FALSE(Peer::footprintTerrainSupported(
      *shallow, driving_pose, Eigen::Vector3d(10.0, 10.0, 0.15)));
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

TEST(PlannerObjective, NavigateCompletesThirtyMetreKnownRoadExactly) {
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
  EXPECT_FALSE(response->partial);
  ASSERT_FALSE(response->path.empty());
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

TEST_F(ObjectiveService, NewObstacleBlocksExplicitHomeThroughActualService) {
  mgg_ros::PlannerNodeTestPeer::configureExploreServiceScene(
      *planner, /*support_root=*/true);
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
  EXPECT_EQ(response->status, Service::Response::BLOCKED);
  EXPECT_TRUE(response->path.empty());
  EXPECT_NE(response->reason.find("route corridor waypoint["),
            std::string::npos);
  EXPECT_NE(response->reason.find("body intersects occupied space at"),
            std::string::npos);
}

TEST_F(ObjectiveService, GridHomeDetoursOnObservedGroundAndBlocksWall) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureGridServiceScene(*planner);
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
}

TEST_F(ObjectiveService, GridBodyOffsetPreservesHeightAndChecksRaisedObstacle) {
  using Peer = mgg_ros::PlannerNodeTestPeer;
  Peer::configureGridServiceScene(*planner, 0.30, 0.0);
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
  std::atomic<double> ground_from_sample{0.30};
  std::atomic<double> received_body_x{0.0};
  std::atomic<double> received_body_y{0.0};
  std::atomic<double> received_body_z{0.0};
  std::vector<geometry_msgs::msg::Point> received;
  auto service = server->create_service<Query>(
      "/robot_1/mapping/query_batch",
      [&delay, &response_status, &ground_offset, &ground_from_sample,
       &received_body_x, &received_body_y, &received_body_z, &received](
          const Query::Request::SharedPtr request,
          Query::Response::SharedPtr response) {
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
        response->occupancy.assign(n, Query::Response::FREE);
        response->ground_z.reserve(n);
        for (const auto& sample : request->samples) {
          response->ground_z.push_back(
              sample.z - ground_from_sample.load() + ground_offset.load());
        }
        response->roughness.assign(n, 0.0);
        response->clearance.assign(n, 100.0);
        response->step.assign(n, false);
        response->drop.assign(n, false);
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
  EXPECT_GE(received.size(), 2u);
  if (received.size() >= 2) {
    EXPECT_NEAR(received.front().x, 10.0, 1e-6);
    EXPECT_NEAR(received.back().x, 11.0, 1e-6);
    EXPECT_NEAR(received.front().z, 0.225, 1e-6);
  }

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
