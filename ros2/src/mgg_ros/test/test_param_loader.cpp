// Tests for the ROS 2 parameter boundary.

#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "mgg_ros/param_loader.h"

namespace {

using mgg_ros::ParamLoader;

class ParamFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions opts;
    // Mirrors how the planner node is constructed: anything present in the
    // YAML is declared for us, so the loader can read nested names without
    // declaring 200 parameters by hand.
    opts.automatically_declare_parameters_from_overrides(true);
    opts.parameter_overrides({
        {"PlanningParams.edge_length_max", 3.0},
        {"PlanningParams.edge_length_min", 0.75},
        {"PlanningParams.build_grid_local_graph", true},
        {"PlanningParams.global_frame_id", std::string("R1/world")},
        {"PlanningParams.type", std::string("kAdaptiveExploration")},
        {"PlanningParams.viewpoint_clearance_margin", 0.15},
        {"PlanningParams.max_cross_slope", 0.1745},
        {"PlanningParams.max_footprint_tilt", 0.3491},
        {"PlanningParams.max_footprint_step", 0.12},
        {"PlanningParams.departure_reverse_allowed", false},
        {"PlanningParams.max_goal_ground_rise", 3.5},
        {"RobotParams.type", std::string("kGroundRobot")},
        {"RobotParams.size", std::vector<double>{0.8, 0.8, 0.2}},
        {"RobotParams.bound_mode", std::string("kExtendedBound")},
        {"BoundedSpaceParams.GridGraphLocal.min_val",
         std::vector<double>{-10.0, -10.0, -1.0}},
        {"BoundedSpaceParams.GridGraphLocal.max_val",
         std::vector<double>{10.0, 10.0, 0.2}},
        {"BoundedSpaceParams.GridGraphLocal.resolution",
         std::vector<double>{0.5, 0.5, 0.2}},
        {"SensorParams.sensor_list", std::vector<std::string>{"VLP16"}},
        {"SensorParams.VLP16.type", std::string("kLidar")},
        {"SensorParams.VLP16.max_range", 20.0},
        {"SensorParams.VLP16.fov", std::vector<double>{6.283185307179586,
                                                       0.5235987755982988}},
        {"SensorParams.VLP16.resolution",
         std::vector<double>{0.08726646259971647, 0.08726646259971647}},
        // A rad() expression that was never evaluated: what an unconverted
        // ROS 1 config looks like from ROS 2's side.
        {"Broken.angle", std::string("rad(2.0*pi)")},
    });
    node_ = std::make_shared<rclcpp::Node>("param_test", opts);
  }
  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(ParamFixture, ReadsNestedNamesUsingTheRos1Spelling) {
  ParamLoader p(node_.get());
  double v = 0.0;
  // Written with '/', as every ported call site does.
  EXPECT_TRUE(p.get("PlanningParams/edge_length_max", v));
  EXPECT_DOUBLE_EQ(v, 3.0);
}

TEST_F(ParamFixture, MissingParameterLeavesTheDefaultAlone) {
  ParamLoader p(node_.get());
  double v = 42.0;
  EXPECT_FALSE(p.get("PlanningParams/not_a_real_parameter", v));
  EXPECT_DOUBLE_EQ(v, 42.0);
  EXPECT_FALSE(p.missing().empty());
}

TEST_F(ParamFixture, LoadsPlanningParams) {
  ParamLoader p(node_.get());
  mgg::PlanningParams params;
  ASSERT_TRUE(mgg_ros::loadPlanningParams(p, "PlanningParams", params));
  EXPECT_DOUBLE_EQ(params.edge_length_max, 3.0);
  EXPECT_DOUBLE_EQ(params.edge_length_min, 0.75);
  EXPECT_TRUE(params.build_grid_local_graph);
  EXPECT_EQ(params.global_frame_id, "R1/world");
  EXPECT_EQ(params.type, mgg::PlanningModeType::kAdaptiveExploration);
  EXPECT_DOUBLE_EQ(params.viewpoint_clearance_margin, 0.15);
  EXPECT_DOUBLE_EQ(params.max_cross_slope, 0.1745);
  EXPECT_DOUBLE_EQ(params.max_footprint_tilt, 0.3491);
  EXPECT_DOUBLE_EQ(params.max_footprint_step, 0.12);
  EXPECT_FALSE(params.departure_reverse_allowed);
  EXPECT_DOUBLE_EQ(params.max_goal_ground_rise, 3.5);
  // Absent from the overrides, so the struct default survives.
  EXPECT_DOUBLE_EQ(params.v_max, 0.2);
}

TEST_F(ParamFixture, LoadsRobotParams) {
  ParamLoader p(node_.get());
  mgg::RobotParams robot;
  ASSERT_TRUE(mgg_ros::loadRobotParams(p, "RobotParams", robot));
  EXPECT_EQ(robot.type, mgg::RobotType::kGroundRobot);
  EXPECT_TRUE(robot.size.isApprox(Eigen::Vector3d(0.8, 0.8, 0.2)));
  EXPECT_EQ(robot.bound_mode, mgg::BoundModeType::kExtendedBound);
}

TEST_F(ParamFixture, LoadsGridGraphParamsFromTheNamedResolutionField) {
  ParamLoader p(node_.get());
  mgg::GridGraphParams grid;
  ASSERT_TRUE(mgg_ros::loadGridGraphParams(
      p, "BoundedSpaceParams/GridGraphLocal", grid));
  EXPECT_TRUE(grid.resolution.isApprox(Eigen::Vector3d(0.5, 0.5, 0.2)));
  EXPECT_TRUE(grid.min_val.isApprox(Eigen::Vector3d(-10.0, -10.0, -1.0)));
}

TEST_F(ParamFixture, LoadsSensorsAndBuildsTheRayTable) {
  ParamLoader p(node_.get());
  std::unordered_map<std::string, mgg::SensorParams> sensors;
  ASSERT_TRUE(mgg_ros::loadSensorSet(p, "SensorParams", sensors));
  ASSERT_EQ(sensors.count("VLP16"), 1u);
  const mgg::SensorParams& s = sensors.at("VLP16");
  // loadSensorParams must call update(): skipping it is what left the ROS 1
  // struct with an uninitialised rotation.
  EXPECT_TRUE(s.isReady());
  EXPECT_DOUBLE_EQ(s.max_range, 20.0);
}

// An unconverted ROS 1 config reaches ROS 2 with rad()/deg() still as strings.
// That must be reported, not silently read as zero.
TEST_F(ParamFixture, UnevaluatedAngleExpressionIsRejected) {
  ParamLoader p(node_.get());
  double v = -1.0;
  EXPECT_FALSE(p.get("Broken/angle", v));
  EXPECT_DOUBLE_EQ(v, -1.0);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
