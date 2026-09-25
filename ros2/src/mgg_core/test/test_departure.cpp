// Tests for the body check of a boxed-in robot's departure.

#include <cmath>
#include <functional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/departure.h"

namespace {

using mgg::OrientedBox;
using mgg::StateVec;
using mgg::VoxelStatus;

/// Wall columns, 0 to 2 m tall, on the map's 0.2 m cells, and nothing else
/// mapped. Only what orientedBoxPathStatus asks is answered: cell centres,
/// and whether a column of a cell is occupied.
class Columns : public mgg::MapInterface {
 public:
  explicit Columns(std::function<bool(double, double)> wall)
      : wall_(std::move(wall)) {}
  double getResolution() const override { return 0.2; }
  bool getStatus() const override { return true; }
  bool getAxisAlignedXYCellCenter(const Eigen::Vector2d& p,
                                  Eigen::Vector2d& center) const override {
    center = ((p / 0.2).array().floor() + 0.5) * 0.2;
    return true;
  }
  VoxelStatus getVoxelStatus(const Eigen::Vector3d&) const override {
    return VoxelStatus::kFree;
  }
  VoxelStatus getBoxStatus(const Eigen::Vector3d& c, const Eigen::Vector3d& s,
                           bool) const override {
    ++columns_checked;
    return wall_(c.x(), c.y()) && c.z() - s.z() / 2 <= 2.0 &&
                   c.z() + s.z() / 2 >= 0.0
               ? VoxelStatus::kOccupied
               : VoxelStatus::kFree;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d&, const Eigen::Vector3d&,
                           bool) const override {
    return VoxelStatus::kFree;
  }
  VoxelStatus getRayStatus(const Eigen::Vector3d&, const Eigen::Vector3d& b,
                           bool, Eigen::Vector3d& e) const override {
    e = b;
    return VoxelStatus::kFree;
  }
  VoxelStatus getPathStatus(const Eigen::Vector3d&, const Eigen::Vector3d&,
                            const Eigen::Vector3d&, bool) const override {
    return VoxelStatus::kFree;
  }
  void getScanStatus(const Eigen::Vector3d&,
                     const std::vector<Eigen::Vector3d>&, mgg::GainCounts&,
                     std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
                     const mgg::SensorModel&) override {}
  void getScanStatusIterative(
      const Eigen::Vector3d&, const std::vector<Eigen::Vector3d>&,
      mgg::GainCounts&, std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>&,
      const mgg::SensorModel&) override {}
  bool augmentFreeBox(const Eigen::Vector3d&,
                      const Eigen::Vector3d&) override {
    return true;
  }
  void augmentFreeFrustum() override {}
  void resetMap() override {}
  void extractLocalMap(const Eigen::Vector3d&, const Eigen::Vector3d&,
                       std::vector<Eigen::Vector3d>&,
                       std::vector<Eigen::Vector3d>&) override {}
  void extractLocalMapAlongAxis(const Eigen::Vector3d&, const Eigen::Vector3d&,
                                const Eigen::Vector3d&,
                                std::vector<Eigen::Vector3d>&,
                                std::vector<Eigen::Vector3d>&) override {}
  void getLocalPointcloud(const Eigen::Vector3d&, double, double,
                          std::vector<Eigen::Vector3d>&, bool) override {}
  void getFreeSpacePointCloud(const std::vector<Eigen::Vector3d>&,
                              const StateVec&,
                              std::vector<Eigen::Vector3d>&) override {}
  void setRaycastingParams(bool, double) override {}
  void setRobotRadius(double) override {}

  mutable int columns_checked = 0;

 private:
  std::function<bool(double, double)> wall_;
};

/// A Bunker's planning box, 1.073 x 0.828 m and 0.45 m tall, facing
/// `heading` at (x, y), 0.5 m up.
OrientedBox bunkerAt(double x, double y, double heading) {
  OrientedBox box;
  box.center = Eigen::Vector3d(x, y, 0.5);
  box.heading = heading;
  box.size = Eigen::Vector3d(1.073, 0.828, 0.45);
  return box;
}

TEST(OrientedBoxPathStatus, ChecksTheBodyTurnedToItsHeadingNotTheBoxThatHoldsIt) {
  // A Bunker facing 45 degrees, a wall cell 0.71 m off its flank at
  // (0.5, -0.5) and another 0.85 m ahead on its axis at (0.6, 0.6). The
  // box aligned with the map that holds the turned body, 1.35 m square,
  // meets the flank's cell; the body does not.
  const Columns map([](double x, double y) {
    return (std::abs(x - 0.5) < 0.05 && std::abs(y + 0.5) < 0.05) ||
           (std::abs(x - 0.7) < 0.05 && std::abs(y - 0.7) < 0.05);
  });
  const OrientedBox body = bunkerAt(0.0, 0.0, M_PI / 4.0);
  EXPECT_EQ(mgg::orientedBoxPathStatus(map, body.center, body.center, body,
                                       true, nullptr),
            VoxelStatus::kFree);
  const double c = std::cos(M_PI / 4.0);
  OrientedBox turned = bunkerAt(0.0, 0.0, 0.0);
  turned.size.x() = turned.size.y() = c * 1.073 + c * 0.828;
  EXPECT_TRUE(
      mgg::cellMeetsBox(Eigen::Vector2d(0.5, -0.5), 0.2, turned, -1e-9));
  // Driven along its heading, its front reaches the cell ahead...
  const Eigen::Vector3d ahead =
      body.center + 0.4 * Eigen::Vector3d(c, c, 0.0);
  EXPECT_EQ(mgg::orientedBoxPathStatus(map, body.center, ahead, body, true,
                                       nullptr),
            VoxelStatus::kOccupied);
  // ...and backed away, its flank stays clear of the other.
  const Eigen::Vector3d back = body.center - 0.8 * Eigen::Vector3d(c, c, 0.0);
  EXPECT_EQ(mgg::orientedBoxPathStatus(map, body.center, back, body, true,
                                       nullptr),
            VoxelStatus::kFree);
}

TEST(OrientedBoxPathStatus, CellsUnderTheStandingBodyAreNotChecked) {
  // A wall column under the front of a Bunker facing +x, where the map
  // puts it although the robot stands there, and a wall 1 m ahead of its
  // front.
  const Columns map([](double x, double y) {
    const bool under = std::abs(x - 0.5) < 0.1 && std::abs(y - 0.3) < 0.1;
    const bool ahead = x > 1.45;
    return under || ahead;
  });
  const OrientedBox standing = bunkerAt(0.0, 0.0, 0.0);
  EXPECT_EQ(mgg::orientedBoxPathStatus(map, standing.center, standing.center,
                                       standing, true, nullptr),
            VoxelStatus::kOccupied);
  EXPECT_EQ(mgg::orientedBoxPathStatus(map, standing.center, standing.center,
                                       standing, true, &standing),
            VoxelStatus::kFree);
  // Back out, and ahead up to the wall: that column stays unchecked...
  const Eigen::Vector3d back = standing.center - Eigen::Vector3d(0.8, 0, 0);
  EXPECT_EQ(mgg::orientedBoxPathStatus(map, standing.center, back, standing,
                                       true, &standing),
            VoxelStatus::kFree);
  const Eigen::Vector3d ahead = standing.center + Eigen::Vector3d(0.4, 0, 0);
  EXPECT_EQ(mgg::orientedBoxPathStatus(map, standing.center, ahead, standing,
                                       true, &standing),
            VoxelStatus::kFree);
  // ...and a wall past the standing body is met.
  const Eigen::Vector3d into = standing.center + Eigen::Vector3d(1.0, 0, 0);
  EXPECT_EQ(mgg::orientedBoxPathStatus(map, standing.center, into, standing,
                                       true, &standing),
            VoxelStatus::kOccupied);
}

TEST(OrientedBoxPathStatus, ACellTouchingTheStandingBodyIsStillChecked) {
  // A wall whose face touches the standing body's front, at x = 0.6 for a
  // 1.2 m body: the robot stands against it, not in it, so driving ahead
  // meets it.
  const Columns map([](double x, double) { return x > 0.6; });
  OrientedBox standing = bunkerAt(0.0, 0.0, 0.0);
  standing.size.x() = 1.2;
  EXPECT_EQ(mgg::orientedBoxPathStatus(map, standing.center,
                                       standing.center +
                                           Eigen::Vector3d(0.1, 0, 0),
                                       standing, true, &standing),
            VoxelStatus::kOccupied);
}

TEST(OrientedBoxPathStatus, APostJustPastTheStandingBodysFrontIsChecked) {
  // Review r0, I-1: a one-cell post centred 0.07 m past the front of a
  // Bunker's planning box. Its square reaches 0.03 m into the box, but its
  // centre lies outside: the robot does not stand on it, and driving ahead
  // meets it.
  const Columns map([](double x, double y) {
    return std::abs(x - 0.7) < 0.05 && std::abs(y - 0.1) < 0.05;
  });
  for (const double past : {0.05, 0.07, 0.09}) {
    SCOPED_TRACE(past);
    const OrientedBox standing = bunkerAt(0.7 - past - 0.5365, 0.1, 0.0);
    EXPECT_TRUE(mgg::cellMeetsBox(Eigen::Vector2d(0.7, 0.1), 0.2, standing,
                                  1e-6));
    const Eigen::Vector3d ahead =
        standing.center + Eigen::Vector3d(0.25, 0.0, 0.0);
    EXPECT_EQ(mgg::orientedBoxPathStatus(map, standing.center, ahead,
                                         standing, true, &standing),
              VoxelStatus::kOccupied);
    // Backing away from it is free: the body reaches into the post's cell
    // only where it already stands.
    const Eigen::Vector3d back =
        standing.center - Eigen::Vector3d(0.25, 0.0, 0.0);
    EXPECT_EQ(mgg::orientedBoxPathStatus(map, standing.center, back,
                                         standing, true, &standing),
              VoxelStatus::kFree);
  }
}

}  // namespace
