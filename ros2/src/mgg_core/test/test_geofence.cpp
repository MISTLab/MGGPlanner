// Tests for keep-out zones. The ROS 1 version had none.

#include <vector>

#include <gtest/gtest.h>

#include "mgg_core/geofence_manager.h"

namespace {

using mgg::GeofenceManager;
using mgg::Polygon2d;

/// A 4 m square keep-out centred on (10, 0).
Polygon2d squareAt(double cx, double cy, double half = 2.0) {
  return Polygon2d(std::vector<Eigen::Vector2d>{
      {cx - half, cy - half}, {cx + half, cy - half},
      {cx + half, cy + half}, {cx - half, cy + half}});
}

TEST(Polygon2d, InsideAndOutside) {
  const Polygon2d p = squareAt(10.0, 0.0);
  EXPECT_TRUE(p.isInside(Eigen::Vector2d(10.0, 0.0)));
  EXPECT_FALSE(p.isInside(Eigen::Vector2d(0.0, 0.0)));
  EXPECT_FALSE(p.isInside(Eigen::Vector2d(13.0, 0.0)));
}

TEST(Polygon2d, CenterIsTheCentroid) {
  Polygon2d p = squareAt(10.0, 4.0);
  Eigen::Vector2d c;
  p.getCenter(c);
  EXPECT_NEAR(c.x(), 10.0, 1e-6);
  EXPECT_NEAR(c.y(), 4.0, 1e-6);
}

TEST(Polygon2d, RectangleIntersectionDetectsOverlap) {
  const Polygon2d p = squareAt(10.0, 0.0);
  // A robot footprint straddling the zone boundary.
  EXPECT_TRUE(p.doIntersectWithRectangle(Eigen::Vector2d(8.5, 0.0),
                                         Eigen::Vector2d(1.0, 1.0)));
  // Well clear of it.
  EXPECT_FALSE(p.doIntersectWithRectangle(Eigen::Vector2d(0.0, 0.0),
                                          Eigen::Vector2d(1.0, 1.0)));
}

TEST(GeofenceManager, PathThroughAZoneIsViolated) {
  GeofenceManager gm;
  Polygon2d zone = squareAt(10.0, 0.0);
  gm.addGeofenceArea(zone);

  // Straight through the keep-out.
  EXPECT_EQ(gm.getPathStatus(Eigen::Vector2d(0.0, 0.0),
                             Eigen::Vector2d(20.0, 0.0),
                             Eigen::Vector2d(0.8, 0.8)),
            GeofenceManager::CoordinateStatus::kViolated);
}

TEST(GeofenceManager, PathClearOfEveryZoneIsOk) {
  GeofenceManager gm;
  Polygon2d zone = squareAt(10.0, 0.0);
  gm.addGeofenceArea(zone);

  EXPECT_EQ(gm.getPathStatus(Eigen::Vector2d(0.0, 20.0),
                             Eigen::Vector2d(20.0, 20.0),
                             Eigen::Vector2d(0.8, 0.8)),
            GeofenceManager::CoordinateStatus::kOK);
}

TEST(GeofenceManager, PointInsideAZoneIsViolated) {
  GeofenceManager gm;
  Polygon2d zone = squareAt(10.0, 0.0);
  gm.addGeofenceArea(zone);
  EXPECT_EQ(gm.getCoordinateStatus(Eigen::Vector2d(10.0, 0.0)),
            GeofenceManager::CoordinateStatus::kViolated);
  EXPECT_EQ(gm.getCoordinateStatus(Eigen::Vector2d(0.0, 0.0)),
            GeofenceManager::CoordinateStatus::kOK);
}

TEST(GeofenceManager, NoZonesMeansEverythingIsAllowed) {
  GeofenceManager gm;
  EXPECT_EQ(gm.getPathStatus(Eigen::Vector2d(0.0, 0.0),
                             Eigen::Vector2d(100.0, 100.0),
                             Eigen::Vector2d(0.8, 0.8)),
            GeofenceManager::CoordinateStatus::kOK);
}

}  // namespace
