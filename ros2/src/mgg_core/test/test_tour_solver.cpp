// Tests for the open tour over frontier clusters (tour-exploration design
// §2.3): optimal on small scenes, within 5 % of the best known on 50
// clusters, and fast.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "mgg_core/tour_solver.h"

namespace {

using mgg::OpenTour;

struct Scene {
  std::vector<double> from_start;
  std::vector<std::vector<double>> between;
};

/// Clusters at `points`, the robot at the origin, straight-line costs.
Scene euclidean(const std::vector<Eigen::Vector2d>& points) {
  Scene scene;
  for (const Eigen::Vector2d& p : points) {
    scene.from_start.push_back(p.norm());
    scene.between.emplace_back();
    for (const Eigen::Vector2d& q : points) {
      scene.between.back().push_back((p - q).norm());
    }
  }
  return scene;
}

Scene randomScene(std::size_t n, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> coordinate(-40.0, 40.0);
  std::vector<Eigen::Vector2d> points;
  for (std::size_t i = 0; i < n; ++i) {
    points.emplace_back(coordinate(rng), coordinate(rng));
  }
  return euclidean(points);
}

double bruteForce(const Scene& scene) {
  std::vector<int> order(scene.from_start.size());
  std::iota(order.begin(), order.end(), 0);
  double best = mgg::kUnreachableCost;
  do {
    best = std::min(best,
                    mgg::openTourCost(order, scene.from_start, scene.between));
  } while (std::next_permutation(order.begin(), order.end()));
  return best;
}

TEST(OpenTour, EmptyAndSingleClusterTours) {
  const OpenTour none = mgg::solveOpenTour({}, {});
  EXPECT_TRUE(none.order.empty());
  EXPECT_DOUBLE_EQ(none.cost, 0.0);
  const Scene one = euclidean({{3.0, 4.0}});
  const OpenTour single = mgg::solveOpenTour(one.from_start, one.between);
  EXPECT_EQ(single.order, std::vector<int>{0});
  EXPECT_DOUBLE_EQ(single.cost, 5.0);
}

TEST(OpenTour, SmallToursAreOptimal) {
  for (std::size_t n = 1; n <= 7; ++n) {
    for (unsigned seed = 1; seed <= 20; ++seed) {
      const Scene scene = randomScene(n, seed);
      const OpenTour tour = mgg::solveOpenTour(scene.from_start, scene.between);
      ASSERT_EQ(tour.order.size(), n);
      EXPECT_NEAR(tour.cost, bruteForce(scene), 1e-9)
          << n << " clusters, seed " << seed;
    }
  }
}

TEST(OpenTour, LocalImprovementUntanglesACrossedTour) {
  // Four clusters on a line ahead of the robot, visited out of order.
  const Scene line = euclidean({{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}});
  const OpenTour free = mgg::improveOpenTour({2, 0, 3, 1}, line.from_start,
                                             line.between);
  EXPECT_EQ(free.order, (std::vector<int>{0, 1, 2, 3}));
  EXPECT_DOUBLE_EQ(free.cost, 4.0);
  // With the first cluster fixed, the rest are ordered back from it.
  const OpenTour fixed = mgg::improveOpenTour({3, 0, 1, 2}, line.from_start,
                                              line.between, true);
  EXPECT_EQ(fixed.order, (std::vector<int>{3, 2, 1, 0}));
  EXPECT_DOUBLE_EQ(fixed.cost, 7.0);
}

TEST(OpenTour, FiftyClustersWithinFivePercentOfTheBestKnown) {
  for (unsigned seed = 1; seed <= 5; ++seed) {
    const Scene scene = randomScene(50, seed);
    const OpenTour tour = mgg::solveOpenTour(scene.from_start, scene.between);
    ASSERT_EQ(tour.order.size(), 50u);
    // Best known: the solver's own tour and local improvement from thirty
    // random orders.
    double best = tour.cost;
    std::mt19937 rng(seed + 100);
    std::vector<int> order(50);
    std::iota(order.begin(), order.end(), 0);
    for (int restart = 0; restart < 30; ++restart) {
      std::shuffle(order.begin(), order.end(), rng);
      best = std::min(best, mgg::improveOpenTour(order, scene.from_start,
                                                 scene.between)
                                .cost);
    }
    EXPECT_LE(tour.cost, 1.05 * best) << "seed " << seed;
  }
}

TEST(OpenTour, FiftyClustersSolveWithinTenMilliseconds) {
  const Scene scene = randomScene(50, 7);
  constexpr int kRuns = 20;
  const auto start = std::chrono::steady_clock::now();
  for (int run = 0; run < kRuns; ++run) {
    const OpenTour tour = mgg::solveOpenTour(scene.from_start, scene.between);
    ASSERT_EQ(tour.order.size(), 50u);
  }
  const double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - start)
                        .count() /
                    kRuns;
  EXPECT_LT(ms, 10.0);
}

TEST(OpenTour, AForcedFirstClusterStartsTheTourOrRefusesIt) {
  Scene scene = euclidean({{1.0, 0.0}, {2.0, 0.0}, {-5.0, 0.0}});
  const OpenTour forced =
      mgg::solveOpenTour(scene.from_start, scene.between, 2);
  EXPECT_EQ(forced.order, (std::vector<int>{2, 0, 1}));
  EXPECT_DOUBLE_EQ(forced.cost, 5.0 + 6.0 + 1.0);
  scene.from_start[2] = mgg::kUnreachableCost;
  const OpenTour refused =
      mgg::solveOpenTour(scene.from_start, scene.between, 2);
  EXPECT_TRUE(refused.order.empty());
  EXPECT_EQ(refused.cost, mgg::kUnreachableCost);
}

TEST(OpenTour, UnreachableClustersAreLeftOut) {
  Scene scene = euclidean({{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});
  scene.from_start[1] = mgg::kUnreachableCost;
  const OpenTour tour = mgg::solveOpenTour(scene.from_start, scene.between);
  EXPECT_EQ(tour.order, (std::vector<int>{0, 2}));
}

TEST(OpenTour, DisconnectedLegsStillGiveAWholeTour) {
  // Ten clusters in two groups, each reachable from the robot but not from
  // the other group: the heuristic still orders all of them.
  Scene scene = randomScene(10, 3);
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 10; ++j) {
      if ((i < 5) != (j < 5)) scene.between[i][j] = mgg::kUnreachableCost;
    }
  }
  const OpenTour tour = mgg::solveOpenTour(scene.from_start, scene.between);
  EXPECT_EQ(tour.order.size(), 10u);
  EXPECT_FALSE(std::isfinite(tour.cost));
}

TEST(OpenTour, CheapestInsertionFindsTheGapAndKeepsAKeptFirst) {
  const Scene line = euclidean({{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});
  const mgg::Insertion gap =
      mgg::cheapestInsertion({0, 2}, 1, line.from_start, line.between, false);
  EXPECT_EQ(gap.position, 1u);
  EXPECT_NEAR(gap.added_cost, 0.0, 1e-12);
  const mgg::Insertion front =
      mgg::cheapestInsertion({1, 2}, 0, line.from_start, line.between, false);
  EXPECT_EQ(front.position, 0u);
  EXPECT_NEAR(front.added_cost, 0.0, 1e-12);
  const mgg::Insertion kept =
      mgg::cheapestInsertion({1, 2}, 0, line.from_start, line.between, true);
  EXPECT_EQ(kept.position, 1u);
  EXPECT_NEAR(kept.added_cost, 2.0, 1e-12);
  // Nothing to insert into: the start leg alone.
  const mgg::Insertion empty =
      mgg::cheapestInsertion({}, 2, line.from_start, line.between, true);
  EXPECT_EQ(empty.position, 0u);
  EXPECT_DOUBLE_EQ(empty.added_cost, 3.0);
}

}  // namespace
