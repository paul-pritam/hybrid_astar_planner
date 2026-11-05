#include <gtest/gtest.h>
#include <cmath>
#include <numbers>
#include "hybrid_astar_planner/reeds_shepp.hpp"

using namespace hybrid_astar;

TEST(ReedsSheppTest, ZeroDistanceToSelf) {
  Pose2D p{1.0, 2.0, 0.5};
  double dist = reeds_shepp_distance(p, p, 1.0);
  EXPECT_NEAR(dist, 0.0, 1e-6);
}

TEST(ReedsSheppTest, StraightForwardDistance) {
  Pose2D start{0.0, 0.0, 0.0};
  Pose2D goal{3.0, 0.0, 0.0};
  double dist = reeds_shepp_distance(start, goal, 1.0);
  EXPECT_NEAR(dist, 3.0, 1e-4);
}

TEST(ReedsSheppTest, EndPointReachability) {
  const double rho = 1.0;
  Pose2D start{0.0, 0.0, 0.0};

  // Test several non-trivial configurations
  std::vector<Pose2D> goals = {
    {2.0, 2.0, 0.5},
    {3.0, -1.0, -0.8},
    {0.5, 0.8, 1.2},
    {-2.0, 1.0, std::numbers::pi / 2.0},
    {0.0, 1.5, 0.0} // Parallel parking lateral shift
  };

  for (const auto & goal : goals) {
    auto path = solve_reeds_shepp(start, goal, rho);
    EXPECT_FALSE(std::isinf(path.total_length));
    EXPECT_GT(path.total_length, 0.0);

    auto samples = sample_reeds_shepp(start, path, rho, 0.02);
    ASSERT_FALSE(samples.empty());

    // First sample must match start
    EXPECT_NEAR(samples.front().x, start.x, 1e-4);
    EXPECT_NEAR(samples.front().y, start.y, 1e-4);
    EXPECT_NEAR(normalize_angle(samples.front().theta - start.theta), 0.0, 1e-4);

    // Final sample must match goal
    EXPECT_NEAR(samples.back().x, goal.x, 1e-3);
    EXPECT_NEAR(samples.back().y, goal.y, 1e-3);
    EXPECT_NEAR(normalize_angle(samples.back().theta - goal.theta), 0.0, 1e-3);
  }
}

TEST(ReedsSheppTest, LateralShiftIsLongerThanEuclidean) {
  // Parallel parking shift: start at (0, 0, 0), goal at (0, 0.5, 0)
  Pose2D start{0.0, 0.0, 0.0};
  Pose2D goal{0.0, 0.5, 0.0};
  double euclidean = 0.5;
  double rs_dist = reeds_shepp_distance(start, goal, 1.0);

  // Because vehicle cannot slide sideways, RS distance must be substantially longer than 0.5m
  EXPECT_GT(rs_dist, euclidean * 2.0);
}
