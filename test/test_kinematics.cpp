#include <gtest/gtest.h>
#include "hybrid_astar_planner/kinematics.hpp"

using namespace hybrid_astar;

TEST(KinematicsTest, NormalizeAngleInBounds) {
  EXPECT_NEAR(normalize_angle(0.0), 0.0, 1e-6);
  EXPECT_NEAR(normalize_angle(std::numbers::pi), -std::numbers::pi, 1e-6);
  EXPECT_NEAR(normalize_angle(3.0 * std::numbers::pi), -std::numbers::pi, 1e-6);
  EXPECT_NEAR(normalize_angle(-3.0 * std::numbers::pi), -std::numbers::pi, 1e-6);
  EXPECT_NEAR(normalize_angle(0.5), 0.5, 1e-6);
  EXPECT_NEAR(normalize_angle(-0.5), -0.5, 1e-6);
}

TEST(KinematicsTest, StraightForwardMotion) {
  Pose2D start{0.0, 0.0, 0.0};
  Pose2D next = forward_simulate(start, 0.0, 1.0, false);
  EXPECT_NEAR(next.x, 1.0, 1e-6);
  EXPECT_NEAR(next.y, 0.0, 1e-6);
  EXPECT_NEAR(next.theta, 0.0, 1e-6);
}

TEST(KinematicsTest, StraightReverseMotion) {
  Pose2D start{0.0, 0.0, 0.0};
  Pose2D next = forward_simulate(start, 0.0, 1.0, true);
  EXPECT_NEAR(next.x, -1.0, 1e-6);
  EXPECT_NEAR(next.y, 0.0, 1e-6);
  EXPECT_NEAR(next.theta, 0.0, 1e-6);
}

TEST(KinematicsTest, QuarterTurnLeft) {
  const double radius = 1.0;
  const double curvature = 1.0 / radius;
  const double quarter_arc = 0.5 * std::numbers::pi * radius;

  Pose2D start{0.0, 0.0, 0.0};
  Pose2D next = forward_simulate(start, curvature, quarter_arc, false);

  EXPECT_NEAR(next.x, 1.0, 1e-5);
  EXPECT_NEAR(next.y, 1.0, 1e-5);
  EXPECT_NEAR(next.theta, 0.5 * std::numbers::pi, 1e-5);
}

TEST(KinematicsTest, FullCircleArcReturnsToStart) {
  const double radius = 1.0;
  const double curvature = 1.0 / radius;
  const double quarter_arc = 0.5 * std::numbers::pi * radius;

  Pose2D pose{0.0, 0.0, 0.0};
  for (int i = 0; i < 4; ++i) {
    pose = forward_simulate(pose, curvature, quarter_arc, false);
  }

  EXPECT_NEAR(pose.x, 0.0, 1e-5);
  EXPECT_NEAR(pose.y, 0.0, 1e-5);
  EXPECT_NEAR(pose.theta, 0.0, 1e-5);
}

TEST(KinematicsTest, GeneratePrimitivesCount) {
  auto fwd_only = generate_primitive_set(0.5, 0.3, false);
  EXPECT_EQ(fwd_only.size(), 3UL);
  EXPECT_FALSE(fwd_only[0].is_reverse);
  EXPECT_FALSE(fwd_only[1].is_reverse);
  EXPECT_FALSE(fwd_only[2].is_reverse);

  auto fwd_and_rev = generate_primitive_set(0.5, 0.3, true);
  EXPECT_EQ(fwd_and_rev.size(), 6UL);
  EXPECT_FALSE(fwd_and_rev[0].is_reverse);
  EXPECT_TRUE(fwd_and_rev[3].is_reverse);
}
