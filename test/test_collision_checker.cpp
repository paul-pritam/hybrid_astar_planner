#include <gtest/gtest.h>
#include <vector>
#include <numbers>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "hybrid_astar_planner/collision_checker.hpp"

using namespace hybrid_astar;

TEST(CollisionCheckerTest, DoorwayOrientationValidation) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);

  for (unsigned int y = 0; y < 100; ++y) {
    if (y < 42 || y > 58) {
      costmap.setCost(50, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      costmap.setCost(49, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      costmap.setCost(51, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  CollisionChecker checker(&costmap);

  std::vector<geometry_msgs::msg::Point> footprint;
  geometry_msgs::msg::Point p1, p2, p3, p4;
  p1.x = -0.5; p1.y = -0.3;
  p2.x = 0.5; p2.y = -0.3;
  p3.x = 0.5; p3.y = 0.3;
  p4.x = -0.5; p4.y = 0.3;
  footprint.push_back(p1);
  footprint.push_back(p2);
  footprint.push_back(p3);
  footprint.push_back(p4);

  checker.set_footprint(footprint);

  EXPECT_TRUE(checker.is_pose_valid(1.0, 1.0, 0.0));
  EXPECT_FALSE(checker.is_pose_valid(2.5, 0.5, 0.0));

  EXPECT_TRUE(checker.is_pose_valid(2.5, 2.5, 0.0));
  EXPECT_FALSE(checker.is_pose_valid(2.5, 2.5, 0.5 * std::numbers::pi));

  EXPECT_FALSE(checker.is_pose_valid(-1.0, 2.5, 0.0));
  EXPECT_FALSE(checker.is_pose_valid(6.0, 2.5, 0.0));
}
