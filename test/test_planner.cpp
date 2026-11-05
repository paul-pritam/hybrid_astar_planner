#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cmath>
#include <numbers>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "hybrid_astar_planner/kinematics.hpp"
#include "hybrid_astar_planner/node3d.hpp"
#include "hybrid_astar_planner/node_pool.hpp"
#include "hybrid_astar_planner/visited_grid.hpp"
#include "hybrid_astar_planner/collision_checker.hpp"
#include "hybrid_astar_planner/dijkstra_heuristic.hpp"
#include "hybrid_astar_planner/hybrid_astar_search.hpp"
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_loader.hpp"

using namespace hybrid_astar;

class HybridAStarSearchTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    const uint32_t width = 120;
    const uint32_t height = 120;
    const float resolution = 0.05f;

    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
      width, height, resolution, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);

    pool_ = std::make_shared<Node3DPool>(100000);
    visited_grid_ = std::make_shared<VisitedGrid3D>(width, height, 72);
    dijkstra_ = std::make_shared<BackwardDijkstra2D>();
    dijkstra_->initialize(width, height, resolution);
    collision_checker_ = std::make_shared<CollisionChecker>(costmap_.get());

    std::vector<geometry_msgs::msg::Point> fp;
    geometry_msgs::msg::Point p1, p2, p3, p4;
    p1.x = -0.2; p1.y = -0.15;
    p2.x = 0.2; p2.y = -0.15;
    p3.x = 0.2; p3.y = 0.15;
    p4.x = -0.2; p4.y = 0.15;
    fp.push_back(p1);
    fp.push_back(p2);
    fp.push_back(p3);
    fp.push_back(p4);
    collision_checker_->set_footprint(fp);

    search_ = std::make_unique<HybridAStarSearch>(
      pool_, visited_grid_, collision_checker_, dijkstra_);

    params_.min_turning_radius = 0.5;
    params_.step_length = 0.25;
    params_.reverse_penalty = 2.0;
    params_.gear_change_penalty = 1.5;
    params_.analytic_expansion_threshold = 3.0;
    params_.analytic_expansion_frequency = 4;
    params_.max_iterations = 50000;
    params_.angular_bins = 72;
    params_.allow_reverse = true;
  }

  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::shared_ptr<Node3DPool> pool_;
  std::shared_ptr<VisitedGrid3D> visited_grid_;
  std::shared_ptr<BackwardDijkstra2D> dijkstra_;
  std::shared_ptr<CollisionChecker> collision_checker_;
  std::unique_ptr<HybridAStarSearch> search_;
  SearchParams params_;
};

TEST_F(HybridAStarSearchTest, OpenMapPlanning) {
  Pose2D start{1.0, 1.0, 0.0};
  Pose2D goal{4.0, 4.0, 0.5};

  unsigned int gmx, gmy;
  ASSERT_TRUE(costmap_->worldToMap(goal.x, goal.y, gmx, gmy));
  dijkstra_->compute(costmap_.get(), gmx, gmy);

  auto path = search_->plan(start, goal, costmap_.get(), params_, 1);
  ASSERT_FALSE(path.empty());

  EXPECT_NEAR(path.front().x, start.x, 0.1);
  EXPECT_NEAR(path.front().y, start.y, 0.1);

  EXPECT_NEAR(path.back().x, goal.x, 0.15);
  EXPECT_NEAR(path.back().y, goal.y, 0.15);
  EXPECT_NEAR(normalize_angle(path.back().theta - goal.theta), 0.0, 0.15);

  for (const auto & p : path) {
    EXPECT_TRUE(collision_checker_->is_pose_valid(p.x, p.y, p.theta));
  }
}

TEST_F(HybridAStarSearchTest, PlanAroundObstacleWall) {
  for (uint32_t y = 0; y <= 80; ++y) {
    costmap_->setCost(50, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap_->setCost(49, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    costmap_->setCost(51, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  Pose2D start{1.2, 2.0, 0.0};
  Pose2D goal{3.8, 2.0, 0.0};

  unsigned int gmx, gmy;
  ASSERT_TRUE(costmap_->worldToMap(goal.x, goal.y, gmx, gmy));
  dijkstra_->compute(costmap_.get(), gmx, gmy);

  auto path = search_->plan(start, goal, costmap_.get(), params_, 2);
  ASSERT_FALSE(path.empty());

  double max_y = 0.0;
  for (const auto & p : path) {
    EXPECT_TRUE(collision_checker_->is_pose_valid(p.x, p.y, p.theta));
    max_y = std::max(max_y, p.y);
  }
  EXPECT_GT(max_y, 3.8);

  EXPECT_NEAR(path.back().x, goal.x, 0.15);
  EXPECT_NEAR(path.back().y, goal.y, 0.15);
}

TEST_F(HybridAStarSearchTest, RejectLethalGoal) {
  unsigned int mx, my;
  ASSERT_TRUE(costmap_->worldToMap(3.0, 3.0, mx, my));
  costmap_->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  Pose2D start{1.0, 1.0, 0.0};
  Pose2D goal{3.0, 3.0, 0.0};

  dijkstra_->compute(costmap_.get(), mx, my);
  auto path = search_->plan(start, goal, costmap_.get(), params_, 3);
  EXPECT_TRUE(path.empty());
}

TEST(PluginLibTest, LoadPluginViaClassLoader) {
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
  EXPECT_NO_THROW({
    auto planner = loader.createSharedInstance("hybrid_astar/HybridAStarPlanner");
    EXPECT_NE(planner, nullptr);
  });
}
