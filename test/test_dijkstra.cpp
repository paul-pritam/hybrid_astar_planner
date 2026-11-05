#include <gtest/gtest.h>
#include <cmath>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "hybrid_astar_planner/dijkstra_heuristic.hpp"

using namespace hybrid_astar;

TEST(DijkstraHeuristicTest, EmptyMapStraightDistance) {
  const uint32_t width = 100;
  const uint32_t height = 100;
  const float resolution = 0.05f;

  nav2_costmap_2d::Costmap2D costmap(width, height, resolution, 0.0, 0.0,
    nav2_costmap_2d::FREE_SPACE);
  BackwardDijkstra2D dijkstra;
  dijkstra.initialize(width, height, resolution);

  // Goal at (50, 50)
  dijkstra.compute(&costmap, 50, 50);

  // Goal cell cost must be 0
  EXPECT_NEAR(dijkstra.get_cost(50, 50), 0.0f, 1e-4);

  // Cell (50, 60) is 10 cells away straight north: 10 * 0.05 = 0.50m
  EXPECT_NEAR(dijkstra.get_cost(50, 60), 0.50f, 1e-3);

  // Cell (60, 60) is 10 diagonal steps away: 10 * 1.41421 * 0.05 ~= 0.7071m
  EXPECT_NEAR(dijkstra.get_cost(60, 60), 0.7071f, 1e-3);
}

TEST(DijkstraHeuristicTest, ObstacleDetourCost) {
  const uint32_t width = 100;
  const uint32_t height = 100;
  const float resolution = 0.05f;

  nav2_costmap_2d::Costmap2D costmap(width, height, resolution, 0.0, 0.0,
    nav2_costmap_2d::FREE_SPACE);

  // Goal at (30, 50), Start at (70, 50)
  // Wall placed at x = 50 from y = 20 to y = 80
  for (uint32_t y = 20; y <= 80; ++y) {
    costmap.setCost(50, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  BackwardDijkstra2D dijkstra;
  dijkstra.initialize(width, height, resolution);
  dijkstra.compute(&costmap, 30, 50);

  // Straight line distance through the wall would be 40 cells * 0.05 = 2.0m
  const float straight_line = 40.0f * resolution;

  // The cell inside the wall must have infinite cost
  EXPECT_TRUE(std::isinf(dijkstra.get_cost(50, 50)));

  // The actual path around the wall must be strictly longer than straight-line distance
  const float detour_cost = dijkstra.get_cost(70, 50);
  EXPECT_FALSE(std::isinf(detour_cost));
  EXPECT_GT(detour_cost, straight_line * 1.2f);
}
