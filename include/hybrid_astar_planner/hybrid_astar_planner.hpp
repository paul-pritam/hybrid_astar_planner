#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "hybrid_astar_planner/node3d.hpp"
#include "hybrid_astar_planner/node_pool.hpp"
#include "hybrid_astar_planner/visited_grid.hpp"
#include "hybrid_astar_planner/dijkstra_heuristic.hpp"
#include "hybrid_astar_planner/collision_checker.hpp"
#include "hybrid_astar_planner/hybrid_astar_search.hpp"

namespace hybrid_astar
{

class HybridAStarPlanner : public nav2_core::GlobalPlanner {
public:
  HybridAStarPlanner() = default;
  ~HybridAStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("HybridAStarPlanner")};
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_{nullptr};
  std::string name_;

  SearchParams search_params_;
  size_t pool_capacity_{150000};

  std::shared_ptr<Node3DPool> pool_;
  std::shared_ptr<VisitedGrid3D> visited_grid_;
  std::shared_ptr<BackwardDijkstra2D> dijkstra_heuristic_;
  std::shared_ptr<CollisionChecker> collision_checker_;
  std::unique_ptr<HybridAStarSearch> search_engine_;

  uint32_t current_run_id_{0};
};

} // namespace hybrid_astar
