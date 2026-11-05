#pragma once

#include <vector>
#include <queue>
#include <memory>
#include <algorithm>
#include <cmath>
#include <numbers>

#include "hybrid_astar_planner/kinematics.hpp"
#include "hybrid_astar_planner/node3d.hpp"
#include "hybrid_astar_planner/node_pool.hpp"
#include "hybrid_astar_planner/visited_grid.hpp"
#include "hybrid_astar_planner/open_set.hpp"
#include "hybrid_astar_planner/collision_checker.hpp"
#include "hybrid_astar_planner/dijkstra_heuristic.hpp"
#include "hybrid_astar_planner/reeds_shepp.hpp"

namespace hybrid_astar
{

struct SearchParams
{
  double min_turning_radius{0.5};
  double step_length{0.3};
  double reverse_penalty{2.0};
  double gear_change_penalty{1.5};
  double analytic_expansion_threshold{3.0};
  int32_t analytic_expansion_frequency{5};
  int32_t max_iterations{60000};
  int32_t angular_bins{72};
  bool allow_reverse{true};
};

class HybridAStarSearch {
public:
  HybridAStarSearch(
    std::shared_ptr<Node3DPool> pool,
    std::shared_ptr<VisitedGrid3D> visited_grid,
    std::shared_ptr<CollisionChecker> collision_checker,
    std::shared_ptr<BackwardDijkstra2D> dijkstra_heuristic)
  : pool_(pool), visited_grid_(visited_grid),
    collision_checker_(collision_checker), dijkstra_heuristic_(dijkstra_heuristic) {}

  std::vector<Pose2D> plan(
    const Pose2D & start_pose,
    const Pose2D & goal_pose,
    const nav2_costmap_2d::Costmap2D * costmap,
    const SearchParams & params,
    uint32_t current_run_id)
  {
    std::vector<Pose2D> path;
    if (!costmap || !pool_ || !visited_grid_ || !collision_checker_ || !dijkstra_heuristic_) {
      return path;
    }

    if (!collision_checker_->is_pose_valid(start_pose.x, start_pose.y, start_pose.theta)) {
      return path;
    }
    if (!collision_checker_->is_pose_valid(goal_pose.x, goal_pose.y, goal_pose.theta)) {
      return path;
    }

    const double resolution = costmap->getResolution();
    const double origin_x = costmap->getOriginX();
    const double origin_y = costmap->getOriginY();
    const double angle_bin_size = (2.0 * std::numbers::pi) /
      static_cast<double>(params.angular_bins);

    auto world_to_grid = [&](double wx, double wy, double wtheta, int32_t & gx, int32_t & gy,
      int32_t & gtheta) {
        gx = static_cast<int32_t>((wx - origin_x) / resolution);
        gy = static_cast<int32_t>((wy - origin_y) / resolution);
        const double norm_th = normalize_angle(wtheta);
        gtheta = static_cast<int32_t>((norm_th + std::numbers::pi) / angle_bin_size) %
          params.angular_bins;
        if (gtheta < 0) {gtheta += params.angular_bins;}
      };

    auto compute_heuristic = [&](double x, double y, double theta, int32_t gx,
      int32_t gy) -> double {
        const double h_rs = reeds_shepp_distance({x, y, theta}, goal_pose,
          params.min_turning_radius);
        const float h_dijk = dijkstra_heuristic_->get_cost(gx, gy);
        const double h_dijk_d = std::isinf(h_dijk) ? 0.0 : static_cast<double>(h_dijk);
        return std::max(h_rs, h_dijk_d);
      };

    OpenSet open_set;
    const auto primitives = generate_primitive_set(
      params.min_turning_radius, params.step_length, params.allow_reverse);

    int32_t start_gx, start_gy, start_gtheta;
    world_to_grid(start_pose.x, start_pose.y, start_pose.theta, start_gx, start_gy, start_gtheta);

    Node3D * start_node = pool_->allocate(
      start_pose.x, start_pose.y, start_pose.theta,
      start_gx, start_gy, start_gtheta, current_run_id);
    if (!start_node) {
      return path;
    }

    start_node->g_cost = 0.0;
    start_node->h_cost = compute_heuristic(start_pose.x, start_pose.y, start_pose.theta, start_gx,
        start_gy);
    start_node->is_open = true;

    open_set.push(start_node);
    visited_grid_->set(start_gx, start_gy, start_gtheta, start_node);

    Node3D * best_node = nullptr;
    std::vector<Pose2D> analytic_tail;
    int32_t iterations = 0;

    while (!open_set.empty() && iterations++ < params.max_iterations) {
      Node3D * current = open_set.top();
      open_set.pop();

      if (current->is_closed) {continue;}
      current->is_closed = true;

      const double dist_to_goal = std::hypot(current->x - goal_pose.x, current->y - goal_pose.y);
      const double angle_diff = std::abs(normalize_angle(current->theta - goal_pose.theta));

      if (dist_to_goal <= params.analytic_expansion_threshold &&
        (iterations % params.analytic_expansion_frequency == 0))
      {
        const float dijk_cost = dijkstra_heuristic_->get_cost(current->x_idx, current->y_idx);
        if (!std::isinf(dijk_cost) && dijk_cost <= params.analytic_expansion_threshold * 2.0) {
          const auto rs_path = solve_reeds_shepp(
            {current->x, current->y, current->theta}, goal_pose, params.min_turning_radius);

          if (!std::isinf(rs_path.total_length)) {
            const double check_step = resolution * 0.5;
            auto samples = sample_reeds_shepp(
              {current->x, current->y, current->theta}, rs_path, params.min_turning_radius,
                check_step);

            bool curve_clear = true;
            for (const auto & sample : samples) {
              if (!collision_checker_->is_pose_valid(sample.x, sample.y, sample.theta)) {
                curve_clear = false;
                break;
              }
            }

            if (curve_clear && !samples.empty()) {
              best_node = current;
              analytic_tail = std::move(samples);
              break;
            }
          }
        }
      }

      if (dist_to_goal < 0.20 && angle_diff < 0.15) {
        best_node = current;
        break;
      }

      for (const auto & prim : primitives) {
        const Pose2D child_pose = forward_simulate(
          {current->x, current->y, current->theta},
          prim.curvature, prim.step_length, prim.is_reverse);

        if (!collision_checker_->is_pose_valid(child_pose.x, child_pose.y, child_pose.theta)) {
          continue;
        }

        int32_t child_gx, child_gy, child_gtheta;
        world_to_grid(child_pose.x, child_pose.y, child_pose.theta, child_gx, child_gy,
            child_gtheta);

        Node3D * existing = visited_grid_->get(child_gx, child_gy, child_gtheta, current_run_id);
        if (existing != nullptr && existing->is_closed) {
          continue;
        }

        double edge_cost = prim.step_length;
        if (prim.is_reverse) {edge_cost *= params.reverse_penalty;}
        if (current->parent != nullptr && current->is_reverse != prim.is_reverse) {
          edge_cost += params.gear_change_penalty;
        }
        const double tentative_g = current->g_cost + edge_cost;

        if (existing != nullptr && tentative_g >= existing->g_cost) {
          continue;
        }

        Node3D * child_node = existing;
        if (child_node == nullptr) {
          child_node = pool_->allocate(
            child_pose.x, child_pose.y, child_pose.theta,
            child_gx, child_gy, child_gtheta, current_run_id);
          if (child_node == nullptr) {continue;}
          visited_grid_->set(child_gx, child_gy, child_gtheta, child_node);
        }

        child_node->parent = current;
        child_node->primitive_id = prim.id;
        child_node->is_reverse = prim.is_reverse;
        child_node->g_cost = tentative_g;
        child_node->h_cost = compute_heuristic(child_pose.x, child_pose.y, child_pose.theta,
            child_gx, child_gy);
        child_node->is_open = true;

        open_set.push(child_node);
      }
    }

    if (!best_node) {
      return path;
    }

    Node3D * curr = best_node;
    while (curr != nullptr) {
      path.push_back({curr->x, curr->y, curr->theta});
      curr = curr->parent;
    }
    std::reverse(path.begin(), path.end());

    if (!analytic_tail.empty()) {
      for (size_t i = 1; i < analytic_tail.size(); ++i) {
        path.push_back(analytic_tail[i]);
      }
    }

    return path;
  }

private:
  std::shared_ptr<Node3DPool> pool_;
  std::shared_ptr<VisitedGrid3D> visited_grid_;
  std::shared_ptr<CollisionChecker> collision_checker_;
  std::shared_ptr<BackwardDijkstra2D> dijkstra_heuristic_;
};

} // namespace hybrid_astar
