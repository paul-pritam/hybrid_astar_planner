#pragma once

#include <queue>
#include <vector>
#include <cmath>
#include "hybrid_astar_planner/node3d.hpp"

namespace hybrid_astar
{

struct Node3DComparator
{
  constexpr bool operator()(const Node3D * lhs, const Node3D * rhs) const noexcept
  {
    constexpr double EPSILON = 1e-5;
    const double diff = lhs->f_cost() - rhs->f_cost();

    // Tie-breaker: if total f-costs are virtually identical, prioritize larger g-cost
    if (std::abs(diff) < EPSILON) {
      return lhs->g_cost < rhs->g_cost;
    }

    // Min-heap: smaller f-cost gets popped first
    return diff > 0.0;
  }
};

using OpenSet = std::priority_queue<
  Node3D *,
  std::vector<Node3D *>,
  Node3DComparator
>;

} // namespace hybrid_astar
