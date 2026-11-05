#pragma once

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <cstdint>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace hybrid_astar
{

struct GridCell
{
  uint32_t x{0};
  uint32_t y{0};
  float distance_cost{0.0f};

  constexpr bool operator>(const GridCell & other) const noexcept
  {
    return distance_cost > other.distance_cost;
  }
};

class BackwardDijkstra2D {
public:
  BackwardDijkstra2D() = default;

  void initialize(uint32_t size_x, uint32_t size_y, float resolution)
  {
    size_x_ = size_x;
    size_y_ = size_y;
    resolution_ = resolution;
    distance_map_.assign(static_cast<size_t>(size_x_) * size_y_,
        std::numeric_limits<float>::infinity());
  }

  void compute(nav2_costmap_2d::Costmap2D * costmap, uint32_t goal_x, uint32_t goal_y)
  {
    if (!costmap || goal_x >= size_x_ || goal_y >= size_y_) {
      return;
    }

    std::fill(distance_map_.begin(), distance_map_.end(), std::numeric_limits<float>::infinity());

    std::priority_queue<GridCell, std::vector<GridCell>, std::greater<GridCell>> queue;

    const uint32_t goal_idx = goal_y * size_x_ + goal_x;
    distance_map_[goal_idx] = 0.0f;
    queue.push({goal_x, goal_y, 0.0f});

    const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    const float step_distances[8] = {
      1.41421356f * resolution_, 1.0f * resolution_, 1.41421356f * resolution_,
      1.0f * resolution_, 1.0f * resolution_,
      1.41421356f * resolution_, 1.0f * resolution_, 1.41421356f * resolution_
    };

    while (!queue.empty()) {
      const auto current = queue.top();
      queue.pop();

      const uint32_t current_idx = current.y * size_x_ + current.x;

      if (current.distance_cost > distance_map_[current_idx]) {
        continue;
      }

      for (int i = 0; i < 8; ++i) {
        const int nx = static_cast<int>(current.x) + dx[i];
        const int ny = static_cast<int>(current.y) + dy[i];

        if (nx < 0 || nx >= static_cast<int>(size_x_) ||
          ny < 0 || ny >= static_cast<int>(size_y_))
        {
          continue;
        }

        const unsigned char cell_cost = costmap->getCost(nx, ny);

        if (cell_cost >= nav2_costmap_2d::LETHAL_OBSTACLE ||
          cell_cost == nav2_costmap_2d::NO_INFORMATION)
        {
          continue;
        }

        const float wall_penalty = 1.0f + (static_cast<float>(cell_cost) / 255.0f);
        const float new_distance = current.distance_cost + (step_distances[i] * wall_penalty);

        const uint32_t neighbor_idx = static_cast<uint32_t>(ny) * size_x_ +
          static_cast<uint32_t>(nx);

        if (new_distance < distance_map_[neighbor_idx]) {
          distance_map_[neighbor_idx] = new_distance;
          queue.push({static_cast<uint32_t>(nx), static_cast<uint32_t>(ny), new_distance});
        }
      }
    }
  }

  [[nodiscard]] float get_cost(uint32_t cell_x, uint32_t cell_y) const noexcept
  {
    if (cell_x >= size_x_ || cell_y >= size_y_) {
      return std::numeric_limits<float>::infinity();
    }
    return distance_map_[cell_y * size_x_ + cell_x];
  }

  [[nodiscard]] float get_cost_world(
    const nav2_costmap_2d::Costmap2D * costmap, double wx, double wy) const noexcept
  {
    if (!costmap) {
      return std::numeric_limits<float>::infinity();
    }
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(wx, wy, mx, my)) {
      return std::numeric_limits<float>::infinity();
    }
    return get_cost(mx, my);
  }

  [[nodiscard]] uint32_t size_x() const noexcept {return size_x_;}
  [[nodiscard]] uint32_t size_y() const noexcept {return size_y_;}

private:
  uint32_t size_x_{0};
  uint32_t size_y_{0};
  float resolution_{0.05f};
  std::vector<float> distance_map_;
};

} // namespace hybrid_astar
