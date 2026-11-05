#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>
#include "hybrid_astar_planner/node3d.hpp"

namespace hybrid_astar
{

class VisitedGrid3D {
public:
  VisitedGrid3D(size_t size_x = 0, size_t size_y = 0, size_t size_theta = 72)
  : size_x_(size_x), size_y_(size_y), size_theta_(size_theta),
    table_(size_x * size_y * size_theta, nullptr) {}

  void update_dimensions(size_t size_x, size_t size_y, size_t size_theta)
  {
    size_x_ = size_x;
    size_y_ = size_y;
    size_theta_ = size_theta;
    table_.assign(size_x_ * size_y_ * size_theta_, nullptr);
  }

  [[nodiscard]] inline size_t compute_index(int32_t x, int32_t y, int32_t theta) const noexcept
  {
    return (static_cast<size_t>(theta) * size_y_ + static_cast<size_t>(y)) * size_x_ +
           static_cast<size_t>(x);
  }

  [[nodiscard]] inline Node3D * get(
    int32_t x, int32_t y, int32_t theta, uint32_t current_run_id) const noexcept
  {

    if (x < 0 || x >= static_cast<int32_t>(size_x_) ||
      y < 0 || y >= static_cast<int32_t>(size_y_) ||
      theta < 0 || theta >= static_cast<int32_t>(size_theta_))
    {
      return nullptr;
    }

    Node3D * node = table_[compute_index(x, y, theta)];

    if (node != nullptr && node->run_id == current_run_id) {
      return node;
    }

    return nullptr;
  }

  inline void set(int32_t x, int32_t y, int32_t theta, Node3D * node) noexcept
  {
    if (x < 0 || x >= static_cast<int32_t>(size_x_) ||
      y < 0 || y >= static_cast<int32_t>(size_y_) ||
      theta < 0 || theta >= static_cast<int32_t>(size_theta_))
    {
      return;
    }
    table_[compute_index(x, y, theta)] = node;
  }

  [[nodiscard]] size_t size_x() const noexcept {return size_x_;}
  [[nodiscard]] size_t size_y() const noexcept {return size_y_;}
  [[nodiscard]] size_t size_theta() const noexcept {return size_theta_;}

private:
  size_t size_x_{0};
  size_t size_y_{0};
  size_t size_theta_{0};
  std::vector<Node3D *> table_;
};

} // namespace hybrid_astar
