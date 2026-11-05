#pragma once

#include <vector>
#include <cstddef>
#include "hybrid_astar_planner/node3d.hpp"

namespace hybrid_astar
{

class Node3DPool {
public:
  explicit Node3DPool(size_t capacity = 150000)
  {
    storage_.resize(capacity);
  }

  [[nodiscard]] Node3D * allocate(
    double x, double y, double theta,
    int32_t x_idx, int32_t y_idx, int32_t theta_idx,
    uint32_t current_run_id) noexcept
  {

    if (allocated_count_ >= storage_.size()) {
      return nullptr;
    }

    Node3D * node = &storage_[allocated_count_++];
    node->reset(current_run_id);
    node->x = x;
    node->y = y;
    node->theta = theta;
    node->x_idx = x_idx;
    node->y_idx = y_idx;
    node->theta_idx = theta_idx;
    return node;
  }

  void reset() noexcept
  {
    allocated_count_ = 0;
  }

  [[nodiscard]] size_t size() const noexcept
  {
    return allocated_count_;
  }

  [[nodiscard]] size_t capacity() const noexcept
  {
    return storage_.size();
  }

private:
  std::vector<Node3D> storage_;
  size_t allocated_count_{0};
};

} // namespace hybrid_astar
