#pragma once

#include <cstdint>
#include <cmath>

namespace hybrid_astar
{

struct alignas(8) Node3D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};

  double g_cost{0.0};
  double h_cost{0.0};

  int32_t x_idx{0};
  int32_t y_idx{0};
  int32_t theta_idx{0};

  Node3D * parent{nullptr};
  uint8_t primitive_id{0};
  bool is_reverse{false};
  bool is_open{false};
  bool is_closed{false};

  uint32_t run_id{0};

  [[nodiscard]] constexpr double f_cost() const noexcept
  {
    return g_cost + h_cost;
  }

  void reset(uint32_t current_run_id) noexcept
  {
    parent = nullptr;
    primitive_id = 0;
    is_reverse = false;
    is_open = false;
    is_closed = false;
    g_cost = 0.0;
    h_cost = 0.0;
    run_id = current_run_id;
  }
};

} // namespace hybrid_astar
