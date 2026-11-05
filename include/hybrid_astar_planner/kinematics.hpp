#pragma once

#include <cmath>
#include <vector>
#include <numbers>
#include <cstdint>

namespace hybrid_astar
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct MotionPrimitive
{
  uint8_t id{0};
  double curvature{0.0};
  double step_length{0.0};
  bool is_reverse{false};
};

[[nodiscard]] inline double normalize_angle(double angle) noexcept
{
  while (angle >= std::numbers::pi) {angle -= 2.0 * std::numbers::pi;}
  while (angle < -std::numbers::pi) {angle += 2.0 * std::numbers::pi;}
  return angle;
}

[[nodiscard]] Pose2D forward_simulate(
  const Pose2D & start,
  double curvature,
  double step_length,
  bool is_reverse) noexcept;

[[nodiscard]] std::vector<MotionPrimitive> generate_primitive_set(
  double min_turning_radius,
  double step_length,
  bool allow_reverse) noexcept;

} // namespace hybrid_astar
