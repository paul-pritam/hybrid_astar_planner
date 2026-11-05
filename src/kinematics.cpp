#include "hybrid_astar_planner/kinematics.hpp"

namespace hybrid_astar
{

Pose2D forward_simulate(
  const Pose2D & start,
  double curvature,
  double step_length,
  bool is_reverse) noexcept
{
  Pose2D result;
  const double direction = is_reverse ? -1.0 : 1.0;
  const double effective_ds = step_length * direction;

  if (std::abs(curvature) < 1e-6) {
    result.x = start.x + effective_ds * std::cos(start.theta);
    result.y = start.y + effective_ds * std::sin(start.theta);
    result.theta = start.theta;
    return result;
  }

  const double radius = 1.0 / curvature;
  const double delta_theta = effective_ds * curvature;

  const double xc = start.x - radius * std::sin(start.theta);
  const double yc = start.y + radius * std::cos(start.theta);

  const double new_theta = start.theta + delta_theta;
  result.x = xc + radius * std::sin(new_theta);
  result.y = yc - radius * std::cos(new_theta);
  result.theta = normalize_angle(new_theta);

  return result;
}

std::vector<MotionPrimitive> generate_primitive_set(
  double min_turning_radius,
  double step_length,
  bool allow_reverse) noexcept
{
  const double max_curvature = 1.0 / min_turning_radius;
  std::vector<MotionPrimitive> primitives;

  primitives.push_back({0, max_curvature, step_length, false});
  primitives.push_back({1, 0.0, step_length, false});
  primitives.push_back({2, -max_curvature, step_length, false});

  if (allow_reverse) {
    primitives.push_back({3, max_curvature, step_length, true});
    primitives.push_back({4, 0.0, step_length, true});
    primitives.push_back({5, -max_curvature, step_length, true});
  }

  return primitives;
}

} // namespace hybrid_astar
