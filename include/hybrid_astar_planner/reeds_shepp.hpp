#pragma once

#include <cmath>
#include <vector>
#include <limits>
#include "hybrid_astar_planner/kinematics.hpp"

namespace hybrid_astar
{

enum class ReedsSheppSegmentType : uint8_t
{
  RS_NOP = 0,
  RS_LEFT = 1,
  RS_STRAIGHT = 2,
  RS_RIGHT = 3
};

struct ReedsSheppPath
{
  ReedsSheppSegmentType type[5]{
    ReedsSheppSegmentType::RS_NOP,
    ReedsSheppSegmentType::RS_NOP,
    ReedsSheppSegmentType::RS_NOP,
    ReedsSheppSegmentType::RS_NOP,
    ReedsSheppSegmentType::RS_NOP
  };
  double length[5]{0.0, 0.0, 0.0, 0.0, 0.0};
  double total_length{std::numeric_limits<double>::infinity()};

  ReedsSheppPath() = default;
  ReedsSheppPath(
    const ReedsSheppSegmentType * t,
    double l0 = 0.0, double l1 = 0.0, double l2 = 0.0, double l3 = 0.0, double l4 = 0.0)
  {
    if (t) {
      for (int i = 0; i < 5; ++i) {
        type[i] = t[i];
      }
    }
    length[0] = l0;
    length[1] = l1;
    length[2] = l2;
    length[3] = l3;
    length[4] = l4;
    total_length = std::abs(l0) + std::abs(l1) + std::abs(l2) + std::abs(l3) + std::abs(l4);
  }
};

/// Solve the optimal Reeds-Shepp path between start and goal poses
[[nodiscard]] ReedsSheppPath solve_reeds_shepp(
  const Pose2D & start,
  const Pose2D & goal,
  double turning_radius) noexcept;

/// Calculate the exact Reeds-Shepp driving distance
[[nodiscard]] double reeds_shepp_distance(
  const Pose2D & start,
  const Pose2D & goal,
  double turning_radius) noexcept;

/// Sample the path into discrete continuous waypoints separated by at most step_size
[[nodiscard]] std::vector<Pose2D> sample_reeds_shepp(
  const Pose2D & start,
  const ReedsSheppPath & path,
  double turning_radius,
  double step_size);

} // namespace hybrid_astar
