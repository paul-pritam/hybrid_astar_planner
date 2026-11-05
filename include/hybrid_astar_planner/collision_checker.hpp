#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace hybrid_astar
{

class CollisionChecker {
public:
  explicit CollisionChecker(nav2_costmap_2d::Costmap2D * costmap = nullptr)
  : costmap_(costmap), footprint_checker_(costmap) {}

  void set_costmap(nav2_costmap_2d::Costmap2D * costmap)
  {
    costmap_ = costmap;
    footprint_checker_ =
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>(costmap);
  }

  void set_footprint(const std::vector<geometry_msgs::msg::Point> & footprint)
  {
    footprint_ = footprint;
    inscribed_radius_ = compute_inscribed_radius(footprint);
    circumscribed_radius_ = compute_circumscribed_radius(footprint);
  }

  [[nodiscard]] bool is_pose_valid(double x, double y, double theta) const
  {
    if (!costmap_) {
      return false;
    }

    unsigned int mx = 0;
    unsigned int my = 0;

    if (!costmap_->worldToMap(x, y, mx, my)) {
      return false;
    }

    const unsigned char center_cost = costmap_->getCost(mx, my);

    if (center_cost >= nav2_costmap_2d::LETHAL_OBSTACLE ||
      center_cost == nav2_costmap_2d::NO_INFORMATION)
    {
      return false;
    }

    if (footprint_.empty()) {
      return center_cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }

    const double footprint_cost = footprint_checker_.footprintCostAtPose(x, y, theta, footprint_);

    const bool is_lethal = (footprint_cost >=
      static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
    const bool is_unknown = (footprint_cost ==
      static_cast<double>(nav2_costmap_2d::NO_INFORMATION));

    return !is_lethal && !is_unknown;
  }

  [[nodiscard]] bool is_trajectory_valid(
    const std::vector<std::tuple<double, double, double>> & poses) const
  {
    for (const auto & [x, y, theta] : poses) {
      if (!is_pose_valid(x, y, theta)) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] double inscribed_radius() const noexcept {return inscribed_radius_;}
  [[nodiscard]] double circumscribed_radius() const noexcept {return circumscribed_radius_;}

private:
  static double compute_circumscribed_radius(
    const std::vector<geometry_msgs::msg::Point> & fp)
  {
    double r_sq_max = 0.0;
    for (const auto & pt : fp) {
      double r_sq = pt.x * pt.x + pt.y * pt.y;
      r_sq_max = std::max(r_sq_max, r_sq);
    }
    return std::sqrt(r_sq_max);
  }

  static double compute_inscribed_radius(
    const std::vector<geometry_msgs::msg::Point> & fp)
  {
    if (fp.size() < 3) {return 0.1;}

    double min_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < fp.size(); ++i) {
      size_t next = (i + 1) % fp.size();

      double dx = fp[next].x - fp[i].x;
      double dy = fp[next].y - fp[i].y;
      double segment_len = std::hypot(dx, dy);
      if (segment_len < 1e-6) {continue;}

      double cross = std::abs(fp[i].x * fp[next].y - fp[i].y * fp[next].x);
      double dist = cross / segment_len;
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  mutable nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  footprint_checker_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  double inscribed_radius_{0.1};
  double circumscribed_radius_{0.3};
};

} // namespace hybrid_astar
