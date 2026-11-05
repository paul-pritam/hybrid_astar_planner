#include "hybrid_astar_planner/hybrid_astar_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace hybrid_astar
{

void HybridAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Parent node expired during configuration");
  }

  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".turning_radius", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".motion_primitive_length", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".angular_bins", rclcpp::ParameterValue(72));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".gear_change_penalty", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".analytic_expansion_threshold", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".analytic_expansion_frequency", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(60000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_reverse", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pool_capacity", rclcpp::ParameterValue(150000));

  search_params_.min_turning_radius = node->get_parameter(name_ + ".turning_radius").as_double();
  search_params_.step_length = node->get_parameter(name_ + ".motion_primitive_length").as_double();
  search_params_.angular_bins = node->get_parameter(name_ + ".angular_bins").as_int();
  search_params_.reverse_penalty = node->get_parameter(name_ + ".reverse_penalty").as_double();
  search_params_.gear_change_penalty = node->get_parameter(name_ +
      ".gear_change_penalty").as_double();
  search_params_.analytic_expansion_threshold =
    node->get_parameter(name_ + ".analytic_expansion_threshold").as_double();
  search_params_.analytic_expansion_frequency =
    node->get_parameter(name_ + ".analytic_expansion_frequency").as_int();
  search_params_.max_iterations = node->get_parameter(name_ + ".max_iterations").as_int();
  search_params_.allow_reverse = node->get_parameter(name_ + ".allow_reverse").as_bool();
  pool_capacity_ = static_cast<size_t>(node->get_parameter(name_ + ".pool_capacity").as_int());

  pool_ = std::make_shared<Node3DPool>(pool_capacity_);
  visited_grid_ = std::make_shared<VisitedGrid3D>(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY(),
    search_params_.angular_bins
  );
  dijkstra_heuristic_ = std::make_shared<BackwardDijkstra2D>();
  dijkstra_heuristic_->initialize(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY(),
    static_cast<float>(costmap_->getResolution())
  );
  collision_checker_ = std::make_shared<CollisionChecker>(costmap_);
  collision_checker_->set_footprint(costmap_ros->getRobotFootprint());

  search_engine_ = std::make_unique<HybridAStarSearch>(
    pool_, visited_grid_, collision_checker_, dijkstra_heuristic_);

  RCLCPP_INFO(
    logger_,
    "Configured HybridAStarPlanner [%s]: turning_radius=%.2f, pool_capacity=%zu",
    name_.c_str(), search_params_.min_turning_radius, pool_capacity_);
}

void HybridAStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating HybridAStarPlanner [%s]", name_.c_str());
}

void HybridAStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating HybridAStarPlanner [%s]", name_.c_str());
}

void HybridAStarPlanner::cleanup()
{
  search_engine_.reset();
  collision_checker_.reset();
  dijkstra_heuristic_.reset();
  visited_grid_.reset();
  pool_.reset();
  RCLCPP_INFO(logger_, "Cleaned up HybridAStarPlanner [%s]", name_.c_str());
}

nav_msgs::msg::Path HybridAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.stamp = costmap_ros_ ? costmap_ros_->get_clock()->now() : rclcpp::Clock().now();
  path.header.frame_id = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : start.header.frame_id;

  if (cancel_checker && cancel_checker()) {
    RCLCPP_WARN(logger_, "Planning cancelled by Nav2 before execution");
    return path;
  }

  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);

  Pose2D start_pose{start.pose.position.x, start.pose.position.y, start_yaw};
  Pose2D goal_pose{goal.pose.position.x, goal.pose.position.y, goal_yaw};

  unsigned int goal_mx, goal_my;
  if (!costmap_->worldToMap(goal_pose.x, goal_pose.y, goal_mx, goal_my)) {
    RCLCPP_ERROR(logger_, "Goal pose (%.2f, %.2f) lies outside costmap boundaries", goal_pose.x,
        goal_pose.y);
    return path;
  }

  ++current_run_id_;
  pool_->reset();

  if (visited_grid_->size_x() != costmap_->getSizeInCellsX() ||
    visited_grid_->size_y() != costmap_->getSizeInCellsY())
  {
    visited_grid_->update_dimensions(
      costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), search_params_.angular_bins);
    dijkstra_heuristic_->initialize(
      costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(),
      static_cast<float>(costmap_->getResolution()));
  }

  if (costmap_ros_) {
    collision_checker_->set_footprint(costmap_ros_->getRobotFootprint());
  }

  dijkstra_heuristic_->compute(costmap_, goal_mx, goal_my);

  if (cancel_checker && cancel_checker()) {
    return path;
  }

  const auto poses = search_engine_->plan(
    start_pose, goal_pose, costmap_, search_params_, current_run_id_);

  if (poses.empty()) {
    RCLCPP_WARN(logger_, "Hybrid A* was unable to find a collision-free path to goal");
    return path;
  }

  path.poses.reserve(poses.size());
  for (const auto & p : poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, p.theta);
    ps.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(ps);
  }

  RCLCPP_INFO(logger_, "Hybrid A* generated smooth path with %zu poses", path.poses.size());
  return path;
}

} // namespace hybrid_astar

PLUGINLIB_EXPORT_CLASS(hybrid_astar::HybridAStarPlanner, nav2_core::GlobalPlanner)
