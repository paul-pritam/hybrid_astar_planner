# hybrid_astar_planner

C++20 Kinodynamic Hybrid A* global path planner plugin for ROS 2 Nav2 (`nav2_core::GlobalPlanner`). Engineered for non-holonomic mobile robots (such as Ackermann vehicles and constrained differential-drive systems).

## Features

* **C++20 Kinodynamic Search**: Discretizes control space into continuous circular arcs satisfying minimum turning radius constraints.
* **Zero-Allocation Inner Loop**: Uses a pre-allocated arena memory pool (`Node3DPool`) during planning cycles, eliminating runtime heap lock contention.
* **O(1) Generational Lattice**: Visited state tracking uses generational run IDs (`VisitedGrid3D`) to invalidate millions of cells in a single CPU cycle without `memset`.
* **Dual Admissible Heuristics**: Combines 2D backward Dijkstra wavefront flooding (obstacle-aware) and closed-form Reeds-Shepp distance (non-holonomic aware) taking `max(h_dijkstra, h_rs)`.
* **Analytic Expansion Shots**: Employs closed-form Reeds-Shepp trajectory curve sampling near the goal to achieve sub-millimeter docking without lattice discretization artifacts.
* **Two-Tier Collision Checking**: Fast center point check with fallback to oriented polygon footprint rasterization against `nav2_costmap_2d::Costmap2D`.

## Architecture Overview

```text
               Global Goal Request (Nav2)
                           │
                           ▼
          ┌──────────────────────────────────┐
          │   2D Backward Dijkstra Flooding  │
          │ (Calculates true obstacle costs) │
          └────────────────┬─────────────────┘
                           │
                           ▼
          ┌──────────────────────────────────┐
          │     Hybrid A* Kinodynamic Loop   │
          │   - Primitives: Forward/Reverse  │
          │   - 3D Generational Lattice      │
          │   - Higher g-cost tie-breaking   │
          └────────────────┬─────────────────┘
                           │
                Within D_gate of Goal?
              ┌────────────┴────────────┐
             YES                        NO
              │                         │
              ▼                         ▼
   ┌──────────────────────┐   ┌──────────────────────┐
   │ Reeds-Shepp Expansion│   │ Keep Expanding Arcs  │
   │ (Analytic Trick Shot)│   │ (Continue Search)    │
   └──────────┬───────────┘   └──────────┬───────────┘
              │                          │
              └────────────┬─────────────┘
                           ▼
                  Smooth Nav2 Path
```

## Parameters

| Parameter | Type | Default | Description |
|:---|:---:|:---:|:---|
| `turning_radius` | double | 0.5 | Minimum turning radius in meters |
| `motion_primitive_length` | double | 0.3 | Physical arc distance per steering primitive |
| `angular_bins` | int | 72 | Heading discretization bins in SE(2) lattice |
| `reverse_penalty` | double | 2.0 | Multiplier penalty for backing up |
| `gear_change_penalty` | double | 1.5 | Fixed cost penalty when changing drive direction |
| `analytic_expansion_threshold` | double | 3.0 | Distance threshold to attempt Reeds-Shepp trick shots |
| `analytic_expansion_frequency` | int | 5 | Attempt analytic expansion every N iterations |
| `max_iterations` | int | 60000 | Safety cap on node expansions |
| `allow_reverse` | bool | true | Enable reverse motion primitives |
| `pool_capacity` | int | 150000 | Number of pre-allocated `Node3D` slots in arena |

## Nav2 Configuration

Add the planner to your Nav2 planner server YAML configuration:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "hybrid_astar/HybridAStarPlanner"
      turning_radius: 0.5
      motion_primitive_length: 0.3
      angular_bins: 72
      reverse_penalty: 2.0
      gear_change_penalty: 1.5
      analytic_expansion_threshold: 3.0
      analytic_expansion_frequency: 5
      max_iterations: 60000
      allow_reverse: true
```

## Build and Testing

```bash
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build package
cd ~/bcr_ws
colcon build --packages-select hybrid_astar_planner --symlink-install

# Run test suite
colcon test --packages-select hybrid_astar_planner
colcon test-result --verbose
```

## Unit Test Coverage

* `test_kinematics`: Arc integration, angle wrap-around, straight driving, closed-loop return.
* `test_collision_checker`: Footprint orientation sensitivity through narrow doorways.
* `test_dijkstra`: Wavefront obstacle avoidance and Euclidean cost bounds.
* `test_reeds_shepp`: Closed-form curve reachability, endpoint precision, non-holonomic penalty.
* `test_planner`: Open map planning, wall obstacle circumnavigation, lethal goal rejection, dynamic pluginlib loading.

## License

Apache-2.0
