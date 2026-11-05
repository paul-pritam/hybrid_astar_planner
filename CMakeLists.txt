cmake_minimum_required(VERSION 3.8)
project(hybrid_astar_planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  geometry_msgs
  nav_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Hybrid A* Planner Library
add_library(hybrid_astar_planner SHARED
  src/kinematics.cpp
  src/reeds_shepp.cpp
  src/hybrid_astar_planner.cpp
)

target_include_directories(hybrid_astar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hybrid_astar_planner ${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# Install targets and headers
install(TARGETS hybrid_astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Stage 1: Kinematics test
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics hybrid_astar_planner)

  # Stage 2: Collision checker test
  ament_add_gtest(test_collision_checker test/test_collision_checker.cpp)
  target_link_libraries(test_collision_checker hybrid_astar_planner)
  ament_target_dependencies(test_collision_checker nav2_costmap_2d geometry_msgs)

  # Stage 3: Dijkstra heuristic test
  ament_add_gtest(test_dijkstra test/test_dijkstra.cpp)
  target_link_libraries(test_dijkstra hybrid_astar_planner)
  ament_target_dependencies(test_dijkstra nav2_costmap_2d)

  # Stage 4: Reeds-Shepp test
  ament_add_gtest(test_reeds_shepp test/test_reeds_shepp.cpp)
  target_link_libraries(test_reeds_shepp hybrid_astar_planner)

  # Stage 5: End-to-end planner search test
  ament_add_gtest(test_planner test/test_planner.cpp)
  target_link_libraries(test_planner hybrid_astar_planner)
  ament_target_dependencies(test_planner nav2_costmap_2d geometry_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(hybrid_astar_planner)
ament_export_dependencies(${dependencies})

ament_package()
