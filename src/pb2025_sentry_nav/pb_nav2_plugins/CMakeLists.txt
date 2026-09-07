cmake_minimum_required(VERSION 3.5)
project(pb_nav2_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(yaml-cpp REQUIRED)

ament_auto_add_library(pb_back_up_frees_space_behavior SHARED
  src/behaviors/back_up_free_space.cpp
)

ament_auto_add_library(pb_nav2_layers SHARED
  src/layers/intensity_voxel_layer.cpp
  src/layers/tunnel_cost_layer.cpp
)

target_link_libraries(pb_nav2_layers yaml-cpp)

ament_auto_add_library(pb_jps_minco_planner SHARED
  src/jps_minco_planner.cpp
)

ament_auto_add_library(pb_topic_bool_condition_bt_node SHARED
  src/bt_nodes/topic_bool_condition.cpp
)

ament_auto_add_library(pb_select_pose_by_topic_bt_node SHARED
  src/bt_nodes/select_pose_by_topic.cpp
)

ament_auto_add_library(pb_wait_for_tunnel_reopen_bt_node SHARED
  src/bt_nodes/wait_for_tunnel_reopen.cpp
)

ament_auto_add_executable(tunnel_region_monitor
  src/tunnel_region_monitor.cpp
)

ament_auto_add_executable(tf_pose_publisher
  src/tf_pose_publisher.cpp
)

if(BUILD_TESTING)
set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_auto_add_executable(tunnel_strategy_node src/tunnel_strategy_node.cpp)
target_link_libraries(tunnel_strategy_node ${dependencies})

ament_auto_package()
