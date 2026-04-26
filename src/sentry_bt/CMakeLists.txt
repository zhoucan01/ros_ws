cmake_minimum_required(VERSION 3.8)
project(sentry_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sentry_decision_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(algo_master REQUIRED)

# find_package(algo_master REQUIRED)

add_executable(sentry_bt_node src/sentry_bt_node.cpp
     src/behaviors/topics2blackboard.cpp
     src/behaviors/anti_autoaim.cpp
)

target_include_directories(sentry_bt_node PRIVATE include)

ament_target_dependencies(sentry_bt_node
  rclcpp
  behaviortree_cpp
  sentry_decision_msg
  geometry_msgs
  ament_index_cpp
  tf2
  tf2_ros
  nav2_msgs
  visualization_msgs
  algo_master
  

  # algo_master
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS sentry_bt_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include
)





ament_package()
