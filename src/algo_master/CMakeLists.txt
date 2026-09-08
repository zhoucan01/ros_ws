cmake_minimum_required(VERSION 3.8)
project(algo_master)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 首先查找生成消息所需的包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成消息接口（必须在任何使用这些消息的目标之前）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PLC2Target.msg"
  "msg/PLC2Imu.msg"
  "msg/WheelRaw.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 然后查找其他需要的包
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(fmt REQUIRED)
find_package(sentry_decision_msg REQUIRED)

# 可选的其他包
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 包含目录
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# 添加可执行文件
add_executable(algo_master_node src/algo_master_node.cpp)

# 关键步骤1：添加对生成消息库的显式依赖
add_dependencies(algo_master_node ${PROJECT_NAME})

# 关键步骤2：显式链接消息库
# ROS 2 会为每个消息生成特定的库
target_link_libraries(algo_master_node
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp  # 如果需要fastrtps
  ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp  # 如果需要introspection
)

# 关键步骤3：添加ament依赖（包含rosidl_default_runtime）
ament_target_dependencies(algo_master_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  nav2_msgs
  rclcpp_action
  rosidl_default_runtime  # 这个非常重要！
  rosidl_typesupport_cpp  # 显式添加类型支持
  sentry_decision_msg
  # tf2_geometry_msgs
  tf2_geometry_msgs   
)

# 链接其他库
target_link_libraries(algo_master_node
  fmt::fmt
)

# 安装
install(TARGETS algo_master_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# 安装 launch 和 config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 安装消息文件
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# 导出依赖
ament_export_dependencies(
  rosidl_default_runtime
  rosidl_typesupport_cpp
)
ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

ament_package()
