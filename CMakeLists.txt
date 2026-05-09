cmake_minimum_required(VERSION 3.8)
project(lifecycle_manager_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # メモリ使用量削減のための最適化レベル調整
  add_compile_options(-O1)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Executable
add_executable(local_planning_manager_node
  src/local_planning_manager.cpp
  src/local_planning_manager_node.cpp
  src/local_planning_manager_component.cpp
  src/core/graph.cpp
)

ament_target_dependencies(local_planning_manager_node
  rclcpp
  lifecycle_msgs
  std_msgs
  geometry_msgs
)

# ========================================
# デモ・サンプルのビルド
# ========================================
option(BUILD_EXAMPLES "Build example demonstrations" ON)

if(BUILD_EXAMPLES)
  # ライフサイクルノード1: straight_drive_node
  add_executable(straight_drive_node
    src/straight_drive_node.cpp
  )
  
  ament_target_dependencies(straight_drive_node
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    lifecycle_msgs
  )
  
  # ライフサイクルノード2: slow_drive_node
  add_executable(slow_drive_node
    src/slow_drive_node.cpp
  )
  
  ament_target_dependencies(slow_drive_node
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    lifecycle_msgs
  )
  
  # ライフサイクルノード3: stop_node
  add_executable(stop_node
    src/stop_node.cpp
  )
  
  ament_target_dependencies(stop_node
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    lifecycle_msgs
  )
  
  # ロボットシミュレーター
  add_executable(robot_simulator_node
    src/robot_simulator_node.cpp
  )
  
  ament_target_dependencies(robot_simulator_node
    rclcpp
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )
  
  # デモマネージャーノード (現在は未実装のためコメントアウト)
  # add_executable(demo_manager_node
  #   src/demo_manager_node.cpp
  #   src/demo_manager_component.cpp
  # )
  # 
  # ament_target_dependencies(demo_manager_node
  #   rclcpp
  #   lifecycle_msgs
  #   std_msgs
  #   geometry_msgs
  #   visualization_msgs
  # )
  
  # デモノードのインストール
  install(TARGETS
    straight_drive_node
    slow_drive_node
    stop_node
    robot_simulator_node
    # demo_manager_node  # 未実装のためコメントアウト
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ========================================
# 本体のインストール
# ========================================
install(TARGETS
  local_planning_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
