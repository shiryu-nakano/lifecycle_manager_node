cmake_minimum_required(VERSION 3.8)
project(transition_judgement_node)

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
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)

# ========================================
# 共有ライブラリ（ROS非依存）
#   graph_lib  : Graph + YAML 状態グラフ生成
#   recipe_lib : SwitchingStrategy + recipe_generator (旧 transition_recipe_test 由来)
# ========================================
add_library(graph_lib
  src/transition_recipe_test/graph.cpp
  src/transition_recipe_test/graph_generator.cpp
)
target_include_directories(graph_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(graph_lib yaml-cpp)

add_library(recipe_lib
  src/transition_recipe_test/switching_strategy.cpp
  src/transition_recipe_test/recipe_generator.cpp
)
target_include_directories(recipe_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Executable
add_executable(local_planning_manager_node
  src/local_planning_manager.cpp
  src/local_planning_manager_node.cpp
  src/local_planning_manager_component.cpp
)

ament_target_dependencies(local_planning_manager_node
  rclcpp
  lifecycle_msgs
  std_msgs
  geometry_msgs
)
target_link_libraries(local_planning_manager_node graph_lib)

# ========================================
# デモ・サンプルのビルド
# ========================================
option(BUILD_EXAMPLES "Build example demonstrations" ON)

if(BUILD_EXAMPLES)
  # ライフサイクルノード1: straight_drive_node
  add_executable(straight_drive_node
    src/sample/straight_drive_node.cpp
  )

  ament_target_dependencies(straight_drive_node
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    lifecycle_msgs
  )

  # ライフサイクルノード2: slow_drive_node
  add_executable(slow_drive_node
    src/sample/slow_drive_node.cpp
  )

  ament_target_dependencies(slow_drive_node
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    lifecycle_msgs
  )

  # ライフサイクルノード3: stop_node
  add_executable(stop_node
    src/sample/stop_node.cpp
  )

  ament_target_dependencies(stop_node
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    lifecycle_msgs
  )

  # ロボットシミュレーター
  add_executable(robot_simulator_node
    src/sample/robot_simulator_node.cpp
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

  # デモマネージャーノード（LocalPlanningManagerComponent を使うリファレンス実装）
  add_executable(demo_manager_node
    src/sample/demo_manager_node.cpp
    src/local_planning_manager_component.cpp
  )

  ament_target_dependencies(demo_manager_node
    rclcpp
    lifecycle_msgs
    std_msgs
    geometry_msgs
  )
  target_link_libraries(demo_manager_node graph_lib)

  # デモノードのインストール
  install(TARGETS
    straight_drive_node
    slow_drive_node
    stop_node
    robot_simulator_node
    demo_manager_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ========================================
# transition_recipe_test からコピーしたサンプル
#   - multiple_node_manager（メインのオーケストレータ）
#   - a_node / b_node / c_node（LifecycleNode サンプル）
#   - simulator_node（cmd_vel 積分シミュレータ）
#   ※ graph_lib / recipe_lib は上で定義済み
# ========================================
add_executable(multiple_node_manager
  src/transition_recipe_test/multiple_node_manager.cpp
)
ament_target_dependencies(multiple_node_manager
  rclcpp
  lifecycle_msgs
  geometry_msgs
  nav_msgs
)
target_link_libraries(multiple_node_manager graph_lib recipe_lib)

add_executable(simulator_node
  src/transition_recipe_test/sample/simulator_node.cpp
)
ament_target_dependencies(simulator_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(a_node src/transition_recipe_test/sample/a_node.cpp)
ament_target_dependencies(a_node rclcpp rclcpp_lifecycle lifecycle_msgs geometry_msgs)

add_executable(b_node src/transition_recipe_test/sample/b_node.cpp)
ament_target_dependencies(b_node rclcpp rclcpp_lifecycle lifecycle_msgs geometry_msgs)

add_executable(c_node src/transition_recipe_test/sample/c_node.cpp)
ament_target_dependencies(c_node rclcpp rclcpp_lifecycle lifecycle_msgs geometry_msgs)

install(TARGETS
  multiple_node_manager
  simulator_node
  a_node
  b_node
  c_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS graph_lib recipe_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

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
