cmake_minimum_required(VERSION 3.8)
project(armor_plate_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(armor_plate_interfaces REQUIRED)

add_executable(armor_plate_planner_node
  src/TargetSelector.cpp
  src/TargetPredictor.cpp
  src/ArmorGenerator.cpp
  src/ArmorSelector.cpp
  src/BallisticSolver.cpp
  src/CommandGenerator.cpp
  src/PlannerNode.cpp
  src/planner_node_main.cpp
)

target_include_directories(armor_plate_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(armor_plate_planner_node PUBLIC cxx_std_17)

ament_target_dependencies(
  armor_plate_planner_node
  "rclcpp"
  "geometry_msgs"
  "armor_plate_interfaces"
)

install(TARGETS armor_plate_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
