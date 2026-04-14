cmake_minimum_required(VERSION 3.8)
project(armor_plate_data_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(armor_plate_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(data_visualization_node
  src/DataVisualization.cpp
  src/ScatterPlot.cpp
  src/ScatterPlotData.cpp
)

target_include_directories(data_visualization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/armor_plate_data_visualization>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(data_visualization_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  data_visualization_node
  "rclcpp"
  "armor_plate_interfaces"
)

target_link_libraries(data_visualization_node ${OpenCV_LIBS})

install(TARGETS data_visualization_node
  DESTINATION lib/${PROJECT_NAME}
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
