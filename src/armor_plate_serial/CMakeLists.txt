cmake_minimum_required(VERSION 3.8)
project(armor_plate_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(armor_plate_interfaces REQUIRED)
find_package(serial_driver REQUIRED)

add_executable(serial_node
  src/serial_node.cpp
  src/crc.cpp
)

ament_target_dependencies(serial_node
  rclcpp
  armor_plate_interfaces
  serial_driver
)
# ======================
# include 路径
# ======================
target_include_directories(serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ======================
# 安装
# ======================
install(TARGETS 
  serial_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}
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
