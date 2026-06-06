cmake_minimum_required(VERSION 3.8)
project(armor_plate_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
# 查找依赖包
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# ========================================
# 公共 C++ 头文件接口库（ArmorPose 等需要 Eigen3）
# ========================================
add_library(${PROJECT_NAME}_cpp INTERFACE)
target_include_directories(${PROJECT_NAME}_cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_cpp INTERFACE Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME}_cpp
  EXPORT export_${PROJECT_NAME}_cpp
)
ament_export_targets(export_${PROJECT_NAME}_cpp HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

# ========================================
# 生成 ROS 2 接口
# ========================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmorPlate.msg"
  "msg/ArmorPlates.msg"
  "msg/AimCommand.msg"
  "msg/TrackerData.msg"
  "msg/TrackerDebug.msg"
  "msg/GimbalAngle.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# ========================================
# 安装公共 C++ 头文件
# ========================================
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# ========================================
# 测试相关（可选）
# ========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
