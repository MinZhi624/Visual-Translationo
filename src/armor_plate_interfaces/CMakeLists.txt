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
# 测试相关（可选）
# ========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
