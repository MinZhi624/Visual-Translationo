cmake_minimum_required(VERSION 3.8)
project(armor_plate_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
# 查找依赖包
# ========================================
find_package(ament_cmake REQUIRED)

# 接口生成依赖
find_package(rosidl_default_generators REQUIRED)

# geometry_msgs 是 ArmorPlate.msg 的依赖（包含 Pose 消息类型）
find_package(geometry_msgs REQUIRED)

# std_msgs 是 ArmorPlates.msg 的依赖（包含 Header 消息类型）
find_package(std_msgs REQUIRED)

# ========================================
# 生成 ROS 2 接口
# ========================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmorPlate.msg"
  "msg/ArmorPlates.msg"
  "msg/DebugTarcker.msg"
  DEPENDENCIES geometry_msgs std_msgs  
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
