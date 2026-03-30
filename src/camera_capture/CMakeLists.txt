cmake_minimum_required(VERSION 3.8)
project(camera_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set ( main
  src/main.cpp
  src/CameraDriver.cpp
)

# ======================
# 依赖
# ======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

# ======================
# 相机 SDK 路径
# ======================
set(CAMERA_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/camera_sdk")
set(CAMERA_SDK_INCLUDE_DIR "${CAMERA_SDK_ROOT}/include")

# ======================
# 库文件
# ======================
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
set(CAMERA_SDK_LIBRARY "${CAMERA_SDK_ROOT}/lib/x64/libMVSDK.so")
else()
  set(CAMERA_SDK_LIBRARY "${CAMERA_SDK_ROOT}/lib/x86/libMVSDK.so")
endif()

add_executable(main ${main})

# ======================
# include 路径
# ======================
target_include_directories(main PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

target_include_directories(main PUBLIC
${OpenCV_INCLUDE_DIRS}
${CAMERA_SDK_INCLUDE_DIR}
)
# ======================
# 链接库
# ======================
target_link_libraries(main
${OpenCV_LIBS}
${CAMERA_SDK_LIBRARY}
)

target_compile_features(main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  main
  "rclcpp"
  "OpenCV"
)
# ======================
# 安装
# ======================
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}
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
