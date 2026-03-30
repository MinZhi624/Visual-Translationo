cmake_minimum_required(VERSION 3.8)
project(armor_plate_identification)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================
# 打包
# ======================
set(  ArmorPlateIdentifcation
  src/ArmorPlateIdentifcation.cpp
  src/CameraDriver.cpp
  src/SolvePnP.cpp
  src/Recognize.cpp
  src/MyKalmanFilter.cpp
  src/DrawTarget.cpp
  src/PairedLights.cpp
  src/TestFunc.cpp
)
set( Test
  src/Test.cpp
  src/TestFunc.cpp
  src/CameraDriver.cpp
  src/SolvePnP.cpp
  src/Recognize.cpp
  src/MyKalmanFilter.cpp
  src/DrawTarget.cpp
  src/PairedLights.cpp
)

# ======================
# 依赖
# ======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

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

add_executable(ArmorPlateIdentifcation ${ArmorPlateIdentifcation})
add_executable(Test ${Test})

# ======================
# include 路径
# ======================
target_include_directories(ArmorPlateIdentifcation PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
target_include_directories(Test PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

target_include_directories(ArmorPlateIdentifcation PUBLIC
${OpenCV_INCLUDE_DIRS}
${CAMERA_SDK_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
)
target_include_directories(Test PUBLIC
${OpenCV_INCLUDE_DIRS}
${CAMERA_SDK_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
)
target_compile_features(ArmorPlateIdentifcation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ======================
# 链接库
# ======================
target_link_libraries(ArmorPlateIdentifcation
${OpenCV_LIBS}
${CAMERA_SDK_LIBRARY}
)
target_link_libraries(Test
${OpenCV_LIBS}
${CAMERA_SDK_LIBRARY}
)
# ======================
# 依赖关系
# ======================
ament_target_dependencies(
  ArmorPlateIdentifcation
  "rclcpp"
  "OpenCV"
)
ament_target_dependencies(
  Test
  "rclcpp"
  "OpenCV"
)
  
# ======================
# 安装
# ======================
install(TARGETS 
  ArmorPlateIdentifcation Test
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY
  video launch
  DESTINATION share/${PROJECT_NAME}
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
