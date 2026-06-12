cmake_minimum_required(VERSION 3.8)
project(armor_plate_identification)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================
# 打包
# ======================
set(  ArmorPlateIdentification
  src/ArmorPlateIdentification.cpp
  src/DetectorArmor.cpp
  src/camera/Camera.cpp
  src/camera/GalaxyCamera.cpp
  src/camera/MindVisionCamera.cpp
  src/PoseSolver.cpp
  src/Detector.cpp
  src/debug/DebugIdentification.cpp
  src/debug/DebugTracker.cpp
  src/NumberClassifier.cpp
  src/GuiWorker.cpp
)
set( Test
  src/Test.cpp
  src/DetectorArmor.cpp
  src/PoseSolver.cpp
  src/Detector.cpp
  src/debug/DebugIdentification.cpp
  src/debug/DebugTest.cpp
  src/NumberClassifier.cpp
  src/GuiWorker.cpp
  src/debug/DebugTracker.cpp
  src/debug/YawSearchBenchmark.cpp
)
# ======================
# 依赖
# ======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(armor_plate_interfaces REQUIRED)
find_package(armor_plate_common REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenVINO REQUIRED)
# ======================
# 库文件
# ======================
add_executable(ArmorPlateIdentification ${ArmorPlateIdentification})
add_executable(Test ${Test})

# ======================
# Yaw 搜索库（内部静态库）
# ======================
add_library(yaw_search STATIC
  src/yaw/YawSearch.cpp
)
target_include_directories(yaw_search PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(yaw_search PUBLIC
  armor_plate_common::armor_plate_common
)
target_compile_features(yaw_search PUBLIC cxx_std_17)

# ======================
# 大恒 SDK 路径
# ======================
set(GALAXY_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_parties/GalaxySDK")
# ======================
# 迈德 SDK 路径
# ======================
set(MINDVISION_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_parties/mvsdk")

# ======================
# include 路径
# ======================
target_include_directories(ArmorPlateIdentification PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GALAXY_SDK_ROOT}/include
  ${MINDVISION_SDK_ROOT}/include
)
target_include_directories(Test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ArmorPlateIdentification PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_include_directories(Test PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_compile_features(ArmorPlateIdentification PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_compile_features(Test PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
# ======================
# 链接库
# ======================
target_link_directories(ArmorPlateIdentification PUBLIC
  ${GALAXY_SDK_ROOT}/lib/amd64
  ${MINDVISION_SDK_ROOT}/lib/amd64
)
target_link_libraries(ArmorPlateIdentification
  ${OpenCV_LIBS}
  openvino::runtime
  gxiapi
  MVSDK
  yaw_search
)
target_link_libraries(Test
  ${OpenCV_LIBS}
  openvino::runtime
  yaw_search
)
# ======================
# 依赖关系
# ======================
ament_target_dependencies(
  ArmorPlateIdentification
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "armor_plate_interfaces"
  "armor_plate_common"
  "ament_index_cpp"
)
ament_target_dependencies(
  Test
  "rclcpp"
  "OpenCV"
  "armor_plate_interfaces"
  "armor_plate_common"
  "ament_index_cpp"
)

# ======================
# 安装第三方动态库
# ======================
file(GLOB GALAXY_LIBS "${GALAXY_SDK_ROOT}/lib/amd64/*")
install(FILES ${GALAXY_LIBS}
  DESTINATION lib/${PROJECT_NAME}
)
file(GLOB MINDVISION_LIBS "${MINDVISION_SDK_ROOT}/lib/amd64/*")
install(FILES ${MINDVISION_LIBS}
  DESTINATION lib/${PROJECT_NAME}
)

# ======================
# 环境钩子：自动配置 LD_LIBRARY_PATH / GENICAM_GENTL64_PATH
# ======================
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/third_parties_lib_path.sh.in")

# ======================
# 安装
# ======================
install(TARGETS
  ArmorPlateIdentification Test
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY
  config video launch model
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yaw_search test/test_yaw_search.cpp)
  target_link_libraries(test_yaw_search
    yaw_search
  )
  ament_target_dependencies(test_yaw_search
    "armor_plate_common"
  )
endif()

ament_package()
