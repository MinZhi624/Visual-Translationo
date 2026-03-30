cmake_minimum_required(VERSION 3.10)

project(camera_demo)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

set(SDK_ROOT "${CMAKE_SOURCE_DIR}/../..")
set(CAMERA_SDK_INCLUDE_DIR "${SDK_ROOT}/include")

# 把这里改成你真实找到的 so 文件名
set(CAMERA_SDK_LIBRARY "${SDK_ROOT}/lib/x64/libMVSDK.so")

add_executable(camera_demo main.cpp)

target_include_directories(camera_demo PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CAMERA_SDK_INCLUDE_DIR}
)

target_link_libraries(camera_demo PRIVATE
    ${OpenCV_LIBS}
    ${CAMERA_SDK_LIBRARY}
    MVSDK
)

get_filename_component(CAMERA_SDK_LIB_DIR "${CAMERA_SDK_LIBRARY}" DIRECTORY)

set_target_properties(camera_demo PROPERTIES
    BUILD_RPATH "${CAMERA_SDK_LIB_DIR}"
)