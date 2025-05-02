cmake_minimum_required(VERSION 3.10)
project(CALIB_CAM)

include(FetchContent)
FetchContent_Declare(
  cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts.git
  GIT_TAG        v3.0.0
)
FetchContent_MakeAvailable(cxxopts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  $(OpenCV_INCLUDE_DIRS)
)

set(SOURCES
  ${CMAKE_SOURCE_DIR}/src/double_buffered_camera.cpp
)

add_executable(capture_mono src/capture_mono.cpp)
target_link_libraries(capture_mono ${OpenCV_LIBS} cxxopts::cxxopts)

add_executable(capture_stereo src/capture_stereo.cpp ${SOURCES})
target_link_libraries(capture_stereo ${OpenCV_LIBS} cxxopts::cxxopts)

add_executable(calibrate_intrinsic src/calibrate_intrinsic.cpp)
target_link_libraries(calibrate_intrinsic ${OpenCV_LIBS} cxxopts::cxxopts)

add_executable(calibrate_stereo src/calibrate_stereo.cpp)
target_link_libraries(calibrate_stereo ${OpenCV_LIBS} cxxopts::cxxopts)

add_executable(undistort_rectify src/undistort_rectify.cpp)
target_link_libraries(undistort_rectify ${OpenCV_LIBS} cxxopts::cxxopts)