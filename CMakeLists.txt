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
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories($(OpenCV_INCLUDE_DIRS))

add_executable(read src/read_images.cpp)
target_link_libraries(read ${OpenCV_LIBS} cxxopts)

add_executable(calibrate src/calib_intrinsic.cpp)
target_link_libraries(calibrate ${OpenCV_LIBS} cxxopts)

# add_executable(calibrate_stereo src/calib_stereo.cpp)
# target_link_libraries(calibrate_stereo ${OpenCV_LIBS})

# add_executable(undistort_rectify src/undistort_rectify.cpp)
# target_link_libraries(undistort_rectify ${OpenCV_LIBS})