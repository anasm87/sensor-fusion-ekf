cmake_minimum_required(VERSION 3.14)
project(SensorFusionEKF VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Ensure -Wall -Wextra -Werror flags are set (MSVC uses different flags)
if(MSVC)
  add_compile_options(/W4 /WX /wd4127 /wd5054)
else()
  add_compile_options(-Wall -Wextra -Werror)
endif()

include(FetchContent)


# Before calling FetchContent_MakeAvailable, disable Eigen's internal tests
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE) # This stops Eigen from adding 900+ tests

# Fetch Eigen3
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(eigen)
# Re-enable testing for YOUR project only
set(BUILD_TESTING ON CACHE BOOL "" FORCE)
enable_testing()

# Fetch GTest
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.12.1
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Include directories
include_directories(include)

# Create library
add_library(ekf_lib src/kalman_filter.cpp)
target_link_libraries(ekf_lib PUBLIC Eigen3::Eigen)
target_include_directories(ekf_lib PUBLIC include)

# Tests
enable_testing()
add_executable(test_kalman_filter tests/test_kalman_filter.cpp)
target_link_libraries(test_kalman_filter PRIVATE ekf_lib gtest_main)

include(GoogleTest)
gtest_discover_tests(test_kalman_filter)