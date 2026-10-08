# Standalone build for the RVC system tests.
# The product stays as the two grader-facing files at the repository root;
# this project only compiles them (without main/mock) and links GoogleTest.
#   cmake -S tests/system -B build/system
#   cmake --build build/system
#   ctest --test-dir build/system
cmake_minimum_required(VERSION 3.20)
project(rvc_system_tests LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(RVC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

# Control modules only: RVC_NO_MAIN drops the CLI and the built-in mock device.
add_library(rvc_controller STATIC "${RVC_ROOT}/controller.c")
target_compile_definitions(rvc_controller PRIVATE RVC_NO_MAIN)
target_include_directories(rvc_controller PUBLIC "${RVC_ROOT}")

add_executable(rvc_system_tests rvc_system_tests.cpp)
target_link_libraries(rvc_system_tests PRIVATE rvc_controller GTest::gtest_main)

foreach(target rvc_controller rvc_system_tests)
  if(MSVC)
    # Sources are UTF-8 without BOM; MSVC needs /utf-8 so C4819 does not trip /WX.
    target_compile_options(${target} PRIVATE /W4 /WX /utf-8)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endforeach()

enable_testing()
include(GoogleTest)
gtest_discover_tests(rvc_system_tests TEST_PREFIX "system.")
