cmake_minimum_required(VERSION 3.20)
project(process_tool_controller LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Language standard: C++17, no extensions, everywhere.
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "This project is Linux-only (uses pthread scheduling, ppoll, prctl).")
endif()

# ---------------------------------------------------------------------------
# Sanitizers: -DPTC_SANITIZE=address   -> ASan + UBSan
#             -DPTC_SANITIZE=thread    -> TSan
# ASan and TSan cannot be combined, so they are separate build trees.
# ---------------------------------------------------------------------------
set(PTC_SANITIZE "" CACHE STRING "Sanitizer build: '', 'address', or 'thread'")
set_property(CACHE PTC_SANITIZE PROPERTY STRINGS "" address thread)

add_library(ptc_options INTERFACE)
target_compile_options(ptc_options INTERFACE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

if(PTC_SANITIZE STREQUAL "address")
  target_compile_options(ptc_options INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
  target_link_options(ptc_options INTERFACE -fsanitize=address,undefined)
elseif(PTC_SANITIZE STREQUAL "thread")
  target_compile_options(ptc_options INTERFACE -fsanitize=thread -fno-omit-frame-pointer)
  target_link_options(ptc_options INTERFACE -fsanitize=thread)
elseif(NOT PTC_SANITIZE STREQUAL "")
  message(FATAL_ERROR "PTC_SANITIZE must be '', 'address' or 'thread'")
endif()

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Core library: everything except main(). Tests link against this.
# ---------------------------------------------------------------------------
add_library(ptc_core STATIC
  src/control/control_loop.cpp
  src/platform/rt_thread.cpp
  src/telemetry/latency_histogram.cpp
  src/telemetry/report.cpp
)
target_include_directories(ptc_core PUBLIC src)
target_link_libraries(ptc_core PUBLIC ptc_options Threads::Threads)

add_executable(ptc src/app/main.cpp src/app/console.cpp)
target_link_libraries(ptc PRIVATE ptc_core)

# ---------------------------------------------------------------------------
# Tests (GoogleTest). Uses a system install if present, otherwise fetches it.
# ---------------------------------------------------------------------------
option(PTC_BUILD_TESTS "Build unit tests" ON)
if(PTC_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.15.2
      GIT_SHALLOW TRUE)
    # Offline / multiple build trees: -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=<path>
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_executable(ptc_tests
    tests/test_latency_histogram.cpp
    tests/test_control_loop.cpp
  )
  target_link_libraries(ptc_tests PRIVATE ptc_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(ptc_tests)
endif()
