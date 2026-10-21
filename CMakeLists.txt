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

if(NOT (CMAKE_SYSTEM_NAME STREQUAL "Linux" OR APPLE))
  message(FATAL_ERROR "Supported platforms: Linux (primary) and macOS.")
endif()

# ---------------------------------------------------------------------------
# Sanitizers: -DPTC_SANITIZE=address   -> ASan + UBSan
#             -DPTC_SANITIZE=thread    -> TSan
# ASan and TSan cannot be combined, so they are separate build trees.
# ---------------------------------------------------------------------------
set(PTC_SANITIZE "" CACHE STRING "Sanitizer build: '', 'address', or 'thread'")
set_property(CACHE PTC_SANITIZE PROPERTY STRINGS "" address thread)

add_library(ptc_options INTERFACE)
target_compile_options(ptc_options INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

if(PTC_SANITIZE STREQUAL "address")
  target_compile_options(ptc_options INTERFACE
    -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
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
  src/control/controller.cpp
  src/control/periodic_loop.cpp
  src/control/pid.cpp
  src/io/sensor_task.cpp
  src/platform/rt_thread.cpp
  src/recipe/recipe_machine.cpp
  src/safety/interlocks.cpp
  src/sim/plant.cpp
  src/telemetry/latency_histogram.cpp
  src/telemetry/logger.cpp
  src/telemetry/report.cpp
)
target_include_directories(ptc_core PUBLIC src)
target_link_libraries(ptc_core PUBLIC ptc_options Threads::Threads)

add_executable(ptc src/app/main.cpp src/app/console.cpp)
target_link_libraries(ptc PRIVATE ptc_core)

# ---------------------------------------------------------------------------
# Tests (GoogleTest). Uses a system install if present, otherwise fetches it.
# Offline: -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=<path to googletest checkout>
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
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_executable(ptc_tests
    tests/test_latency_histogram.cpp
    tests/test_periodic_loop.cpp
    tests/test_seqlock_ring.cpp
    tests/test_spsc_queue.cpp
    tests/test_plant.cpp
    tests/test_pid.cpp
    tests/test_guards.cpp
    tests/test_interlocks.cpp
    tests/test_recipe_machine.cpp
    tests/test_system.cpp
    tests/test_logger.cpp
  )
  target_include_directories(ptc_tests PRIVATE tests)
  target_link_libraries(ptc_tests PRIVATE ptc_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(ptc_tests DISCOVERY_TIMEOUT 60)

  # End-to-end fault injection through the real multi-threaded binary:
  # every interlock must fire and the run must end in a safe state.
  function(ptc_fault_test name fault expect)
    set(fault_args "")
    if(NOT fault STREQUAL "none")
      set(fault_args --fault ${fault})
    endif()
    add_test(NAME e2e.${name}
      COMMAND ptc --wafers 1 --duration 40 ${fault_args} --no-mlock --log e2e_${name}.log)
    set_tests_properties(e2e.${name} PROPERTIES
      PASS_REGULAR_EXPRESSION "${expect}" TIMEOUT 60 LABELS e2e)
  endfunction()
  ptc_fault_test(nominal_wafer      none           "wafers processed     1")
  ptc_fault_test(temp_runaway       temp-runaway   "abort->FAULT +CriticalOverTemperature")
  ptc_fault_test(vacuum_leak        vacuum-leak    "abort +OverPressureHeaterOn")
  ptc_fault_test(stuck_valve        stuck-valve    "abort +SlitValveOpenUnderVacuum")
  ptc_fault_test(sensor_dropout     sensor-dropout "FAULT +StaleSensorData")
endif()
