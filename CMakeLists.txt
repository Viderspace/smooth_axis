cmake_minimum_required(VERSION 3.21)
project(smooth_axis C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# headers and sources
include_directories(${CMAKE_SOURCE_DIR}/include)

# Library target
add_library(smooth_axis_lib src/smooth_axis.c)


# Demo / test executable
add_executable(smooth_axis_demo archived/demo_basic.c)
add_executable(smooth_axis_tests
        tests/test_main.c
        tests/smooth_axis_scenario.c
        tests/smooth_axis_pipeline.c
        tests/smooth_axis_time_stub.c src/smooth_axis_debug.h tests/test_debug_logging.c tests/step_response_test.c)

target_link_libraries(smooth_axis_tests PRIVATE smooth_axis_lib)

target_link_libraries(smooth_axis_demo PRIVATE smooth_axis_lib)

enable_testing()
add_test(NAME smooth_axis_tests COMMAND smooth_axis_tests)