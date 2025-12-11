cmake_minimum_required(VERSION 3.21)
project(smooth_axis C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/src)

# Library target
add_library(smooth_axis_lib src/smooth_axis.c)

# Test executables (individual, matching Makefile structure)
add_executable(ramp_test
        tests/c_tests/ramp_response_test.c
        src/smooth_axis.c)

add_executable(step_test
        tests/c_tests/step_response_test.c
        src/smooth_axis.c)

add_executable(test_api
        tests/c_tests/test_api_sanity_enhanced.c
        src/smooth_axis.c)

# Link math library to all tests
target_link_libraries(ramp_test PRIVATE m)
target_link_libraries(step_test PRIVATE m)
target_link_libraries(test_api PRIVATE m)

# Set release mode for test_api (matches Makefile: -DNDEBUG)
target_compile_definitions(test_api PRIVATE NDEBUG)

# Enable testing
enable_testing()

# Add tests with proper working directory (project root)
add_test(NAME ramp_test COMMAND ramp_test)
set_tests_properties(ramp_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_test(NAME step_test COMMAND step_test)
set_tests_properties(step_test PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

add_test(NAME test_api COMMAND test_api)
set_tests_properties(test_api PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# Create test data directories in build tree (optional, for convenience)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/tests/data/ramp_files)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/tests/data/step_files)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/tests/data/renders)