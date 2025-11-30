cmake_minimum_required(VERSION 3.21)
project(smooth_axis C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Where we'll put headers and sources
include_directories(${CMAKE_SOURCE_DIR}/include)

# Library target (your actual smooth_axis code)
add_library(smooth_axis_lib src/smooth_axis.c)


# Demo / test executable
add_executable(smooth_axis_demo src/main.c)



target_link_libraries(smooth_axis_demo PRIVATE smooth_axis_lib)