cmake_minimum_required(VERSION 2.8.7)
project(berkeley_sfm C CXX)

# Build options.
option(BUILD_TESTS "Build tests" ON)
option(BUILD_DOCUMENTATION "Build documentation" ON)

# Add cmake modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)
message("Cmake module path: ${CMAKE_MODULE_PATH}")

# Add cmake utilities.
include(cmake/Utils.cmake)

# Add dependencies.
include(cmake/Dependencies.cmake)

# Check for C++11 features and enable.
bsfm_enable_cpp11()

# Set the build type. Default to Release mode.
if(NOT CMAKE_BUILD_TYPE)
  message("Defaulting to building targets in Release mode.")
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# Find and build third party libraries.
add_subdirectory(external)

# Add the source directories to the search path so cmake can find our headers.
include_directories(${CMAKE_SOURCE_DIR}/src/cpp/)

# Collect source files and build the berkeley_sfm object.
file(GLOB_RECURSE bsfm_srcs ${CMAKE_SOURCE_DIR}/src/cpp/*.cpp)
add_library(berkeley_sfm ${bsfm_srcs})

# Set up output directory for executables.
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin/)

# Find and build executables.
#add_subdirectory(exec)

# Find and build tests.
if (BUILD_TESTS)
  message("Build tests is enabled.")
  add_subdirectory(test)
endif (BUILD_TESTS)

# Find and build documentation.
if (BUILD_DOCUMENTATION)
  message("Build documentation is enabled.")
  add_subdirectory(documentation)
endif (BUILD_DOCUMENTATION)
