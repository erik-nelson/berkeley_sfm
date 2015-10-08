cmake_minimum_required(VERSION 2.6)
project(berkeley_sfm)

# Build options.
option(BUILD_TESTS "Enabled testing" ON)

# Add a directory for Find*.cmake files.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Check for C++11 features and enable them.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  message(FATAL_ERROR "The compiler (${CMAKE_CXX_COMPILER}) does not support c++11.")
endif()

# Set the build type. Default to Release mode.
if(NOT CMAKE_BUILD_TYPE)
  message("Defaulting to building targets in Release mode.")
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# Add the source directories to the search path. This makes it so that our
# compiler can find all the header files in the src/cpp/ directory.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/cpp/)

# Find OpenCV.
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

# Find Theia.
find_package( Theia REQUIRED )
include_directories(${THEIA_INCLUDE_DIRS})

# Find Eigen.
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

# Find OpenGL.
find_package( OPENGL REQUIRED )
include_directories(${OPENGL_INCLUDE_DIRS})

# Find Glut.
find_package( GLUT REQUIRED )
include_directories(${GLUT_INCLUDE_DIRS})

# Find Gflags.
find_package( Gflags REQUIRED )
include_directories(${GFLAGS_INCLUDE_DIRS})

# Find Glog.
find_package( Glog REQUIRED )
include_directories(${GLOG_INCLUDE_DIRS})

# Find third party libraries.
add_subdirectory(third_party)
include_directories(
  third_party
  ${gtest_SOURCE_DIR}/include
  ${gtest_SOURCE_DIR}
)
#find_package( gtest REQUIRED )
#include_directories(${GTEST_INCLUDE_DIRS})

# Make a variable for all library dependencies.
set(CIDER_LIBRARY_DEPENDENCIES
  ${OpenCV_LIBS}
  ${THEIA_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
)

# Grab all source files and compile them into a library.
#file(GLOB_RECURSE COMMON_SRC
#  ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp/dir1/*.cpp
#  ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp/dir2/*.cpp
#)
#add_library(core OBJECT ${COMMON_SRC})

# Allow colored outputs
if(NOT WIN32)
  string(ASCII 27 Esc)
  set( ColorReset  "${Esc}[m"     )
  set( ColorBold   "${Esc}[1m"    )
  set( Red         "${Esc}[31m"   )
  set( Green       "${Esc}[32m"   )
  set( Yellow      "${Esc}[33m"   )
  set( Blue        "${Esc}[34m"   )
  set( Magenta     "${Esc}[35m"   )
  set( Cyan        "${Esc}[36m"   )
  set( White       "${Esc}[37m"   )
  set( BoldRed     "${Esc}[1;31m" )
  set( BoldGreen   "${Esc}[1;32m" )
  set( BoldYellow  "${Esc}[1;33m" )
  set( BoldBlue    "${Esc}[1;34m" )
  set( BoldMagenta "${Esc}[1;35m" )
  set( BoldCyan    "${Esc}[1;36m" )
  set( BoldWhite   "${Esc}[1;37m" )
endif()

# Add subdirectory for executables.
add_subdirectory(exec)

# Add subdirectory for tests.
add_subdirectory(test)
