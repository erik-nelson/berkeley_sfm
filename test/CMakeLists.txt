# If testing is enabled, build all tests.
if (BUILD_TESTS)
  enable_testing()

  # Set the local gtest include directory.
  set(GTEST_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/gtest")
  include_directories(${gtest_SOURCE_DIR} ${gtest_SOURCE_DIR}/include)

  # Gtest bug fix.
  add_definitions(-DGTEST_HAS_TR1_TUPLE=0)

  # Set a name for the output binary and testing arguments.
  set(test_target run_tests)
  set(test_args --gtest_shuffle)

  # Compile all tests into a single executable and link to berkeley_sfm and gtest.
  file(GLOB test_srcs ${CMAKE_SOURCE_DIR}/test/*.cpp)
  foreach(test ${test_srcs})
    get_filename_component(test_no_ext ${test} NAME_WE)
    message("Including test   \"${BoldBlue}${test_no_ext}${ColorReset}\".")
  endforeach()
  add_executable(${test_target} ${test_srcs})
  target_link_libraries(${test_target} berkeley_sfm ${berkeley_sfm_LIBRARIES} gtest)
  bsfm_default_properties(${test_target})

  # Define a variable storing the path to test data.
  add_definitions(-DBSFM_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test/test_data")
endif(BUILD_TESTS)
