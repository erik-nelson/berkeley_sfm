# If testing is enabled, build all tests.
if (BUILD_TESTS)
 enable_testing()
 add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=1)
 set(GTEST_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third_party/gtest")

 # Loop over the tests and create the targets.
 file(GLOB TESTS ${CMAKE_SOURCE_DIR}/test/*.cpp)
 foreach(TEST ${TESTS})
   get_filename_component(TEST_WE ${TEST} NAME_WE) # (W)ithout (E)xtension.
   message("Including test   \"${BoldBlue}${TEST_WE}${ColorReset}\".")
   add_executable(${TEST_WE} ${CMAKE_SOURCE_DIR}/test/${TEST_WE}.cpp)
   target_link_libraries(${TEST_WE}
     ${CIDER_LIBRARY_DEPENDENCIES}
     ${GTEST_BOTH_LIBRARIES}
   )
   #add_test(NAME ${TEST_WE} COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TEST_WE})
 endforeach(TEST)
endif(BUILD_TESTS)
