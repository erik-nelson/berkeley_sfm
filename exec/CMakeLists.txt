# Loop over the executables and create the targets.
bsfm_subdir_list(executables ${CMAKE_SOURCE_DIR}/exec)
foreach(executable ${executables})
  message("Including binary \"${BoldMagenta}${executable}${ColorReset}\".")
  file(GLOB_RECURSE local_src ${CMAKE_SOURCE_DIR}/exec/${executable}/*.cpp)
  add_executable(${executable} ${local_src})
  target_link_libraries(${executable} berkeley_sfm ${bsfm_LINKER_LIBS})
endforeach()
