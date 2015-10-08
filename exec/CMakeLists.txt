# Set up output directory for binaries. This tells CMAKE where to instruct the
# compiler to build the output executables.
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin/)

# Macro to list all directories in a directory.
macro(subdirlist result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# Loop over the executables and create the targets.
subdirlist(EXECUTABLES ${CMAKE_SOURCE_DIR}/exec)
foreach(EXECUTABLE ${EXECUTABLES})
  message("Including binary \"${BoldMagenta}${EXECUTABLE}${ColorReset}\".")
  file(GLOB_RECURSE LOCAL_SRC ${CMAKE_SOURCE_DIR}/exec/${EXECUTABLE}/*.cpp)
  add_executable(${EXECUTABLE} ${LOCAL_SRC})# $<TARGET_OBJECTS:core>)
  target_link_libraries(${EXECUTABLE} ${CIDER_LIBRARY_DEPENDENCIES})
endforeach(EXECUTABLE)
