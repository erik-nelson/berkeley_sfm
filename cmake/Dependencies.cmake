include(ExternalProject)
set(berkeley_sfm_LIBRARIES "")

# Find OpenCV.
find_package( OpenCV REQUIRED )
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${OpenCV_LIBS})

# Find Eigen.
find_package( Eigen3 REQUIRED )
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
list(APPEND berkeley_sfm_LIBRARIES ${EIGEN3_LIBRARIES})

# Find Ceres.
find_package( Ceres REQUIRED )
include_directories(SYSTEM ${CERES_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${CERES_LIBRARIES})

# Find Google-gflags.
include("cmake/External/gflags.cmake")
include_directories(SYSTEM ${GFLAGS_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${GFLAGS_LIBRARIES})

# Find Google-glog.
include("cmake/External/glog.cmake")
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${GLOG_LIBRARIES})

# Find OpenMP.
find_package( OpenMP REQUIRED )
if (OpenMP_FOUND)
  list(APPEND CMAKE_C_FLAGS ${OpenMP_C_FLAGS})
  list(APPEND CMAKE_CXX_FLAGS ${OpenMP_CXX_FLAGS})
  list(APPEND CMAKE_EXE_LINKER_FLAGS ${OpenMP_EXE_LINKER_FLAGS})
endif()
