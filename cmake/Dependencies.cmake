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

# Find Flann.
find_package( Flann REQUIRED )
include_directories(SYSTEM ${FLANN_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${FLANN_LIBRARIES})

# Find Google-gflags.
include("cmake/External/gflags.cmake")
include_directories(SYSTEM ${GFLAGS_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${GFLAGS_LIBRARIES})

# Find Google-glog.
include("cmake/External/glog.cmake")
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${GLOG_LIBRARIES})
