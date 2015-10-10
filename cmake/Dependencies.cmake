set(berkeley_sfm_LIBRARIES "")

# Find OpenCV.
find_package( OpenCV REQUIRED )
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${OpenCV_LIBS})

# Find Theia.
find_package( Theia REQUIRED )
include_directories(SYSTEM ${THEIA_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${THEIA_LIBRARIES})

# Find Eigen.
find_package( Eigen3 REQUIRED )
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
list(APPEND berkeley_sfm_LIBRARIES ${EIGEN3_LIBRARIES})

# Find OpenGL.
#find_package( OPENGL REQUIRED )
#include_directories(SYSTEM ${OPENGL_INCLUDE_DIRS})
#list(APPEND berkeley_sfm_LIBRARIES ${OPENGL_LIBRARIES})

# Find Glut.
#find_package( GLUT REQUIRED )
#include_directories(SYSTEM ${GLUT_INCLUDE_DIRS})
#list(APPEND berkeley_sfm_LIBRARIES ${GLUT_LIBRARIES})

# Find Google-gflags.
include("cmake/External/gflags.cmake")
include_directories(SYSTEM ${GFLAGS_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${GFLAGS_LIBRARIES})

# Find Google-glog.
include("cmake/External/glog.cmake")
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
list(APPEND berkeley_sfm_LIBRARIES ${GLOG_LIBRARIES})
