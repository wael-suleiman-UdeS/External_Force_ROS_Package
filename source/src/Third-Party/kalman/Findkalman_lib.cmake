SET(KALMAN_HEADERS ekfilter.hpp ekfilter_impl.hpp kfilter.hpp kfilter_impl.hpp kmatrix.hpp kmatrix_impl.hpp ktypes.hpp kvector.hpp kvector_impl.hpp)

FIND_PATH(kalman_INCLUDE_DIR ${KALMAN_HEADERS} ${KALMAN_LIB_PATH}/kalman)

FIND_LIBRARY(kalman_lib_LIBRARY
    NAMES kalman.a
    PATHS ${KALMAN_LIB_PATH})

IF (kalman_INCLUDE_DIR AND kalman_lib_LIBRARY)
   SET(kalman_lib_FOUND TRUE)
ENDIF (kalman_INCLUDE_DIR AND kalman_lib_LIBRARY)

IF (kalman_lib_FOUND)
   IF (NOT kalman_lib_FIND_QUIETLY)
      MESSAGE(STATUS "Found kalman_lib: ${kalman_lib_LIBRARY}")
      SET(kalman_lib_INCLUDE_DIR ${kalman_INCLUDE_DIR})
   ENDIF (NOT kalman_lib_FIND_QUIETLY)
ELSE (kalman_lib_FOUND)
   IF (kalman_lib_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find kalman_lib")
   ENDIF (kalman_lib_FIND_REQUIRED)
ENDIF (kalman_lib_FOUND)

