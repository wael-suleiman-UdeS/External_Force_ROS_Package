cmake_minimum_required(VERSION 2.8.3)
project(force_observer)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        cmake_modules
        eigen_conversions
        force_observer_msgs
        sensor_msgs
        geometry_msgs
        std_msgs
        std_srvs
)

find_package(Boost)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/Third-Party/kalman")
SET(KALMAN_LIB_PATH "${CMAKE_SOURCE_DIR}/Third-Party/kalman")
find_package(kalman_lib REQUIRED)
MESSAGE(STATUS "${kalman_lib_INCLUDE_DIR}")

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} DEPENDS kalman_lib_LIBRARY
    DEPENDS
    Eigen
)

set(Kalman_FILES 
	src/kalman_filt.cpp 
	src/kalman_filter.cpp
	src/ekalman_filter.cpp 
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${kalman_INCLUDE_DIR}  ${eigen_INCLUDE_DIRS})


add_library(KalmanFilterlib ${Kalman_FILES})
#define some target ...
target_link_libraries(KalmanFilterlib
    ${catkin_LIBRARIES}
    ${kalman_lib_LIBRARY}
)

set(Observer_FILES 
	src/KalmanObserver_X.cpp
	src/KalmanObserver_Z.cpp
	src/KalmanForceObserver.cpp
	src/KalmanForceObserverNode.cpp
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${kalman_INCLUDE_DIR} ${eigen_INCLUDE_DIRS})



add_library(${PROJECT_NAME}
    ${Observer_FILES}
)
add_dependencies(${PROJECT_NAME} force_observer_msgs_generate_messages_cpp KalmanFilterlib)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    KalmanFilterlib
)

add_executable(kalman_observer_node src/kalman_force_observer.cpp)
add_dependencies(kalman_observer_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(kalman_observer_node
    ${PROJECT_NAME}
)


# install
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
) 

