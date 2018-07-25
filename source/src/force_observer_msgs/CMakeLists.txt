cmake_minimum_required(VERSION 2.8.3)
project(force_observer_msgs)

set(MESSAGE_DEPS
    geometry_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
    COMPONENTS
        genmsg
        message_generation
        ${MESSAGE_DEPS}
)


# Find all the messages files
FILE(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
      "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ${MSG_FILES}
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   ${MESSAGE_DEPS}
)

catkin_package(
  CATKIN_DEPENDS genmsg message_runtime ${MESSAGE_DEPS}
)


