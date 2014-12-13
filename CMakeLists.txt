cmake_minimum_required(VERSION 2.8.3)
project(jsk_2014w_rinko)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
#  oculus_rviz_plugins
  pr2eus
  rospy
  tf2_py
  cv_bridge
  message_generation
  std_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

find_package(
  PkgConfig
)
pkg_check_modules(PC_PSMOVEAPI REQUIED psmoveapi)
set(PSMOVEAPI_DEFINITIONS ${PC_PSMOVEAPI_CFLAGS_OTHER})
find_path(PSMOVEAPI_INCLUDE_DIR psmove.h psmove_config.h
  HINTS ${PC_PSMOVEAPI_INCLUDEDIR} ${PC_PSMOVEAPI_INCLUDE_DIRS}
  PATH_SUFFIXES psmoveapi)
find_library(PSMOVEAPI_LIBRARY NAMES psmoveapi
  HINTS ${PC_PSMOVEAPI_LIBDIR} ${PC_PSMOVEAPI_LIBRARY_DIRS})
set(PSMOVEAPI_LIBRARIES ${PSMOVEAPI_LIBRARY})
set(PSMOVEAPI_INCLUDE_DIRS ${PSMOVEAPI_INCLUDE_DIR})
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PSMOVEAPI DEFAULT_MSG PSMOVEAPI_LIBRARY PSMOVEAPI_INCLUDE_DIR)
mark_as_advanced(PSMOVEAPI_INCLUDE_DIR PSMOVEAPI_LIBRARY)

add_message_files(
  FILES PSMoveButton.msg
)

generate_messages(
  DEPENDENCIES std_msgs
  )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 2014w_rinko
#  CATKIN_DEPENDS actionlib oculus_rviz_plugins pr2eus rospy tf2 std_msgs message_runtime
#  DEPENDS system_lib
)
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PSMOVEAPI_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/glm
)

## Declare a cpp library
# add_library(2014w_rinko
#   src/${PROJECT_NAME}/2014w_rinko.cpp
# )

## Declare a cpp executable
add_executable(psmove_tracker src/psmove_tracker.cpp)
add_executable(linear_2d_kalman_filter src/linear_2d_kalman_filter.cpp)
add_executable(linear_3d_kalman_filter src/linear_3d_kalman_filter.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(2014w_rinko_node 2014w_rinko_generate_messages_cpp)
message("hogehoge"${PSMOVEAPI_LIBRARIES})
## Specify libraries to link a library or executable target against

target_link_libraries(psmove_tracker
  ${catkin_LIBRARIES}
  ${PSMOVEAPI_LIBRARIES}
  ${OpenCV_LIBRARIES}
  /usr/lib
)


target_link_libraries(linear_2d_kalman_filter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(linear_3d_kalman_filter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS 2014w_rinko 2014w_rinko_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_2014w_rinko.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
