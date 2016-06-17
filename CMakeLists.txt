cmake_minimum_required(VERSION 2.8.3)
project(model_joint_plugin)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED rtt_ros cmake_modules)

find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)
find_package(GAZEBO REQUIRED)
find_package(Protobuf REQUIRED)

if(GAZEBO_VERSION)
  if(${GAZEBO_VERSION} GREATER 6)
      add_definitions(-DGAZEBO_GREATER_6)
  endif()
endif()


set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  proto/joint_state.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)



###########
## Build ##
###########

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
orocos_library(joint_state_msgs SHARED ${PROTO_SRCS})
target_link_libraries(joint_state_msgs ${PROTOBUF_LIBRARY})
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES
                            GENERATED TRUE)
file(COPY
  ${PROTO_HDRS}
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})

include_directories(
  include
  ${PROJECT_BINARY_DIR}/../ # for protobuf
  ${GAZEBO_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(model_joint_plugin
  src/model_joint_plugin.cc
)

orocos_component(gz_model_remote_interface
  src/gz_model_remote_interface.cc
)
target_link_libraries(gz_model_remote_interface
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Protobuf_LIBRARIES}
  joint_state_msgs
  ${orocos_kdl_LIBRARIES}
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(model_joint_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(model_joint_listener_test src/model_joint_listener_test.cc)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(gz_model_remote_interface joint_state_msgs)
add_dependencies(model_joint_plugin joint_state_msgs)
add_dependencies(model_joint_listener_test joint_state_msgs)

## Specify libraries to link a library or executable target against
target_link_libraries(model_joint_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Protobuf_LIBRARIES}
  joint_state_msgs
)
target_link_libraries(model_joint_listener_test
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Protobuf_LIBRARIES}
  joint_state_msgs
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
install(TARGETS model_joint_plugin model_joint_listener_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ${PROJECT_SOURCE_DIR}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  ${PROTO_HDRS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

orocos_install_headers(DIRECTORY include)
orocos_install_headers(${PROTO_HDRS})
orocos_generate_package(INCLUDE_DIRS include)
